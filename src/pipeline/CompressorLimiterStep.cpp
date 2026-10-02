//=========================================================================
// Name:            CompressorLimiterStep.cpp
// Purpose:         Describes a soft-knee compressor/limiter step in the
//                  audio pipeline.
//
// Authors:         Claude Code (for Barry Jackson, G4MKT), design from
//                  g4dya (Richard)'s spec on PR #1472
// License:
//
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// - Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//
// - Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER
// OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//=========================================================================

#include <algorithm>
#include <cassert>
#include <cmath>

#include "CompressorLimiterStep.h"

// Limiter knee: ceiling just below full scale (similar to the -1 to -2dBFS
// limiter level used previously), high ratio, narrow but still soft knee.
constexpr float LIMITER_THRESHOLD_DB = -1.5f;
constexpr float LIMITER_RATIO = 20.0f;
constexpr float LIMITER_KNEE_WIDTH_DB = 2.0f;

// Attack is held at roughly one period of the lowest voice frequencies
// (~300Hz = 3.3ms) rather than <=1ms: a gain command that moves within a
// single cycle amplitude-modulates the waveform and generates harmonics,
// much like clipping.
constexpr float ATTACK_TIME_SEC = 0.003f;
constexpr float RELEASE_TIME_SEC = 0.25f;

// Look-ahead lets gain reduction start before the corresponding (delayed)
// peak reaches the output. Adds this much fixed latency to TX audio, which
// is negligible next to RADE's own latency and the leveler's time
// constants. Roughly matches ATTACK_TIME_SEC.
constexpr float LOOKAHEAD_TIME_SEC = 0.004f;

constexpr float LEVEL_FLOOR_DB = -120.0f; // for log10(0) avoidance
constexpr int TEN_MS_DIVIDER = 100;

// Silence floors for the output loudness measurement (see the
// constructor's comment in the header). Digital silence is always
// rejected regardless of floor.
constexpr double SILENCE_FLOOR_LUFS_RNNOISE_ON = -70.0;
constexpr double SILENCE_FLOOR_LUFS_RNNOISE_OFF = -85.0;

namespace {

// Standard soft-knee compressor curve (Giannoulis, Massberg & Reiss,
// "Digital Dynamic Range Compressor Design", JAES 2012). Returns the
// output level (dB) for a given input level (dB).
float softKneeGainDb(float levelDb, float thresholdDb, float ratio, float kneeWidthDb)
{
    float overshoot = levelDb - thresholdDb;
    if (2.0f * overshoot < -kneeWidthDb)
    {
        // Below the knee -- no change.
        return levelDb;
    }
    else if (2.0f * std::fabs(overshoot) <= kneeWidthDb)
    {
        // Inside the knee -- quadratic transition.
        float kneeTerm = overshoot + kneeWidthDb / 2.0f;
        return levelDb + (1.0f / ratio - 1.0f) * (kneeTerm * kneeTerm) / (2.0f * kneeWidthDb);
    }
    else
    {
        // Above the knee -- straight-line compression at the given ratio.
        return thresholdDb + overshoot / ratio;
    }
}

} // namespace

std::atomic<float> CompressorLimiterStep::lastOutputLoudnessLufs_{-100.0f};

CompressorLimiterStep::CompressorLimiterStep(int sampleRate, std::shared_ptr<DiagnosticCsvLogger> diagLogger,
                                              realtime_fp<bool()> const& noiseReductionEnabledFn)
    : sampleRate_(sampleRate)
    , loudnessMeter_(sampleRate)
    , diagLogger_(diagLogger)
    , noiseReductionEnabledFn_(noiseReductionEnabledFn)
    , lookAheadLength_(std::max(1, (int)std::lround(sampleRate * LOOKAHEAD_TIME_SEC)))
    , lookAheadBuffer_(std::make_unique<float[]>(lookAheadLength_)) // value-initialized (zeroed)
    , lookAheadPos_(0)
    , smoothedGainReductionDb_(0.0f)
{
    assert(lookAheadBuffer_ != nullptr);

    // Pre-allocate buffers so we don't have to do so during real-time operation.
    outputSamples_ = std::make_unique<short[]>(sampleRate_);
    assert(outputSamples_ != nullptr);

    // One-pole smoothing coefficients: alpha = 1 - exp(-dt/tau), dt = 1 sample.
    inputHeadroomScale_ = powf(10.0f, INPUT_HEADROOM_DB / 20.0f);

    float dt = 1.0f / sampleRate_;
    attackAlpha_ = 1.0f - expf(-dt / ATTACK_TIME_SEC);
    releaseAlpha_ = 1.0f - expf(-dt / RELEASE_TIME_SEC);
}

CompressorLimiterStep::~CompressorLimiterStep()
{
    // empty
}

int CompressorLimiterStep::getInputSampleRate() const FREEDV_NONBLOCKING
{
    return sampleRate_;
}

int CompressorLimiterStep::getOutputSampleRate() const FREEDV_NONBLOCKING
{
    return sampleRate_;
}

float CompressorLimiterStep::getLastOutputLoudnessLufs() FREEDV_NONBLOCKING
{
    return lastOutputLoudnessLufs_.load(std::memory_order_relaxed);
}

short* CompressorLimiterStep::execute(short* inputSamples, int numInputSamples, int* numOutputSamples) FREEDV_NONBLOCKING
{
    int tenMsSamples = std::max(1, sampleRate_ / TEN_MS_DIVIDER);

    short* inPtr = inputSamples;
    short* outPtr = outputSamples_.get();
    *numOutputSamples = numInputSamples;

    int remaining = numInputSamples;
    while (remaining > 0)
    {
        int chunkSize = std::min(remaining, tenMsSamples);
        double peakOutAbs = 0.0;

        for (int i = 0; i < chunkSize; i++)
        {
            float currentSample = 0.0f;
            ConvertSingleSampleToFloatSampleType_<float, short>(&inPtr[i], &currentSample);
            currentSample *= inputHeadroomScale_; // restore true level (see INPUT_HEADROOM_DB)

            // Step 1: envelope detection on the undelayed signal, so that
            // with the look-ahead delay below, gain starts dropping before
            // the peak itself reaches the output.
            float absVal = std::fabs(currentSample);
            float levelDb = absVal > 0.0f ? 20.0f * log10f(absVal) : LEVEL_FLOOR_DB;

            // Step 2: static soft-knee curve.
            float kneeOutDb = softKneeGainDb(levelDb, LIMITER_THRESHOLD_DB, LIMITER_RATIO, LIMITER_KNEE_WIDTH_DB);
            float staticGainReductionDb = kneeOutDb - levelDb; // <= 0

            // Step 3: smooth the gain-reduction command -- fast attack while
            // reduction is increasing, slow release back toward unity. This
            // stage never applies gain above unity.
            float alpha = (staticGainReductionDb < smoothedGainReductionDb_) ? attackAlpha_ : releaseAlpha_;
            smoothedGainReductionDb_ += (staticGainReductionDb - smoothedGainReductionDb_) * alpha;

            // Step 4: look-ahead delay line (read the oldest sample, then
            // overwrite that slot with the newest).
            float delayedSample = lookAheadBuffer_[lookAheadPos_];
            lookAheadBuffer_[lookAheadPos_] = currentSample;
            lookAheadPos_++;
            if (lookAheadPos_ >= lookAheadLength_) lookAheadPos_ = 0;

            // Step 5: apply gain to the delayed sample. The int16 saturation
            // in the conversion remains only as a last-resort backstop.
            float scaleFactor = expf(smoothedGainReductionDb_ / 20.0f * logf(10.0f));
            float outSampleFloat = delayedSample * scaleFactor;
            ConvertSingleSampleToIntSampleType_<short, float>(&outSampleFloat, &outPtr[i]);

            double outAbs = std::fabs((double)outPtr[i]) / 32768.0;
            if (outAbs > peakOutAbs) peakOutAbs = outAbs;
        }

        // Measure this chunk's output loudness for LevelerStep's feedback.
        // Publish -100 on silence/no reading rather than leaving a stale
        // value in place, so LevelerStep holds its gain.
        loudnessMeter_.addFrames(outPtr, chunkSize);
        double lufs = 0.0;
        double silenceFloorLufs = noiseReductionEnabledFn_() ? SILENCE_FLOOR_LUFS_RNNOISE_ON : SILENCE_FLOOR_LUFS_RNNOISE_OFF;
        if (loudnessMeter_.getMomentaryLoudness(&lufs, silenceFloorLufs))
        {
            lastOutputLoudnessLufs_.store((float)lufs, std::memory_order_relaxed);
        }
        else
        {
            lastOutputLoudnessLufs_.store(-100.0f, std::memory_order_relaxed);
        }

        // No-op unless built with ENABLE_AUDIO_DIAG_LOGGING.
        double outputDbfs = peakOutAbs > 0.0 ? 20.0 * std::log10(peakOutAbs) : -100.0;
        diagLogger_->logCompressorLimiterHalfAndFlush(smoothedGainReductionDb_, outputDbfs);

        inPtr += chunkSize;
        outPtr += chunkSize;
        remaining -= chunkSize;
    }

    return outputSamples_.get();
}

void CompressorLimiterStep::reset() FREEDV_NONBLOCKING
{
    smoothedGainReductionDb_ = 0.0f;
    lookAheadPos_ = 0;
    for (int i = 0; i < lookAheadLength_; i++)
    {
        lookAheadBuffer_[i] = 0.0f;
    }

    // No-op -- see LoudnessMeter::reset().
    loudnessMeter_.reset();
}
