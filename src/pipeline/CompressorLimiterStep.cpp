//=========================================================================
// Name:            CompressorLimiterStep.cpp
// Purpose:         Describes a soft-knee compressor/limiter step in the
//                  audio pipeline.
//
// Authors:         Claude Code (for Barry Jones, G4MKT), design from
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

// Knee 1 ("compressor"): starting recommendation only, tune via live A/B
// testing. Pushed close to the ceiling rather than spread across a wide
// range -- Important constraint (Barry, 2026-09-15): RADE's neural encoder
// was almost certainly trained on uncompressed speech, so this threshold
// must sit well above where normal/loud (but not clipping-adjacent) speech
// actually peaks, so this stage effectively never engages during ordinary
// speech -- verify empirically that normal speech shows ~0dB reduction
// here, and raise the threshold further if real testing shows otherwise.
constexpr float KNEE1_THRESHOLD_DB = -6.0f;
constexpr float KNEE1_RATIO = 2.0f;
constexpr float KNEE1_WIDTH_DB = 6.0f;

// Knee 2 ("limiter"): reuses AgcStep's old ~-1 to -2dBFS ceiling precedent,
// near-infinite ratio, tighter knee for a sharper (but still soft, per
// spec) transition into brickwall-like behavior right at the ceiling.
constexpr float KNEE2_THRESHOLD_DB = -1.5f;
constexpr float KNEE2_RATIO = 20.0f;
constexpr float KNEE2_WIDTH_DB = 2.0f;

// Important constraint (Barry, 2026-09-15): a fast limiter risks generating
// its own harmonic distortion if the gain-reduction command moves within
// less than one cycle of the audio it's acting on (amplitude-modulates the
// waveform, like clipping). Richard's spec's literal "attack <=1ms" is fast
// enough to distort the low end of the voice band (~300Hz has a ~3.3ms
// period). Floored at ~one low-frequency cycle instead -- starting
// recommendation, tune via live A/B testing.
constexpr float ATTACK_TIME_SEC = 0.003f;
constexpr float RELEASE_TIME_SEC = 0.25f; // per spec, ~250ms

// Short look-ahead (Barry, 2026-09-15, adopted for v1 to address the
// harmonics concern above): lets the gain begin dropping before the
// corresponding (delayed) peak reaches the output, rather than only
// reacting after it. Cheap CPU-wise (a small ring buffer, no filtering/
// resampling) but adds this much fixed latency to the live TX audio path
// (and, via the feedback loop, to LevelerStep's loudness reading -- both
// negligible next to EBU R128's own ~400ms latency and the leveler's
// multi-second time constant). Roughly matches ATTACK_TIME_SEC above.
constexpr float LOOKAHEAD_TIME_SEC = 0.004f;

constexpr float LEVEL_FLOOR_DB = -120.0f; // for log10(0) avoidance
constexpr int TEN_MS_DIVIDER = 100;

namespace {

// Standard two-parameter soft-knee compressor curve (Giannoulis/Massberg/
// Reiss). Returns the *output* level (dB) for a given *input* level (dB);
// caller subtracts input from the result to get the gain-reduction amount.
float softKneeGainDb(float levelDb, float thresholdDb, float ratio, float kneeWidthDb)
{
    float overshoot = levelDb - thresholdDb;
    if (2.0f * overshoot < -kneeWidthDb)
    {
        // Below the knee entirely -- no change.
        return levelDb;
    }
    else if (2.0f * std::fabs(overshoot) <= kneeWidthDb)
    {
        // Inside the knee -- smooth quadratic transition.
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

CompressorLimiterStep::CompressorLimiterStep(int sampleRate, std::shared_ptr<DiagnosticCsvLogger> diagLogger)
    : sampleRate_(sampleRate)
    , loudnessMeter_(sampleRate)
    , diagLogger_(diagLogger)
    , lookAheadLength_(std::max(1, (int)std::lround(sampleRate * LOOKAHEAD_TIME_SEC)))
    , lookAheadBuffer_(std::make_unique<float[]>(lookAheadLength_)) // value-initialized (zeroed)
    , lookAheadPos_(0)
    , smoothedGainReductionDb_(0.0f)
{
    assert(lookAheadBuffer_ != nullptr);

    // Pre-allocate buffers so we don't have to do so during real-time operation.
    outputSamples_ = std::make_unique<short[]>(sampleRate_);
    assert(outputSamples_ != nullptr);

    // Precompute one-pole smoothing coefficients (fixed since sampleRate_
    // is fixed at construction): alpha = 1 - exp(-dt/tau), dt = 1 sample.
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

            // Step 1: per-sample envelope detection on the *pre-delay*
            // signal -- combined with the look-ahead delay line below,
            // this lets the gain start dropping before the corresponding
            // (delayed) peak reaches the output, without needing a more
            // expensive windowed peak scan.
            float absVal = std::fabs(currentSample);
            float levelDb = absVal > 0.0f ? 20.0f * log10f(absVal) : LEVEL_FLOOR_DB;

            // Step 2: static two-knee soft-knee gain curve, applied in series.
            float knee1OutDb = softKneeGainDb(levelDb, KNEE1_THRESHOLD_DB, KNEE1_RATIO, KNEE1_WIDTH_DB);
            float knee2OutDb = softKneeGainDb(knee1OutDb, KNEE2_THRESHOLD_DB, KNEE2_RATIO, KNEE2_WIDTH_DB);
            float staticGainReductionDb = knee2OutDb - levelDb; // <= 0

            // Step 3: smooth the *gain-reduction command* (not the level),
            // fast attack when reduction is increasing, slow release when
            // recovering toward unity -- only ever attenuates (spec:
            // "maximum gain for this stage is unity").
            float alpha = (staticGainReductionDb < smoothedGainReductionDb_) ? attackAlpha_ : releaseAlpha_;
            smoothedGainReductionDb_ += (staticGainReductionDb - smoothedGainReductionDb_) * alpha;

            // Step 4: look-ahead delay line (read-then-write same slot --
            // lookAheadBuffer_[lookAheadPos_] holds the sample from
            // lookAheadLength_ samples ago; overwrite it with the new
            // sample once read, then advance).
            float delayedSample = lookAheadBuffer_[lookAheadPos_];
            lookAheadBuffer_[lookAheadPos_] = currentSample;
            lookAheadPos_++;
            if (lookAheadPos_ >= lookAheadLength_) lookAheadPos_ = 0;

            // Step 5: apply smoothed gain to the delayed sample, convert
            // back to int16 -- the existing int16 saturate becomes a true
            // last-resort backstop, not the primary (broken) limiting
            // mechanism it was in AgcStep/WebRtcAgc_Process.
            float scaleFactor = expf(smoothedGainReductionDb_ / 20.0f * logf(10.0f));
            float outSampleFloat = delayedSample * scaleFactor;
            ConvertSingleSampleToIntSampleType_<short, float>(&outSampleFloat, &outPtr[i]);

            double outAbs = std::fabs((double)outPtr[i]) / 32768.0;
            if (outAbs > peakOutAbs) peakOutAbs = outAbs;
        }

        // Feed this chunk's actual output into the loudness meter --
        // LevelerStep's feedback loop (see getLastOutputLoudnessLufs())
        // reads whatever this stores below.
        loudnessMeter_.addFrames(outPtr, chunkSize);
        double lufs = 0.0;
        if (loudnessMeter_.getMomentaryLoudness(&lufs))
        {
            lastOutputLoudnessLufs_.store((float)lufs, std::memory_order_relaxed);
        }
        else
        {
            // Silence/gated -- explicitly signal "invalid" rather than
            // leaving a stale loud reading in place, so LevelerStep
            // correctly freezes its own gain (see LevelerStep::execute()'s
            // SILENCE_THRESHOLD_LUFS check).
            lastOutputLoudnessLufs_.store(-100.0f, std::memory_order_relaxed);
        }

        // DIAGNOSTIC ONLY (no-op unless built with ENABLE_AUDIO_DIAG_LOGGING).
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

    // No-op -- see LoudnessMeter::reset()'s header comment (ebur128 state
    // can't be cleared without reallocating, which reset() -- declared
    // FREEDV_NONBLOCKING -- must not do).
    loudnessMeter_.reset();
}
