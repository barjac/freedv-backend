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

// Look-ahead window. Gain reduction ramps in over this time ahead of each
// peak, so the gain is fully down when the peak reaches the output. 4ms
// is just over one period of the lowest voice frequencies (~300Hz =
// 3.3ms): a gain change within a single cycle would amplitude-modulate the
// waveform and generate harmonics, much like clipping. Adds this much
// fixed latency to TX audio, negligible next to RADE's own latency.
constexpr float LOOKAHEAD_TIME_SEC = 0.004f;
constexpr float RELEASE_TIME_SEC = 0.25f;

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
    , windowLength_(std::max(2, (int)std::lround(sampleRate * LOOKAHEAD_TIME_SEC)))
    , delayBuffer_(std::make_unique<float[]>(windowLength_ - 1))
    , holdValues_(std::make_unique<float[]>(windowLength_))
    , holdIndices_(std::make_unique<int64_t[]>(windowLength_))
    , averageBuffer_(std::make_unique<float[]>(windowLength_))
{
    assert(delayBuffer_ != nullptr && holdValues_ != nullptr && holdIndices_ != nullptr && averageBuffer_ != nullptr);

    // Pre-allocate buffers so we don't have to do so during real-time operation.
    outputSamples_ = std::make_unique<short[]>(sampleRate_);
    assert(outputSamples_ != nullptr);

    inputHeadroomScale_ = powf(10.0f, INPUT_HEADROOM_DB / 20.0f);

    // One-pole release coefficient: alpha = 1 - exp(-dt/tau), dt = 1 sample.
    releaseAlpha_ = 1.0f - expf(-1.0f / (sampleRate_ * RELEASE_TIME_SEC));

    reset();
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

float CompressorLimiterStep::slidingMinimum_(float value) FREEDV_NONBLOCKING
{
    // Monotonic queue: values increase from front to back, so the front is
    // the minimum of the current window. Each entry is pushed and popped at
    // most once, so this is O(1) amortized per sample.
    if (holdCount_ > 0 && holdIndices_[holdHead_] <= sampleIndex_ - windowLength_)
    {
        holdHead_ = (holdHead_ + 1) % windowLength_;
        holdCount_--;
    }
    while (holdCount_ > 0)
    {
        int back = (holdHead_ + holdCount_ - 1) % windowLength_;
        if (holdValues_[back] < value) break;
        holdCount_--;
    }
    int tail = (holdHead_ + holdCount_) % windowLength_;
    holdValues_[tail] = value;
    holdIndices_[tail] = sampleIndex_;
    holdCount_++;
    sampleIndex_++;

    return holdValues_[holdHead_];
}

short* CompressorLimiterStep::execute(short* inputSamples, int numInputSamples, int* numOutputSamples) FREEDV_NONBLOCKING
{
    int tenMsSamples = std::max(1, sampleRate_ / TEN_MS_DIVIDER);
    const float kneeStartDb = LIMITER_THRESHOLD_DB - LIMITER_KNEE_WIDTH_DB / 2.0f;

    short* inPtr = inputSamples;
    short* outPtr = outputSamples_.get();
    *numOutputSamples = numInputSamples;

    int remaining = numInputSamples;
    while (remaining > 0)
    {
        int chunkSize = std::min(remaining, tenMsSamples);
        double peakOutAbs = 0.0;
        float minGainThisChunk = 1.0f;

        for (int i = 0; i < chunkSize; i++)
        {
            float currentSample = 0.0f;
            ConvertSingleSampleToFloatSampleType_<float, short>(&inPtr[i], &currentSample);
            currentSample *= inputHeadroomScale_; // restore true level (see INPUT_HEADROOM_DB)

            // Step 1: gain this sample needs, from the static soft-knee curve.
            float requiredGain = 1.0f;
            float absVal = std::fabs(currentSample);
            float levelDb = absVal > 0.0f ? 20.0f * log10f(absVal) : LEVEL_FLOOR_DB;
            if (levelDb > kneeStartDb)
            {
                float kneeOutDb = softKneeGainDb(levelDb, LIMITER_THRESHOLD_DB, LIMITER_RATIO, LIMITER_KNEE_WIDTH_DB);
                requiredGain = powf(10.0f, (kneeOutDb - levelDb) / 20.0f);
            }

            // Step 2: lowest required gain over the look-ahead window, so a
            // peak is accounted for for the whole time it spends in the
            // delay line.
            float heldGain = slidingMinimum_(requiredGain);

            // Step 3: release. Follow reductions immediately (the ramp comes
            // from step 4), recover slowly toward unity.
            if (heldGain < releasedGain_)
            {
                releasedGain_ = heldGain;
            }
            else
            {
                releasedGain_ += (heldGain - releasedGain_) * releaseAlpha_;
            }

            // Step 4: moving average over the same window length, giving a
            // smooth ramp into each reduction. With the delay below being
            // one sample shorter than the window, every value averaged for
            // an output sample includes that sample in its hold window, so
            // the applied gain is never more than the sample requires.
            averageSum_ += releasedGain_ - averageBuffer_[averagePos_];
            averageBuffer_[averagePos_] = releasedGain_;
            averagePos_ = (averagePos_ + 1) % windowLength_;
            float gain = std::min(1.0f, (float)(averageSum_ / windowLength_));
            if (gain < minGainThisChunk) minGainThisChunk = gain;

            // Step 5: delay line (windowLength_ - 1 samples) -- read the
            // oldest sample, then overwrite that slot with the newest.
            float delayedSample = delayBuffer_[delayPos_];
            delayBuffer_[delayPos_] = currentSample;
            delayPos_ = (delayPos_ + 1) % (windowLength_ - 1);

            // Step 6: apply gain. The int16 saturation in the conversion
            // remains only as a last-resort backstop.
            float outSampleFloat = delayedSample * gain;
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

        // No-op unless built with ENABLE_AUDIO_DIAG_LOGGING. Logs the
        // deepest gain reduction within the chunk.
        double outputDbfs = peakOutAbs > 0.0 ? 20.0 * std::log10(peakOutAbs) : -100.0;
        diagLogger_->logCompressorLimiterHalfAndFlush(20.0 * std::log10((double)minGainThisChunk), outputDbfs);

        inPtr += chunkSize;
        outPtr += chunkSize;
        remaining -= chunkSize;
    }

    return outputSamples_.get();
}

void CompressorLimiterStep::reset() FREEDV_NONBLOCKING
{
    for (int i = 0; i < windowLength_ - 1; i++)
    {
        delayBuffer_[i] = 0.0f;
    }
    delayPos_ = 0;

    holdHead_ = 0;
    holdCount_ = 0;
    sampleIndex_ = 0;

    releasedGain_ = 1.0f;

    for (int i = 0; i < windowLength_; i++)
    {
        averageBuffer_[i] = 1.0f;
    }
    averageSum_ = windowLength_;
    averagePos_ = 0;

    // No-op -- see LoudnessMeter::reset().
    loudnessMeter_.reset();
}
