//=========================================================================
// Name:            LevelerStep.cpp
// Purpose:         Describes a loudness leveler step in the audio pipeline.
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

#include "LevelerStep.h"
#include "CompressorLimiterStep.h"

constexpr float LEVELER_GAIN_LIMIT_DB = 12.0f; // symmetric +/-12dB
static_assert(CompressorLimiterStep::INPUT_HEADROOM_DB >= LEVELER_GAIN_LIMIT_DB,
              "limiter input headroom must cover the leveler's maximum gain");

// Smoothing time constant for current gain moving toward target gain.
// Symmetric (same rise and fall), per the leveler spec.
constexpr float LEVELER_TIME_CONSTANT_SEC = 2.0f;

// PI controller. The measured loudness already includes the gain being
// applied, so a proportional term on its own settles at only half the
// required correction; the integral term removes that remaining error so
// the output converges on the target. The integral time constant is longer
// than LEVELER_TIME_CONSTANT_SEC because its job is only to remove that
// persistent offset, not to react quickly.
constexpr float LEVELER_KP = 1.0f;
constexpr float LEVELER_INTEGRAL_TIME_CONSTANT_SEC = 4.0f;

// Feedback at or below this is treated as a pause in speech and gain is
// held. The RNNoise-on/off values are kept separate so they can be tuned
// independently; measured room noise with RNNoise off (~-34 LUFS) put the
// off value at the same level as the on value for now.
constexpr float SILENCE_THRESHOLD_LUFS_RNNOISE_ON = -33.0f;
constexpr float SILENCE_THRESHOLD_LUFS_RNNOISE_OFF = -33.0f;

// Startup ramp-in. When the leveler is seeded with a saved gain, applying
// it in full on the first syllable can push that syllable into clipping
// before the limiter's envelope has anything to react to. The applied
// gain is therefore ramped in over STARTUP_RAMP_SEC, counted from the
// first real (non-silent) input rather than from construction, since
// there's normally idle time between pressing Start and speaking. Has no
// effect on an unseeded session, where gain starts at 0dB.
constexpr float STARTUP_RAMP_SEC = 0.3f;

// Peak level (-20dBFS) that counts as real audio for starting the ramp.
// Deliberately well above typical background noise with RNNoise off, so the
// ramp isn't used up on hiss before speech starts.
constexpr double REAL_AUDIO_PEAK_THRESHOLD = 0.1;

constexpr int TEN_MS_DIVIDER = 100;

LevelerStep::LevelerStep(int sampleRate, realtime_fp<float()> const& feedbackLoudnessLufsFn, std::shared_ptr<DiagnosticCsvLogger> diagLogger,
                         float initialGainDb, float initialIntegralErrorDb, float targetLufs,
                         realtime_fp<bool()> const& noiseReductionEnabledFn)
    : sampleRate_(sampleRate)
    , feedbackLoudnessLufsFn_(feedbackLoudnessLufsFn)
    , targetGainDb_(initialGainDb)
    , currentGainDb_(initialGainDb)
    , integralErrorDb_(initialIntegralErrorDb)
    , rampStarted_(false)
    , rampElapsedSec_(0.0f)
    , targetLufs_(targetLufs)
    , noiseReductionEnabledFn_(noiseReductionEnabledFn)
    , diagLogger_(diagLogger)
{
    // Pre-allocate buffers so we don't have to do so during real-time operation.
    outputSamples_ = std::make_unique<short[]>(sampleRate_);
    assert(outputSamples_ != nullptr);
}

LevelerStep::~LevelerStep()
{
    // empty
}

int LevelerStep::getInputSampleRate() const FREEDV_NONBLOCKING
{
    return sampleRate_;
}

int LevelerStep::getOutputSampleRate() const FREEDV_NONBLOCKING
{
    return sampleRate_;
}

std::atomic<float> LevelerStep::liveAppliedGainDb_{0.0f};

float LevelerStep::getLiveAppliedGainDb() FREEDV_NONBLOCKING
{
    return liveAppliedGainDb_.load(std::memory_order_relaxed);
}

short* LevelerStep::execute(short* inputSamples, int numInputSamples, int* numOutputSamples) FREEDV_NONBLOCKING
{
    int tenMsSamples = std::max(1, sampleRate_ / TEN_MS_DIVIDER);

    short* inPtr = inputSamples;
    short* outPtr = outputSamples_.get();
    *numOutputSamples = numInputSamples;

    int remaining = numInputSamples;
    while (remaining > 0)
    {
        int chunkSize = std::min(remaining, tenMsSamples);

        // Step 1: get the limiter's last measured output loudness. At or
        // below the silence threshold means a pause in speech (or no
        // reading yet).
        float feedbackLufs = feedbackLoudnessLufsFn_();
        float silenceThresholdLufs = noiseReductionEnabledFn_() ? SILENCE_THRESHOLD_LUFS_RNNOISE_ON : SILENCE_THRESHOLD_LUFS_RNNOISE_OFF;
        bool feedbackValid = feedbackLufs > silenceThresholdLufs;
        float blockDurationSec = (float)chunkSize / sampleRate_;

        // Input peak for this chunk, needed before gain is applied to
        // decide whether the startup ramp has started.
        double peakAbs = 0.0;
        for (int i = 0; i < chunkSize; i++)
        {
            double absVal = std::abs((double)inPtr[i]) / 32768.0;
            if (absVal > peakAbs) peakAbs = absVal;
        }

        if (!rampStarted_ && peakAbs > REAL_AUDIO_PEAK_THRESHOLD)
        {
            rampStarted_ = true;
        }
        if (rampStarted_)
        {
            rampElapsedSec_ += blockDurationSec;
        }

        if (feedbackValid)
        {
            // Step 2: PI controller target gain (see LEVELER_KP above).
            float instantErrorDb = targetLufs_ - feedbackLufs;

            // Anti-windup: limit the integral term's contribution to the
            // gain range, so a long loud or quiet stretch can't build up
            // an excess that takes a long time to unwind.
            integralErrorDb_ += instantErrorDb * blockDurationSec;
            float integralClampDb = LEVELER_GAIN_LIMIT_DB * LEVELER_INTEGRAL_TIME_CONSTANT_SEC;
            if (integralErrorDb_ > integralClampDb) integralErrorDb_ = integralClampDb;
            if (integralErrorDb_ < -integralClampDb) integralErrorDb_ = -integralClampDb;

            targetGainDb_ = LEVELER_KP * instantErrorDb + integralErrorDb_ / LEVELER_INTEGRAL_TIME_CONSTANT_SEC;
            if (targetGainDb_ > LEVELER_GAIN_LIMIT_DB) targetGainDb_ = LEVELER_GAIN_LIMIT_DB;
            if (targetGainDb_ < -LEVELER_GAIN_LIMIT_DB) targetGainDb_ = -LEVELER_GAIN_LIMIT_DB;
        }

        // Step 3: move current gain a fraction of the way toward target
        // each block (first-order smoothing), rather than a fixed dB/sec
        // step. Held during pauses.
        if (feedbackValid)
        {
            currentGainDb_ += ((targetGainDb_ - currentGainDb_) / LEVELER_TIME_CONSTANT_SEC) * blockDurationSec;
        }

        // Step 4: apply gain, scaled down during the startup ramp-in. Only
        // the applied gain is ramped; the controller state is unaffected.
        // Output is handed to CompressorLimiterStep INPUT_HEADROOM_DB below
        // its true level so that positive gain can't clip in the int16
        // handover; the limiter restores it.
        float rampInFactor = rampStarted_ ? std::min(1.0f, rampElapsedSec_ / STARTUP_RAMP_SEC) : 1.0f;
        float appliedGainDb = currentGainDb_ * rampInFactor;
        float scaleFactor = expf((appliedGainDb - CompressorLimiterStep::INPUT_HEADROOM_DB) / 20.0f * logf(10.0f));
        float temp = 0.0f;
        for (int i = 0; i < chunkSize; i++)
        {
            ConvertSingleSampleToFloatSampleType_<float, short>(&inPtr[i], &temp);
            temp *= scaleFactor;
            ConvertSingleSampleToIntSampleType_<short, float>(&temp, &outPtr[i]);
        }

        // No-op unless built with ENABLE_AUDIO_DIAG_LOGGING.
        double inputDbfs = peakAbs > 0.0 ? 20.0 * std::log10(peakAbs) : -100.0;
        diagLogger_->logLevelerHalf(inputDbfs, feedbackValid ? (double)feedbackLufs : -100.0, targetGainDb_, currentGainDb_, (double)appliedGainDb);

        liveAppliedGainDb_.store(appliedGainDb, std::memory_order_relaxed);

        inPtr += chunkSize;
        outPtr += chunkSize;
        remaining -= chunkSize;
    }

    return outputSamples_.get();
}

void LevelerStep::reset() FREEDV_NONBLOCKING
{
    // Intentionally keeps gain and integral state. reset() is called at
    // the start of every transmission, and resetting would make each one
    // re-climb from 0dB; keeping the integral term consistent with the
    // current gain also avoids a jump in target on the next transmission.
}
