//=========================================================================
// Name:            LevelerStep.cpp
// Purpose:         Describes a loudness leveler step in the audio pipeline.
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

#include "LevelerStep.h"

// Leveler settings. Single symmetric time constant (spec: "same slow up
// and down ramps"), unlike AgcStep's old asymmetric 0.5s/6.0s attack/
// release -- proven in earlier live A/B testing that a proportional/
// time-constant formula (see execute() below) tracks real speech far more
// smoothly under EBU R128's silence-gating than a fixed dB/sec ramp.
constexpr float LEVELER_TARGET_LUFS = -23.0f;
constexpr float LEVELER_GAIN_LIMIT_DB = 12.0f; // symmetric +/-12dB per spec
// 2.0s, per Barry's own prior live-tuning history on the old AgcStep (2026-09-15
// clarification): 0.5s/6.0s asymmetric -> symmetric 3.0s/3.0s (confirmed
// on-air, backend commit 4f59d0b) -> a later refinement down to symmetric
// 2.0s/2.0s, which supersedes the 3.0s value. Not re-derived from scratch for
// this redesign; carried forward as the known-good starting point.
constexpr float LEVELER_TIME_CONSTANT_SEC = 2.0f;
constexpr float SILENCE_THRESHOLD_LUFS = -33.0f;

constexpr int TEN_MS_DIVIDER = 100;

LevelerStep::LevelerStep(int sampleRate, realtime_fp<float()> const& feedbackLoudnessLufsFn, std::shared_ptr<DiagnosticCsvLogger> diagLogger)
    : sampleRate_(sampleRate)
    , feedbackLoudnessLufsFn_(feedbackLoudnessLufsFn)
    , targetGainDb_(0.0f)
    , currentGainDb_(0.0f)
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

        // Step 1: pull the compressor/limiter's last-measured output
        // loudness (feedback loop -- see the plan's Architecture section).
        // A value at/below the silence threshold means either genuine
        // silence or that no valid reading is available yet (e.g. right
        // after construction) -- either way, freeze gain rather than
        // chasing it, same as AgcStep's original silence-gating behavior.
        float feedbackLufs = feedbackLoudnessLufsFn_();
        bool feedbackValid = feedbackLufs > SILENCE_THRESHOLD_LUFS;

        if (feedbackValid)
        {
            // Step 2: calculate target gain.
            //
            // feedbackLufs is measured on the *output* of the compressor/
            // limiter, i.e. after currentGainDb_ has already been applied
            // (the closed feedback loop described in the plan). Computing
            // the target directly from that raw measurement is self-
            // referential: at equilibrium (currentGainDb_ == targetGainDb_
            // == G, and output == input + G), solving
            // G = LEVELER_TARGET_LUFS - (input + G) gives
            // G = (LEVELER_TARGET_LUFS - input) / 2 -- the loop settles at
            // only *half* the actually-needed correction, a permanent
            // steady-state error confirmed both mathematically and against
            // real capture data (2026-09-18: current_gain plateaued flat
            // for 16+ seconds at exactly half the implied input deficit,
            // ruling out a convergence-speed/time-constant explanation).
            //
            // Fix: subtract the already-applied gain back out of the
            // measurement first, recovering an estimate of the pre-gain
            // input loudness. That estimate is independent of G, so the
            // loop converges to the full correction instead of half.
            float estimatedInputLufs = feedbackLufs - currentGainDb_;
            targetGainDb_ = LEVELER_TARGET_LUFS - estimatedInputLufs;
            if (targetGainDb_ > LEVELER_GAIN_LIMIT_DB) targetGainDb_ = LEVELER_GAIN_LIMIT_DB;
            if (targetGainDb_ < -LEVELER_GAIN_LIMIT_DB) targetGainDb_ = -LEVELER_GAIN_LIMIT_DB;

            // Step 3: move current gain a fixed *fraction* of the way toward
            // target each block, rather than a fixed dB/sec step -- this is
            // what makes the formula self-compensate for EBU R128's
            // irregular update opportunities during real speech.
            float blockDurationSec = (float)chunkSize / sampleRate_;
            currentGainDb_ += ((targetGainDb_ - currentGainDb_) / LEVELER_TIME_CONSTANT_SEC) * blockDurationSec;
        }

        // Scale samples based on current gain.
        float scaleFactor = expf(currentGainDb_ / 20.0f * logf(10.0f));
        double peakAbs = 0.0;
        float temp = 0.0f;
        for (int i = 0; i < chunkSize; i++)
        {
            double absVal = std::abs((double)inPtr[i]) / 32768.0;
            if (absVal > peakAbs) peakAbs = absVal;

            ConvertSingleSampleToFloatSampleType_<float, short>(&inPtr[i], &temp);
            temp *= scaleFactor;
            ConvertSingleSampleToIntSampleType_<short, float>(&temp, &outPtr[i]);
        }

        // DIAGNOSTIC ONLY (no-op unless built with ENABLE_AUDIO_DIAG_LOGGING).
        double inputDbfs = peakAbs > 0.0 ? 20.0 * std::log10(peakAbs) : -100.0;
        diagLogger_->logLevelerHalf(inputDbfs, feedbackValid ? (double)feedbackLufs : -100.0, targetGainDb_, currentGainDb_);

        inPtr += chunkSize;
        outPtr += chunkSize;
        remaining -= chunkSize;
    }

    return outputSamples_.get();
}

void LevelerStep::reset() FREEDV_NONBLOCKING
{
    // Reversed 2026-09-18 (Barry): reset() is called on *every* TX entry
    // within a session (TxRxThread.cpp/MinimalTxRxThread.cpp's "just
    // entered TX from RX" path is its only call site in either repo), not
    // just once at Start -- so zeroing gain here meant every single PTT
    // press re-ran the leveler's full climb from 0dB, rather than just
    // once per session. That climb became far more noticeable after the
    // 2026-09-18 feedback-formula fix (see the target-gain comment above):
    // the corrected formula is a genuine integrator of the loudness error
    // with no fixed equilibrium for real (non-constant) speech, so gain
    // takes real, visible time to reach a sensible operating point from a
    // cold 0dB start -- confirmed live via the diagnostic captures.
    // Persisting gain across transmissions (i.e. not touching it here at
    // all) means only the *first* transmission of a session pays that
    // climb; later ones start already near the right operating point.
    // This was Richard's original spec suggestion, explicitly rejected on
    // 2026-09-15 before the above was known -- gain still starts at 0dB
    // per-session via the constructor's own initialization, just no longer
    // re-zeroed on every individual PTT press.
}
