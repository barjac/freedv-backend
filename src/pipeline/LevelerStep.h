//=========================================================================
// Name:            LevelerStep.h
// Purpose:         Describes a loudness leveler step in the audio pipeline.
//
// Authors:         Claude Code (for Barry Jackson, G4MKT), based on design
//                  suggestions from g4dya (Richard) on PR #1472
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

#ifndef AUDIO_PIPELINE__LEVELER_STEP_H
#define AUDIO_PIPELINE__LEVELER_STEP_H

#include <atomic>
#include <memory>

#include "IPipelineStep.h"
#include "../util/realtime_fp.h"
#include "../util/DiagnosticCsvLogger.h"

// Loudness leveler: slowly adjusts gain (within +/-12dB) so that the
// momentary loudness measured at the *output* of the downstream
// CompressorLimiterStep converges on a target LUFS. Measuring after the
// limiter closes the loop, so the leveler responds to what is actually
// sent to the encoder, and brief peaks caught by the limiter don't pull
// the leveler's gain down.
//
// Gain is driven by a PI controller and smoothed, so the correction scales
// with the size of the error and settles on the target rather than moving
// at a fixed dB/sec rate. Gain holds during pauses in speech.
//
// Must be wired directly upstream of a CompressorLimiterStep in the same
// pipeline, with feedbackLoudnessLufsFn returning
// CompressorLimiterStep::getLastOutputLoudnessLufs(). Output is
// CompressorLimiterStep::INPUT_HEADROOM_DB below its true level, which
// the limiter restores. To switch levelling off, use enabledFn rather than
// bypassing this step, so the limiter stays in circuit with a consistent
// input level.
class LevelerStep : public IPipelineStep
{
public:
    // initialGainDb/initialIntegralErrorDb: resume controller state saved
    // from a previous session (e.g. in the application's config file)
    // instead of starting at 0dB.
    // targetLufs: loudness target for the output of the limiter.
    // noiseReductionEnabledFn: polled each block to choose the threshold
    //   below which feedback is treated as a pause in speech.
    // enabledFn: polled each block. While false, gain is 0dB (audio still
    //   passes through, with the handover headroom, so the downstream
    //   limiter stays in circuit) and the controller state is frozen, so
    //   re-enabling resumes from where it was.
    LevelerStep(int sampleRate, realtime_fp<float()> const& feedbackLoudnessLufsFn, std::shared_ptr<DiagnosticCsvLogger> diagLogger,
                float initialGainDb = 0.0f, float initialIntegralErrorDb = 0.0f, float targetLufs = -23.0f,
                realtime_fp<bool()> const& noiseReductionEnabledFn = +[]() FREEDV_NONBLOCKING { return true; },
                realtime_fp<bool()> const& enabledFn = +[]() FREEDV_NONBLOCKING { return true; });
    virtual ~LevelerStep();

    virtual int getInputSampleRate() const FREEDV_NONBLOCKING override;
    virtual int getOutputSampleRate() const FREEDV_NONBLOCKING override;
    virtual short* execute(short* inputSamples, int numInputSamples, int* numOutputSamples) FREEDV_NONBLOCKING override;
    virtual void reset() FREEDV_NONBLOCKING override;

    // Controller state, for persisting across sessions. Not synchronized:
    // only call once the pipeline thread has stopped calling execute().
    float getCurrentGainDb() const FREEDV_NONBLOCKING { return currentGainDb_; }
    float getIntegralErrorDb() const FREEDV_NONBLOCKING { return integralErrorDb_; }

    // Gain actually applied to the most recent block (including the
    // startup ramp-in). Safe to poll from another thread while the
    // pipeline is running, e.g. for a GUI display. Static for the same
    // reason as CompressorLimiterStep::getLastOutputLoudnessLufs().
    static float getLiveAppliedGainDb() FREEDV_NONBLOCKING;

private:
    int sampleRate_;
    realtime_fp<float()> feedbackLoudnessLufsFn_;
    float targetGainDb_;
    float currentGainDb_;
    // PI controller integral term (accumulated loudness error, dB*sec).
    float integralErrorDb_;
    // Startup ramp-in: rampStarted_ latches the first time real (non-silent)
    // input is seen; rampElapsedSec_ only advances after that.
    bool rampStarted_;
    float rampElapsedSec_;
    float targetLufs_;
    realtime_fp<bool()> noiseReductionEnabledFn_;
    realtime_fp<bool()> enabledFn_;
    std::unique_ptr<short[]> outputSamples_;
    std::shared_ptr<DiagnosticCsvLogger> diagLogger_;

    static std::atomic<float> liveAppliedGainDb_;
};

#endif // AUDIO_PIPELINE__LEVELER_STEP_H
