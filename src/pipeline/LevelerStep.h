//=========================================================================
// Name:            LevelerStep.h
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

#ifndef AUDIO_PIPELINE__LEVELER_STEP_H
#define AUDIO_PIPELINE__LEVELER_STEP_H

#include <memory>

#include "IPipelineStep.h"
#include "../util/realtime_fp.h"
#include "../util/DiagnosticCsvLogger.h"

// Loudness leveler: slow (multi-second, symmetric) gain toward a target
// LUFS, driven by feedback of the *downstream* CompressorLimiterStep's
// measured output loudness (not this step's own input) -- see the
// "Replace AgcStep with a Leveler + Compressor/Limiter pair" plan's
// Architecture section for why. Must be constructed and wired downstream
// of a CompressorLimiterStep in the same pipeline.
class LevelerStep : public IPipelineStep
{
public:
    // initialGainDb/initialIntegralErrorDb (2026-09-20): seed the same
    // persisted state reset() already preserves across transmissions
    // within a session (see reset()'s own comment) -- lets a caller resume
    // from a value saved at the end of a *previous* session (e.g. in the
    // GUI's config file) instead of always starting cold at 0dB. Callers
    // that don't care (e.g. existing tests) get the original 0dB/0dB
    // cold-start behavior via the defaults.
    LevelerStep(int sampleRate, realtime_fp<float()> const& feedbackLoudnessLufsFn, std::shared_ptr<DiagnosticCsvLogger> diagLogger,
                float initialGainDb = 0.0f, float initialIntegralErrorDb = 0.0f);
    virtual ~LevelerStep();

    virtual int getInputSampleRate() const FREEDV_NONBLOCKING override;
    virtual int getOutputSampleRate() const FREEDV_NONBLOCKING override;
    virtual short* execute(short* inputSamples, int numInputSamples, int* numOutputSamples) FREEDV_NONBLOCKING override;
    virtual void reset() FREEDV_NONBLOCKING override;

    // Only safe to call once the pipeline thread that calls execute() has
    // actually stopped calling it (e.g. after the owning TxRxThread has
    // been joined) -- these read currentGainDb_/integralErrorDb_ with no
    // synchronization of their own, matching how execute() itself is only
    // ever called from that one thread. Intended for a caller to persist
    // this state (e.g. to a config file) once a session ends.
    float getCurrentGainDb() const FREEDV_NONBLOCKING { return currentGainDb_; }
    float getIntegralErrorDb() const FREEDV_NONBLOCKING { return integralErrorDb_; }

private:
    int sampleRate_;
    realtime_fp<float()> feedbackLoudnessLufsFn_;
    float targetGainDb_;
    float currentGainDb_;
    // PI controller integral term -- see the "PI controller" comment in
    // execute() for why this exists (2026-09-18). Accumulates the raw
    // loudness error over time (dB*sec); divided by
    // LEVELER_INTEGRAL_TIME_CONSTANT_SEC to get its dB contribution to
    // targetGainDb_. Persists across transmissions, same as
    // currentGainDb_/targetGainDb_ (see reset()) -- keeping all three
    // consistent avoids a discontinuous jump in target at the start of a
    // new transmission.
    float integralErrorDb_;
    // Startup ramp-in (2026-09-20) -- see execute()'s own comment on why
    // this exists. rampStarted_ latches true the first time real
    // (non-silent) audio is seen and never resets (including across
    // reset(), same persistence philosophy as currentGainDb_ etc. above)
    // -- only the *first* real audio of a whole session needs this
    // protection. rampElapsedSec_ only advances once rampStarted_ is true.
    bool rampStarted_;
    float rampElapsedSec_;
    std::unique_ptr<short[]> outputSamples_;
    std::shared_ptr<DiagnosticCsvLogger> diagLogger_;
};

#endif // AUDIO_PIPELINE__LEVELER_STEP_H
