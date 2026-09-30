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

#include <atomic>
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
    // noiseReductionEnabledFn (2026-09-21, Barry: "this is the leveller
    // gain freeze during pauses in speech. It could get chattery in high
    // noise environments when rnnoise is off") -- queried each block to
    // pick between two silence-freeze thresholds (see
    // SILENCE_THRESHOLD_LUFS_RNNOISE_ON/OFF in execute()). Defaults to
    // "always on" (the original, unconditional -33 LUFS threshold) so
    // existing callers (tests, freedv-backend's own MinimalTxRxThread.cpp)
    // are unaffected.
    // targetLufs (2026-09-24, Barry: real QSO data showed the leveler
    // riding within ~1dB of its +12dB ceiling for an entire session when
    // input was gain-staged conservatively for headroom, because -23 LUFS
    // sits ~7-11dB above where careful gain-staging naturally lands --
    // "Can we add 'targetLUFS' in the settings to speed testing without
    // rebuilds?") -- replaces the old fixed LEVELER_TARGET_LUFS constant
    // as this step's actual target, so a caller (the GUI's config file) can
    // test different values without a rebuild. Defaults to -23.0f, the
    // original constant's value, so existing callers are unaffected.
    // pauseGracePeriodSec (2026-09-30, Barry: "Is the grace period
    // accessible to change?", after asking to empirically test whether
    // 300ms is actually enough for a large, still-converging correction
    // to keep progressing through natural inter-word pauses -- see the
    // pause grace period's own comment in execute()) -- same rebuild-free
    // testing pattern as targetLufs above. Defaults to 0.3f, the original
    // constant's value, so existing callers are unaffected.
    LevelerStep(int sampleRate, realtime_fp<float()> const& feedbackLoudnessLufsFn, std::shared_ptr<DiagnosticCsvLogger> diagLogger,
                float initialGainDb = 0.0f, float initialIntegralErrorDb = 0.0f, float targetLufs = -23.0f,
                realtime_fp<bool()> const& noiseReductionEnabledFn = +[]() FREEDV_NONBLOCKING { return true; },
                float pauseGracePeriodSec = 0.3f);
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

    // Live gain, for a GUI display polling this *while* execute() is
    // running concurrently on the pipeline thread (2026-09-28, Barry:
    // "I would like to add a plot of AGCdB similar to the SNR plot...
    // something I want to see while transmitting") -- unlike
    // getCurrentGainDb() above, this is safe to call from another thread
    // at any time. A static atomic mirror of each block's actually-
    // applied gain (post startup-ramp, matching the diagnostic CSV's own
    // "applied_gain_db" column -- what the encoder is really getting,
    // not the PI controller's pre-ramp internal state), same "exactly
    // one instance active in the real TX pipeline at a time" assumption
    // already made by CompressorLimiterStep::getLastOutputLoudnessLufs().
    static float getLiveAppliedGainDb() FREEDV_NONBLOCKING;

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
    // Pause grace period (2026-09-30) -- see execute()'s own comment.
    // lastValidFeedbackLufs_ starts at NO_VALID_FEEDBACK_YET_SENTINEL_LUFS
    // (LevelerStep.cpp) so the grace period can never substitute an unset
    // default in for a real reading before any valid feedback has arrived.
    float lastValidFeedbackLufs_;
    float invalidFeedbackElapsedSec_;
    float pauseGracePeriodSec_;
    float targetLufs_;
    realtime_fp<bool()> noiseReductionEnabledFn_;
    std::unique_ptr<short[]> outputSamples_;
    std::shared_ptr<DiagnosticCsvLogger> diagLogger_;

    static std::atomic<float> liveAppliedGainDb_;
};

#endif // AUDIO_PIPELINE__LEVELER_STEP_H
