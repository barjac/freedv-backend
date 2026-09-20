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

// PI controller integral time constant (2026-09-18) -- see execute()'s
// "PI controller" comment for the full derivation. Deliberately longer
// than LEVELER_TIME_CONSTANT_SEC: the integral term's only job is slowly
// eliminating any *persistent* bias the proportional term's own self-
// reference leaves behind, not reacting quickly (that's the proportional
// term's job, via the existing current-gain smoothing below).
// 2026-09-19: lowered from an initial 15.0f (which live-tested fine for
// "normal"/"high" input -- converged and settled correctly within a
// ~20-30s test -- but a "low" test needing ~7-9dB of correction was still
// visibly climbing, not yet converged, by ~22 seconds) to 6.0f, matching
// Richard (G4DYA)'s original spec suggestion for the old AgcStep's release
// time -- confirmed a clear live improvement (low converged noticeably
// faster/closer, high stayed rock-steady at target throughout).
//
// Further lowered same day to 4.0f, per Barry: the old AgcStep tolerated
// symmetric attack/release as fast as 2-3s without excessive wander (see
// LEVELER_TIME_CONSTANT_SEC's own history comment below). Worth treating
// that precedent cautiously rather than assuming it transfers directly,
// though -- unlike a simple lag (which is inherently self-limiting/
// restoring at any speed), this integral term has *no* restoring force of
// its own (that's the proportional term's and the 2.0s current-gain
// smoothing's job); making it faster mainly risks overshoot past the
// target rather than the kind of wander a simple lag would show. Anti-
// windup clamping is already in place either way. Re-tune further if this
// over/undershoots in practice.
constexpr float LEVELER_INTEGRAL_TIME_CONSTANT_SEC = 4.0f;

constexpr int TEN_MS_DIVIDER = 100;

// Startup ramp-in (2026-09-20, Barry -- found via a real capture): applying
// a seeded, persisted currentGainDb_ (see the constructor's own comment)
// in full from sample one stacks with CompressorLimiterStep's own
// gain-reduction envelope *also* starting cold (0dB reduction) at the
// exact same moment -- a genuinely loud first syllable gets the full
// persisted boost before the limiter's fast (but not instant) attack has
// had any real audio to react to. Confirmed on a real capture: a first
// syllable that peaked around -5dBFS raw (safely below clipping on its
// own) touched true 0dBFS after a persisted +5.25dB was applied
// instantly, while every later syllable at similar raw levels stayed
// comfortably clear once the limiter had "warmed up". A fresh (non-seeded)
// session doesn't need this: currentGainDb_ starts at 0dB and can't move
// far in under a second thanks to LEVELER_TIME_CONSTANT_SEC, so this ramp
// is a no-op there. 300ms is fast enough to be inaudible against any real
// speech onset, but gives the limiter's per-sample envelope (2-5ms attack)
// dozens of reaction cycles on real, gradually-increasing gain before the
// full persisted value ever lands.
//
// Correction, same day (Barry, watching the newly-added applied-gain
// trace): the ramp must NOT be keyed to wall-clock time since
// construction -- a first version did exactly that, and Barry caught
// (from the graph) that it was completing "immediately after TX starts,
// long before the first syllable". In real use there's always some delay
// between pressing Start (LevelerStep's construction) and actually
// keying PTT and speaking, so a construction-time-based ramp had always
// already finished by the time real audio arrived -- protecting nothing.
// Fixed by keying the ramp to elapsed time since the first genuinely
// non-silent input is actually seen (REAL_AUDIO_PEAK_THRESHOLD below),
// which is the moment protection is actually needed.
constexpr float STARTUP_RAMP_SEC = 0.3f;

// -50dBFS -- quiet room tone/mic self-noise should stay below this, while
// even a soft spoken word should exceed it. Only used to decide when the
// startup ramp-in above should start counting; unrelated to
// SILENCE_THRESHOLD_LUFS (a *measured loudness* gate on the leveler's
// feedback, not a raw-peak gate on its input).
constexpr double REAL_AUDIO_PEAK_THRESHOLD = 0.00316;

LevelerStep::LevelerStep(int sampleRate, realtime_fp<float()> const& feedbackLoudnessLufsFn, std::shared_ptr<DiagnosticCsvLogger> diagLogger,
                         float initialGainDb, float initialIntegralErrorDb)
    : sampleRate_(sampleRate)
    , feedbackLoudnessLufsFn_(feedbackLoudnessLufsFn)
    , targetGainDb_(initialGainDb)
    , currentGainDb_(initialGainDb)
    , integralErrorDb_(initialIntegralErrorDb)
    , rampStarted_(false)
    , rampElapsedSec_(0.0f)
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
        float blockDurationSec = (float)chunkSize / sampleRate_;

        // Raw input peak for this chunk -- needed both for the startup
        // ramp-in's "has real audio actually started" check below and (as
        // always) for the diagnostic input_dbfs value further down.
        // Deliberately a separate pass over inPtr before any gain is
        // applied, rather than folded into the sample-scaling loop below
        // as it used to be -- this chunk's ramp state must be decided
        // *before* computing this same chunk's applied gain.
        double peakAbs = 0.0;
        for (int i = 0; i < chunkSize; i++)
        {
            double absVal = std::abs((double)inPtr[i]) / 32768.0;
            if (absVal > peakAbs) peakAbs = absVal;
        }

        // Startup ramp-in bookkeeping -- see STARTUP_RAMP_SEC's own
        // comment above for why this exists and why it's keyed to real
        // audio arriving, not wall-clock time since construction.
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
            // Step 2: calculate target gain -- PI controller (2026-09-18).
            //
            // feedbackLufs is measured on the *output* of the compressor/
            // limiter, i.e. after currentGainDb_ has already been applied
            // (the closed feedback loop described in the plan), so the raw
            // instantaneous error below is self-referential in exactly the
            // way the original (single-term, proportional-only) formula
            // was: at equilibrium (current==target==G, output==input+G),
            // G = LEVELER_TARGET_LUFS - (input + G) only has a solution at
            // G = (LEVELER_TARGET_LUFS - input) / 2 -- half the needed
            // correction, a permanent steady-state error (confirmed
            // 2026-09-18: current_gain plateaued flat for 16+ seconds at
            // exactly half the implied input deficit).
            //
            // An earlier fix (2026-09-18, same day) tried subtracting
            // currentGainDb_ back out of the estimate to cancel that self-
            // reference algebraically. It worked for a genuinely constant
            // signal (LevelerStepTest's synthetic sine wave), but
            // substituting it into the smoothing update below shows the
            // currentGainDb_ terms cancel *completely*, turning the whole
            // formula into a pure integrator of the loudness error with no
            // restoring force at all. For real, time-varying speech that
            // has no fixed equilibrium -- confirmed live (2026-09-18):
            // current_gain climbing steadily through an entire transmission
            // despite steady -23 LUFS input (not converging, just slowly
            // drifting), and later, gain persisting near 0dB through a
            // sustained loud passage despite target repeatedly diving to
            // -6..-8dB (the net average of a real recording's momentary
            // loudness swings doesn't have to average to zero just because
            // the recording is "loud overall").
            //
            // Fix: a genuine PI controller. The proportional term below is
            // the same self-referential raw error the original formula
            // used (still only "correct" to within the same 50% bias in
            // isolation) -- but paired with a separate, slowly-accumulating
            // integral term that has no such bias and dominates at true
            // equilibrium. Substituting into the smoothing update: at
            // equilibrium (current==target==G, *and* the integral term has
            // stopped changing, which only happens once the instantaneous
            // error is itself zero), solving requires feedbackLufs to reach
            // LEVELER_TARGET_LUFS exactly -- independent of the
            // proportional term's own gain (Kp) or the integral time
            // constant (Ki), which only affect *how fast* it gets there,
            // not the final value. The proportional term still supplies a
            // genuine restoring force for real, varying speech (reacting
            // to each block's error directly, smoothed by the existing
            // current-gain lag below) that the pure-integrator attempt
            // above lacked entirely.
            constexpr float KP = 1.0f;
            float instantErrorDb = LEVELER_TARGET_LUFS - feedbackLufs;

            integralErrorDb_ += instantErrorDb * blockDurationSec;
            // Anti-windup: without this, a long loud or quiet stretch that
            // saturates targetGainDb_'s clamp below could keep accumulating
            // integralErrorDb_ far beyond what's ever usable, so once real
            // conditions reverse, gain would take a long time to "unwind"
            // that excess before it starts responding correctly again --
            // the classic PI integrator-windup problem. Clamping
            // integralErrorDb_ itself to the range that keeps its own
            // contribution within +/-LEVELER_GAIN_LIMIT_DB avoids that.
            float integralClampDb = LEVELER_GAIN_LIMIT_DB * LEVELER_INTEGRAL_TIME_CONSTANT_SEC;
            if (integralErrorDb_ > integralClampDb) integralErrorDb_ = integralClampDb;
            if (integralErrorDb_ < -integralClampDb) integralErrorDb_ = -integralClampDb;

            targetGainDb_ = KP * instantErrorDb + integralErrorDb_ / LEVELER_INTEGRAL_TIME_CONSTANT_SEC;
            if (targetGainDb_ > LEVELER_GAIN_LIMIT_DB) targetGainDb_ = LEVELER_GAIN_LIMIT_DB;
            if (targetGainDb_ < -LEVELER_GAIN_LIMIT_DB) targetGainDb_ = -LEVELER_GAIN_LIMIT_DB;

            // Step 3: move current gain a fixed *fraction* of the way toward
            // target each block, rather than a fixed dB/sec step -- this is
            // what makes the formula self-compensate for EBU R128's
            // irregular update opportunities during real speech.
            currentGainDb_ += ((targetGainDb_ - currentGainDb_) / LEVELER_TIME_CONSTANT_SEC) * blockDurationSec;
        }

        // Scale samples based on current gain -- ramped in over
        // STARTUP_RAMP_SEC of real elapsed time since real audio was first
        // seen (a no-op once past it, and effectively a no-op for a fresh,
        // non-seeded session too, since currentGainDb_ can't move far from
        // 0 in under a second anyway). Deliberately doesn't touch
        // currentGainDb_ itself -- the PI controller's own state keeps
        // evolving normally underneath; only the actually-*applied* gain
        // is held back at the very start. Before real audio has ever been
        // seen (rampStarted_ still false), there's nothing to protect
        // against yet, so apply gain in full -- harmless on silence.
        float rampInFactor = rampStarted_ ? std::min(1.0f, rampElapsedSec_ / STARTUP_RAMP_SEC) : 1.0f;
        float appliedGainDb = currentGainDb_ * rampInFactor;
        float scaleFactor = expf(appliedGainDb / 20.0f * logf(10.0f));
        float temp = 0.0f;
        for (int i = 0; i < chunkSize; i++)
        {
            ConvertSingleSampleToFloatSampleType_<float, short>(&inPtr[i], &temp);
            temp *= scaleFactor;
            ConvertSingleSampleToIntSampleType_<short, float>(&temp, &outPtr[i]);
        }

        // DIAGNOSTIC ONLY (no-op unless built with ENABLE_AUDIO_DIAG_LOGGING).
        double inputDbfs = peakAbs > 0.0 ? 20.0 * std::log10(peakAbs) : -100.0;
        diagLogger_->logLevelerHalf(inputDbfs, feedbackValid ? (double)feedbackLufs : -100.0, targetGainDb_, currentGainDb_, (double)appliedGainDb);

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
    //
    // integralErrorDb_ (added same day, PI controller redesign) is left
    // untouched here for the same reason -- resetting it while
    // currentGainDb_ persists would make targetGainDb_ jump discontinuously
    // at the start of the next transmission (losing the integral
    // contribution that was supporting wherever currentGainDb_ currently
    // sits), the opposite of the smooth persistence intended above.
}
