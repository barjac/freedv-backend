//=========================================================================
// Name:            PostLoopCompressorStep.h
// Purpose:         Describes an optional, independent two-knee soft-knee
//                  compressor step, positioned after (outside) the
//                  LevelerStep/CompressorLimiterStep feedback loop.
//
// Authors:         Claude Code (for Barry Jones, G4MKT)
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

#ifndef AUDIO_PIPELINE__POST_LOOP_COMPRESSOR_STEP_H
#define AUDIO_PIPELINE__POST_LOOP_COMPRESSOR_STEP_H

#include <chrono>
#include <cstdio>
#include <memory>

#include "IPipelineStep.h"

// Optional two-knee soft-knee compressor, 2026-09-21 (Barry): "I wonder if
// there really is any advantage in wrapping the fast clipper inside the PI
// feedback loop. If it was outside we could use the 2 knee soft
// compression, which would curb some of the high peaks."
//
// CompressorLimiterStep.h's own history explains why its original
// "compressor" knee (a gentler ~-6dBFS/2:1 stage) was removed: its gain
// reduction fed LevelerStep's closed feedback loop a one-directional
// upward bias (the leveler has no way to distinguish "this stage just
// reduced the level" from "the input actually got quieter"). That problem
// is specific to being *inside* the loop -- a second compression stage
// positioned entirely outside/after it (see TxRxThread.cpp's wiring) can
// never feed that bias back, because nothing downstream of the loop is
// ever measured by it. This class is deliberately standalone (no shared
// state with CompressorLimiterStep at all, not even the same static
// gain-curve helper function -- duplicated rather than shared, so there
// is no code path by which this stage could ever affect the existing,
// already-tested PI loop) so enabling/disabling/tuning it can't touch
// LevelerStep's behaviour under any circumstance.
//
// Same two thresholds as the original (pre-2026-09-18) two-knee design --
// knee A ("compressor," ~-6dBFS/2:1) feeding knee B ("limiter,"
// ~-1.5dBFS/20:1) -- as a starting point for live A/B tuning via its own
// independent toggle, not re-derived from scratch. Like
// CompressorLimiterStep, this only ever *reduces* gain (soft-knee curve
// applied directly to level, no separate makeup-gain term anywhere in the
// signal path) -- maximum gain is unity, per Richard's original spec
// constraint and Barry's explicit requirement here.
//
// DIAGNOSTIC ONLY (2026-09-21): writes its own small CSV
// (~/postloop_compressor_diag.csv, only when built with
// -DENABLE_AUDIO_DIAG_LOGGING=ON, otherwise a no-op) -- deliberately a
// separate file via its own private fopen/fprintf, NOT routed through the
// shared DiagnosticCsvLogger used by LevelerStep/CompressorLimiterStep.
// Found the need for this the hard way: a first A/B test comparing two
// captures with this stage toggled on vs. off showed no difference at all
// in ~/agc_diag.csv, because that file's own output_dbfs/gain-reduction
// columns are written by CompressorLimiterStep *before* this stage ever
// runs -- this stage's own action was completely invisible regardless of
// its setting. Keeping this as a genuinely separate file (rather than
// extending the shared logger's row schema again) avoids any change to
// CompressorLimiterStep.cpp's own already-tested logging call, for the
// same "can't touch the existing loop" reason the rest of this class is
// standalone.
class PostLoopCompressorStep : public IPipelineStep
{
public:
    PostLoopCompressorStep(int sampleRate);
    virtual ~PostLoopCompressorStep();

    virtual int getInputSampleRate() const FREEDV_NONBLOCKING override;
    virtual int getOutputSampleRate() const FREEDV_NONBLOCKING override;
    virtual short* execute(short* inputSamples, int numInputSamples, int* numOutputSamples) FREEDV_NONBLOCKING override;
    virtual void reset() FREEDV_NONBLOCKING override;

private:
    int sampleRate_;

    // Look-ahead delay line -- same ring-buffer pattern as
    // CompressorLimiterStep (lookAheadLength_ declared before
    // lookAheadBuffer_ deliberately -- member init order follows
    // declaration order, and the buffer's size depends on it).
    int lookAheadLength_;
    std::unique_ptr<float[]> lookAheadBuffer_;
    int lookAheadPos_;

    // Per-sample envelope-follower state: smoothed *gain-reduction*
    // command (dB, <= 0), not the level itself.
    float smoothedGainReductionDb_;
    float attackAlpha_;
    float releaseAlpha_;

    std::unique_ptr<short[]> outputSamples_;

    // DIAGNOSTIC ONLY -- see class comment above.
    FILE* diagFile_;
    std::chrono::steady_clock::time_point diagStartTime_;
};

#endif // AUDIO_PIPELINE__POST_LOOP_COMPRESSOR_STEP_H
