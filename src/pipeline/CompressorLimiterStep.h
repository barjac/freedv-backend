//=========================================================================
// Name:            CompressorLimiterStep.h
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

#ifndef AUDIO_PIPELINE__COMPRESSOR_LIMITER_STEP_H
#define AUDIO_PIPELINE__COMPRESSOR_LIMITER_STEP_H

#include <atomic>
#include <memory>

#include "IPipelineStep.h"
#include "../util/LoudnessMeter.h"
#include "../util/DiagnosticCsvLogger.h"
#include "../util/realtime_fp.h"

// Soft-knee limiter, replacing WebRtcAgc_Process (which turned out to be a
// hardcoded ~3:1 compressor with hidden makeup gain, not a limiter -- see
// the "Replace AgcStep with a Leveler + Compressor/Limiter pair" plan).
// Runs a genuine per-sample envelope follower with a short (~3-5ms)
// look-ahead delay line, applying a single near-infinite-ratio soft-knee
// gain stage right at the ceiling, so it only ever engages on genuinely
// loud excursions near clipping, never on ordinary speech -- important
// because RADE's neural encoder was almost certainly trained on
// uncompressed speech (Barry, 2026-09-15).
//
// Originally a *two*-knee design (a gentler "compressor" knee pushed close
// to the ceiling, feeding this same near-clip "limiter" knee). The
// compressor knee was removed 2026-09-18 -- see CompressorLimiterStep.cpp's
// comment at its old threshold constant -- once live testing showed its
// one-directional gain reduction was feeding LevelerStep's closed feedback
// loop a systematic upward bias on loud input (the leveler has no way to
// distinguish "the compressor just reduced this" from "the input actually
// got quieter"), and it was engaging often enough (~40% of rows on loud
// speech) for that bias to be significant. A single rare, near-clip-only
// limiter stage keeps the feedback loop meaningful (per Richard's spec --
// the leveler should react to whatever this step actually does) while
// keeping that bias negligible in practice.
//
// Owns a LoudnessMeter on its own *output*, feeding LevelerStep's feedback
// loop via getLastOutputLoudnessLufs() (a static accessor, not an instance
// method -- realtime_fp<float()> can only hold a plain captureless function
// pointer, not one capturing a specific instance; see LevelAdjustStep's own
// usage precedent in TxRxThread.cpp). Safe as a singleton-style value since
// exactly one CompressorLimiterStep is ever active in the real TX pipeline
// at a time, same assumption already made by g_agcEnabled and friends.
class CompressorLimiterStep : public IPipelineStep
{
public:
    // noiseReductionEnabledFn (2026-09-24, Barry -- found via a real
    // capture plus an on/off/on/off live test confirming it always
    // recovers, ruling out a stuck/corrupted state): picks between two
    // silence floors for the internal LoudnessMeter's momentary reading
    // (see LoudnessMeter.h's own comment and SILENCE_FLOOR_LUFS_RNNOISE_ON/
    // OFF in the .cpp) -- with RNNoise off, genuine gaps between words can
    // read quieter than RNNoise's own small residual noise floor during
    // the same gaps, invalidating far more real (if quiet) speech than
    // intended. Defaults to "always on" (the original, unconditional
    // -70.0f floor) so existing callers (tests, freedv-backend's own
    // MinimalTxRxThread.cpp) are unaffected.
    CompressorLimiterStep(int sampleRate, std::shared_ptr<DiagnosticCsvLogger> diagLogger,
                           realtime_fp<bool()> const& noiseReductionEnabledFn = +[]() FREEDV_NONBLOCKING { return true; });
    virtual ~CompressorLimiterStep();

    virtual int getInputSampleRate() const FREEDV_NONBLOCKING override;
    virtual int getOutputSampleRate() const FREEDV_NONBLOCKING override;
    virtual short* execute(short* inputSamples, int numInputSamples, int* numOutputSamples) FREEDV_NONBLOCKING override;
    virtual void reset() FREEDV_NONBLOCKING override;

    static float getLastOutputLoudnessLufs() FREEDV_NONBLOCKING;

private:
    int sampleRate_;
    LoudnessMeter loudnessMeter_;
    std::shared_ptr<DiagnosticCsvLogger> diagLogger_;
    realtime_fp<bool()> noiseReductionEnabledFn_;

    // Look-ahead delay line: buf_[pos_] always holds the oldest (soon to be
    // overwritten) sample, i.e. the one lookAheadLength_ samples in the
    // past relative to the sample about to be written -- see execute()'s
    // "read-then-write same slot" ring buffer pattern.
    // (lookAheadLength_ declared before lookAheadBuffer_ deliberately --
    // member init order follows declaration order, and the buffer's size
    // depends on the length computed in the constructor init list.)
    int lookAheadLength_;
    std::unique_ptr<float[]> lookAheadBuffer_;
    int lookAheadPos_;

    // Per-sample envelope-follower state: smoothed *gain-reduction*
    // command (dB, <= 0), not the level itself.
    float smoothedGainReductionDb_;
    float attackAlpha_;
    float releaseAlpha_;

    std::unique_ptr<short[]> outputSamples_;

    static std::atomic<float> lastOutputLoudnessLufs_;
};

#endif // AUDIO_PIPELINE__COMPRESSOR_LIMITER_STEP_H
