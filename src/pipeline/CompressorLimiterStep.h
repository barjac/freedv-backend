//=========================================================================
// Name:            CompressorLimiterStep.h
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

#ifndef AUDIO_PIPELINE__COMPRESSOR_LIMITER_STEP_H
#define AUDIO_PIPELINE__COMPRESSOR_LIMITER_STEP_H

#include <atomic>
#include <memory>

#include "IPipelineStep.h"
#include "../util/LoudnessMeter.h"
#include "../util/DiagnosticCsvLogger.h"
#include "../util/realtime_fp.h"

// Soft-knee peak limiter, replacing WebRtcAgc_Process (which, even with
// compressionGaindB=0, applies a fixed internal ~3:1 compression ratio with
// makeup gain rather than acting as a pure limiter).
//
// A per-sample envelope follower with a short look-ahead delay line drives
// a single high-ratio soft knee just below full scale, so it only engages
// on loud excursions near clipping and leaves ordinary speech untouched.
// Leaving speech dynamics alone matters because the RADE encoder is
// expected to have been trained on uncompressed speech. A lower
// "compressor" knee was tried and removed: its frequent gain reduction fed
// LevelerStep's feedback loop an upward bias on loud input.
//
// Also measures the loudness of its own output and publishes it via
// getLastOutputLoudnessLufs() as LevelerStep's feedback signal. This is
// static rather than per-instance because realtime_fp can only hold a
// captureless function pointer (see LevelAdjustStep's usage); only one
// CompressorLimiterStep is active in the TX pipeline at a time.
class CompressorLimiterStep : public IPipelineStep
{
public:
    // noiseReductionEnabledFn is polled each block to choose the silence
    // floor for the output loudness measurement. With RNNoise off, gaps
    // between words can measure quieter than RNNoise's own residual noise
    // floor, so a lower floor is used to avoid rejecting quiet speech.
    CompressorLimiterStep(int sampleRate, std::shared_ptr<DiagnosticCsvLogger> diagLogger,
                           realtime_fp<bool()> const& noiseReductionEnabledFn = +[]() FREEDV_NONBLOCKING { return true; });
    virtual ~CompressorLimiterStep();

    virtual int getInputSampleRate() const FREEDV_NONBLOCKING override;
    virtual int getOutputSampleRate() const FREEDV_NONBLOCKING override;
    virtual short* execute(short* inputSamples, int numInputSamples, int* numOutputSamples) FREEDV_NONBLOCKING override;
    virtual void reset() FREEDV_NONBLOCKING override;

    // Most recent momentary loudness of this step's output, or -100.0 if
    // the last reading was silent/invalid.
    static float getLastOutputLoudnessLufs() FREEDV_NONBLOCKING;

private:
    int sampleRate_;
    LoudnessMeter loudnessMeter_;
    std::shared_ptr<DiagnosticCsvLogger> diagLogger_;
    realtime_fp<bool()> noiseReductionEnabledFn_;

    // Look-ahead ring buffer: lookAheadBuffer_[lookAheadPos_] holds the
    // sample from lookAheadLength_ samples ago. lookAheadLength_ must be
    // declared before lookAheadBuffer_ since the buffer is sized from it in
    // the constructor's init list.
    int lookAheadLength_;
    std::unique_ptr<float[]> lookAheadBuffer_;
    int lookAheadPos_;

    // Smoothed gain-reduction command in dB (always <= 0).
    float smoothedGainReductionDb_;
    float attackAlpha_;
    float releaseAlpha_;

    std::unique_ptr<short[]> outputSamples_;

    static std::atomic<float> lastOutputLoudnessLufs_;
};

#endif // AUDIO_PIPELINE__COMPRESSOR_LIMITER_STEP_H
