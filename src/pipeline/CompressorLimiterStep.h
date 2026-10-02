//=========================================================================
// Name:            CompressorLimiterStep.h
// Purpose:         Describes a soft-knee compressor/limiter step in the
//                  audio pipeline.
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

#ifndef AUDIO_PIPELINE__COMPRESSOR_LIMITER_STEP_H
#define AUDIO_PIPELINE__COMPRESSOR_LIMITER_STEP_H

#include <atomic>
#include <cstdint>
#include <memory>

#include "IPipelineStep.h"
#include "../util/LoudnessMeter.h"
#include "../util/DiagnosticCsvLogger.h"
#include "../util/realtime_fp.h"

// Soft-knee peak limiter, replacing WebRtcAgc_Process (which, even with
// compressionGaindB=0, applies a fixed internal ~3:1 compression ratio with
// makeup gain rather than acting as a pure limiter).
//
// Look-ahead peak limiter driving a single high-ratio soft knee just below
// full scale, so it only engages on loud excursions near clipping and
// leaves ordinary speech untouched. The audio is delayed by the look-ahead
// window; the gain each sample needs is held at its minimum across the
// window, then smoothed by a moving average of the same length. The gain
// therefore ramps down smoothly ahead of each peak and is fully in place
// when the peak reaches the output, so peaks never overshoot the knee
// curve, however sudden their onset.
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

    // Input level convention: the upstream LevelerStep hands its output
    // over INPUT_HEADROOM_DB below its true level, and this step restores
    // it (in floating point) before limiting. Pipeline steps exchange
    // int16 samples, so without this, peaks the leveler pushes above full
    // scale would be hard-clipped in the handover before the limiter ever
    // saw them. Equal to LevelerStep's maximum gain, so the handover can
    // never clip.
    static constexpr float INPUT_HEADROOM_DB = 12.0f;

private:
    int sampleRate_;
    LoudnessMeter loudnessMeter_;
    std::shared_ptr<DiagnosticCsvLogger> diagLogger_;
    realtime_fp<bool()> noiseReductionEnabledFn_;

    // Returns the minimum of the last windowLength_ values passed in.
    float slidingMinimum_(float value) FREEDV_NONBLOCKING;

    // Look-ahead window length in samples. Must be declared before the
    // buffers below, which are sized from it in the constructor's init list.
    int windowLength_;

    // Audio delay line, windowLength_ - 1 samples.
    std::unique_ptr<float[]> delayBuffer_;
    int delayPos_;

    // Sliding-minimum ring buffer (monotonic queue) of required gains.
    std::unique_ptr<float[]> holdValues_;
    std::unique_ptr<int64_t[]> holdIndices_;
    int holdHead_;
    int holdCount_;
    int64_t sampleIndex_;

    // Held gain after release smoothing (linear, <= 1).
    float releasedGain_;
    float releaseAlpha_;

    // Moving average of releasedGain_ over windowLength_ samples.
    std::unique_ptr<float[]> averageBuffer_;
    double averageSum_;
    int averagePos_;

    float inputHeadroomScale_;

    std::unique_ptr<short[]> outputSamples_;

    static std::atomic<float> lastOutputLoudnessLufs_;
};

#endif // AUDIO_PIPELINE__COMPRESSOR_LIMITER_STEP_H
