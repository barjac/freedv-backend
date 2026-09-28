//=========================================================================
// Name:            AgcStep.h
// Purpose:         Describes an AGC step in the audio pipeline.
//
// Authors:         Mooneer Salem
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

#ifndef AUDIO_PIPELINE__AGC_STEP_H
#define AUDIO_PIPELINE__AGC_STEP_H

#include "IPipelineStep.h"
#include "../util/GenericFIFO.h"
#include "../3rdparty/WebRTC_AGC/agc.h"

#include <chrono>
#include <cstdio>
#include <memory>

class AgcStep : public IPipelineStep
{
public:
    AgcStep(int sampleRate, bool enableLimiter = true, bool enableLeveler = true);
    virtual ~AgcStep();
    
    virtual int getInputSampleRate() const FREEDV_NONBLOCKING override;
    virtual int getOutputSampleRate() const FREEDV_NONBLOCKING override;
    virtual short* execute(short* inputSamples, int numInputSamples, int* numOutputSamples) FREEDV_NONBLOCKING override;
    virtual void reset() FREEDV_NONBLOCKING override;
    
private:
    int sampleRate_;
    float targetGainDb_;
    float currentGainDb_;
    WebRtcAgcConfig agcConfig_;
    void* agcState_;

    void* ebur128State_;

    int numSamplesPerRun_;
    int blocksSinceLoudnessUpdate_;
    bool lastMeasurementValid_;
    GenericFIFO<short> inputSampleFifo_;
    std::unique_ptr<short[]> outputSamples_;
    std::unique_ptr<short[]> tmpInput_;
    std::unique_ptr<float[]> tmpInputFloat_;

    bool enableLimiter_;
    bool enableLeveler_;

    // COMPARISON-BRANCH ONLY (1485-diag-tests, 2026-09-28) -- writes the
    // same 7-column schema LevelerStep/CompressorLimiterStep use on
    // bcj-backend-audio-dev (elapsed_ms,input_dbfs,feedback_lufs,
    // leveler_target_gain_db,leveler_current_gain_db,
    // comp_limiter_gain_reduction_db,output_dbfs) so the existing
    // agc_ptt_capture.sh/agc_diag_wide_plot.py tooling works unchanged
    // against this branch for a direct before/after comparison. Not
    // gated behind a CMake option (unlike the other branch) since this
    // is explicitly a throwaway test branch, never intended to merge --
    // see PR #1485's own "DO NOT MERGE - TEST BRANCH ONLY".
    // comp_limiter_gain_reduction_db here is an approximation: the cubic
    // soft-clip has no single "gain" value the way a real limiter does,
    // so it's measured as 20*log10(peakOut/peakIn) across just the clip
    // stage itself (post-leveler, pre-clip vs. post-clip), isolating its
    // effect from the leveler's own gain the same way the real
    // CompressorLimiterStep's column isolates its own action.
    float lastLoggedLufs_;
    FILE* diagCsvFile_;
    std::chrono::steady_clock::time_point diagStartTime_;
};


#endif // AUDIO_PIPELINE__AGC_STEP_H
