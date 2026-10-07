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

    bool enableLimiter_;
    bool enableLeveler_;

    // COMPARISON-BRANCH ONLY (1485-diag-tests). Writes ~/agc_diag.csv in
    // the same schema as freedv-backend's DiagnosticCsvLogger, so the
    // existing capture/plot tools work unchanged, plus three extra columns
    // at the end: short-term loudness after the leveler and after the
    // WebRTC limiter (requested on PR #1485), and the number of samples
    // saturated at int16 by the leveler's gain before the limiter.
    // comp_limiter_gain_reduction_db is 20*log10(peakOut/peakPreLimiter)
    // per block. Not gated behind a CMake option: this is a throwaway
    // test branch, never intended to merge.
    float lastLoggedLufs_;
    float lastPostLevelerShortTermLufs_;
    float lastOutputShortTermLufs_;
    int diagBlocksSinceUpdate_;
    void* diagPostLevelerEbur128_;
    void* diagOutputEbur128_;
    FILE* diagCsvFile_;
    std::chrono::steady_clock::time_point diagStartTime_;
};


#endif // AUDIO_PIPELINE__AGC_STEP_H
