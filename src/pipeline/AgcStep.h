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

#include <atomic>
#include <memory>
#include <cstdio>
#include <chrono>

class AgcStep : public IPipelineStep
{
public:
    // gainOutputDb, if non-null, is updated with currentGainDb_ at the end
    // of every execute() call -- lets a caller (e.g. a GUI meter) read the
    // live gain value from another thread without needing a pointer to
    // this AgcStep instance itself, whose lifetime is tied to the audio
    // pipeline and can be rebuilt/destroyed independently of anything
    // polling it. The pointed-to atomic must outlive this object; the
    // caller owns it and is expected to keep it around for as long as
    // anything might still be reading it (a static/global works well).
    AgcStep(int sampleRate, std::atomic<float>* gainOutputDb = nullptr);
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
    GenericFIFO<short> inputSampleFifo_;
    std::unique_ptr<short[]> outputSamples_;
    std::unique_ptr<short[]> tmpInput_;

    std::atomic<float>* gainOutputDb_;

    // DIAGNOSTIC ONLY: logs each 10ms block's input loudness, target/current
    // AGC gain, and post-AGC output level to ~/agc_diag.csv, for offline
    // gnuplot analysis of the AGC loop's convergence behavior. Not for
    // production use -- see the bcj-agc-diagnostic-log branch. Only this
    // step's own single owning thread ever calls execute(), so no locking
    // is needed around this file handle.
    FILE* diagLogFile_;
    std::chrono::steady_clock::time_point diagLogStartTime_;
};


#endif // AUDIO_PIPELINE__AGC_STEP_H
