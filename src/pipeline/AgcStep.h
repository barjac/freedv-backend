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

#include <memory>
#include <cstdio>

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
    GenericFIFO<short> inputSampleFifo_;
    std::unique_ptr<short[]> outputSamples_;
    std::unique_ptr<short[]> tmpInput_;

    bool enableLimiter_;
    bool enableLeveler_;

    // DIAGNOSTIC ONLY: logs each 10ms block's input loudness, target/current
    // AGC gain, and post-AGC output level to ~/agc_diag.csv, for offline
    // gnuplot analysis of the AGC loop's convergence behavior. Not for
    // production use -- see the bcj-agc-diagnostic-log branch. Only this
    // step's own single owning thread ever calls execute(), so no locking
    // is needed around this file handle.
    //
    // diagLogSampleCount_ is a running count of audio samples processed,
    // used to timestamp each row by its position in the audio stream
    // (sample count * 1000 / sampleRate_) rather than by wall-clock time --
    // execute() gets called in bursts of several 10ms blocks at once
    // (whenever the upstream pipeline hands over a chunk), not one block
    // per 10ms of real elapsed time, so a wall-clock timestamp stamped
    // several genuinely-sequential blocks from one burst with the same (or
    // near-identical) millisecond, while the next burst's first block jumps
    // the timestamp forward by the whole inter-burst gap. Plotted with
    // lines, that produced clusters of points stacked at nearly the same x
    // position (a real value spread of ~10dB within one burst was seen)
    // joined by long diagonal connectors to the next cluster -- looking
    // like a row of vertical lines joined by sloping lines, not the smooth
    // audio-time trace intended. An audio-sample-based timestamp is exactly
    // 10ms apart for every row regardless of when the CPU got around to
    // writing it, so this can't happen.
    FILE* diagLogFile_;
    long long diagLogSampleCount_;
};


#endif // AUDIO_PIPELINE__AGC_STEP_H
