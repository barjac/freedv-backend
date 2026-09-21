//=========================================================================
// Name:            PostLoopCompressorStep.cpp
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

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <string>

#include "PostLoopCompressorStep.h"
#include "freedv_sanitizers.h"

// Same values as the original (pre-2026-09-18) two-knee design in
// CompressorLimiterStep.cpp -- starting points for live A/B tuning via
// this stage's own independent toggle, not re-derived from scratch. Knee A
// ("compressor") pushed close to the ceiling so it engages only on
// genuinely loud excursions, not ordinary speech (RADE's neural encoder
// was almost certainly trained on uncompressed speech -- Barry,
// 2026-09-15); knee B ("limiter") a tighter, near-infinite-ratio backstop
// right at the ceiling.
constexpr float KNEE_A_THRESHOLD_DB = -6.0f;
constexpr float KNEE_A_RATIO = 2.0f;
constexpr float KNEE_A_WIDTH_DB = 2.0f;
constexpr float KNEE_B_THRESHOLD_DB = -1.5f;
constexpr float KNEE_B_RATIO = 20.0f;
constexpr float KNEE_B_WIDTH_DB = 2.0f;

// Same reasoning as CompressorLimiterStep.cpp's own constants -- floored
// attack to avoid the gain command itself generating harmonics by moving
// within less than one low-frequency voice-band cycle, ~250ms release per
// spec, short look-ahead so gain can start dropping before the
// corresponding (delayed) peak reaches the output.
constexpr float ATTACK_TIME_SEC = 0.003f;
constexpr float RELEASE_TIME_SEC = 0.25f;
constexpr float LOOKAHEAD_TIME_SEC = 0.004f;

constexpr float LEVEL_FLOOR_DB = -120.0f; // for log10(0) avoidance
constexpr int TEN_MS_DIVIDER = 100;

namespace {

// Standard two-parameter soft-knee compressor curve (Giannoulis/Massberg/
// Reiss). Deliberately duplicated from CompressorLimiterStep.cpp rather
// than shared -- see this class's own header comment on why it must not
// share any code/state with that class. Returns the *output* level (dB)
// for a given *input* level (dB); caller subtracts input from the result
// to get the gain-reduction amount.
float softKneeGainDb(float levelDb, float thresholdDb, float ratio, float kneeWidthDb)
{
    float overshoot = levelDb - thresholdDb;
    if (2.0f * overshoot < -kneeWidthDb)
    {
        // Below the knee entirely -- no change.
        return levelDb;
    }
    else if (2.0f * std::fabs(overshoot) <= kneeWidthDb)
    {
        // Inside the knee -- smooth quadratic transition.
        float kneeTerm = overshoot + kneeWidthDb / 2.0f;
        return levelDb + (1.0f / ratio - 1.0f) * (kneeTerm * kneeTerm) / (2.0f * kneeWidthDb);
    }
    else
    {
        // Above the knee -- straight-line compression at the given ratio.
        return thresholdDb + overshoot / ratio;
    }
}

} // namespace

PostLoopCompressorStep::PostLoopCompressorStep(int sampleRate)
    : sampleRate_(sampleRate)
    , lookAheadLength_(std::max(1, (int)std::lround(sampleRate * LOOKAHEAD_TIME_SEC)))
    , lookAheadBuffer_(std::make_unique<float[]>(lookAheadLength_)) // value-initialized (zeroed)
    , lookAheadPos_(0)
    , smoothedGainReductionDb_(0.0f)
    , diagFile_(nullptr)
{
    assert(lookAheadBuffer_ != nullptr);

    // Pre-allocate buffers so we don't have to do so during real-time operation.
    outputSamples_ = std::make_unique<short[]>(sampleRate_);
    assert(outputSamples_ != nullptr);

    // Precompute one-pole smoothing coefficients (fixed since sampleRate_
    // is fixed at construction): alpha = 1 - exp(-dt/tau), dt = 1 sample.
    float dt = 1.0f / sampleRate_;
    attackAlpha_ = 1.0f - expf(-dt / ATTACK_TIME_SEC);
    releaseAlpha_ = 1.0f - expf(-dt / RELEASE_TIME_SEC);

    // DIAGNOSTIC ONLY -- see class comment in the header. Own file, own
    // fopen -- deliberately not routed through DiagnosticCsvLogger.
#if defined(FREEDV_ENABLE_AUDIO_DIAG_LOGGING)
    const char* home = std::getenv("HOME");
    if (home != nullptr)
    {
        std::string path = std::string(home) + "/postloop_compressor_diag.csv";
        diagFile_ = fopen(path.c_str(), "w");
        if (diagFile_ != nullptr)
        {
            // encoder_input_dbfs, not output_dbfs (2026-09-21, Barry:
            // "That measurement should have been the encoder input") --
            // this stage genuinely is the last thing to touch the signal
            // before freedvInterface.createTransmitPipeline() (the actual
            // RADE encoder call) in TxRxThread.cpp; nothing after it but
            // non-modifying taps. Naming it plainly avoids the same stale-
            // label confusion the shared ~/agc_diag.csv's own
            // "output_dbfs"/"Input to Encoder" legend now has, now that
            // this stage sits downstream of what that file measures.
            fprintf(diagFile_, "elapsed_ms,input_dbfs,gain_reduction_db,encoder_input_dbfs\n");
            fflush(diagFile_);
        }
    }
#endif // defined(FREEDV_ENABLE_AUDIO_DIAG_LOGGING)
    diagStartTime_ = std::chrono::steady_clock::now();
}

PostLoopCompressorStep::~PostLoopCompressorStep()
{
    if (diagFile_ != nullptr)
    {
        fclose(diagFile_);
        diagFile_ = nullptr;
    }
}

int PostLoopCompressorStep::getInputSampleRate() const FREEDV_NONBLOCKING
{
    return sampleRate_;
}

int PostLoopCompressorStep::getOutputSampleRate() const FREEDV_NONBLOCKING
{
    return sampleRate_;
}

short* PostLoopCompressorStep::execute(short* inputSamples, int numInputSamples, int* numOutputSamples) FREEDV_NONBLOCKING
{
    int tenMsSamples = std::max(1, sampleRate_ / TEN_MS_DIVIDER);

    short* inPtr = inputSamples;
    short* outPtr = outputSamples_.get();
    *numOutputSamples = numInputSamples;

    int remaining = numInputSamples;
    while (remaining > 0)
    {
        int chunkSize = std::min(remaining, tenMsSamples);
        double peakInAbs = 0.0;
        double peakOutAbs = 0.0;

        for (int i = 0; i < chunkSize; i++)
        {
            float currentSample = 0.0f;
            ConvertSingleSampleToFloatSampleType_<float, short>(&inPtr[i], &currentSample);
            double inAbs = std::fabs((double)currentSample);
            if (inAbs > peakInAbs) peakInAbs = inAbs;

            // Step 1: per-sample envelope detection on the *pre-delay*
            // signal, same as CompressorLimiterStep.
            float absVal = std::fabs(currentSample);
            float levelDb = absVal > 0.0f ? 20.0f * log10f(absVal) : LEVEL_FLOOR_DB;

            // Step 2: static soft-knee gain curve, both knees in series --
            // never adds gain, only ever reduces it (no makeup-gain term
            // anywhere in this function).
            float kneeAOutDb = softKneeGainDb(levelDb, KNEE_A_THRESHOLD_DB, KNEE_A_RATIO, KNEE_A_WIDTH_DB);
            float kneeBOutDb = softKneeGainDb(kneeAOutDb, KNEE_B_THRESHOLD_DB, KNEE_B_RATIO, KNEE_B_WIDTH_DB);
            float staticGainReductionDb = kneeBOutDb - levelDb; // <= 0

            // Step 3: smooth the *gain-reduction command* (not the level),
            // fast attack when reduction is increasing, slow release when
            // recovering toward unity.
            float alpha = (staticGainReductionDb < smoothedGainReductionDb_) ? attackAlpha_ : releaseAlpha_;
            smoothedGainReductionDb_ += (staticGainReductionDb - smoothedGainReductionDb_) * alpha;

            // Step 4: look-ahead delay line (read-then-write same slot).
            float delayedSample = lookAheadBuffer_[lookAheadPos_];
            lookAheadBuffer_[lookAheadPos_] = currentSample;
            lookAheadPos_++;
            if (lookAheadPos_ >= lookAheadLength_) lookAheadPos_ = 0;

            // Step 5: apply smoothed gain to the delayed sample, convert
            // back to int16 -- the existing int16 saturate remains a true
            // last-resort backstop.
            float scaleFactor = expf(smoothedGainReductionDb_ / 20.0f * logf(10.0f));
            float outSampleFloat = delayedSample * scaleFactor;
            ConvertSingleSampleToIntSampleType_<short, float>(&outSampleFloat, &outPtr[i]);

            double outAbs = std::fabs((double)outPtr[i]) / 32768.0;
            if (outAbs > peakOutAbs) peakOutAbs = outAbs;
        }

        // DIAGNOSTIC ONLY -- see class comment in the header.
        if (diagFile_ != nullptr)
        {
            auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - diagStartTime_).count();
            // peakInAbs is already normalized (ConvertSingleSampleToFloatSampleType_'s
            // convention, ~[-1,1]) -- NOT raw int16 magnitude, so no /32768 here
            // (unlike peakOutAbs above, which is deliberately computed from the
            // raw post-conversion short).
            double inputDbfs = peakInAbs > 0.0 ? 20.0 * std::log10(peakInAbs) : -100.0;
            double encoderInputDbfs = peakOutAbs > 0.0 ? 20.0 * std::log10(peakOutAbs) : -100.0;

            FREEDV_BEGIN_VERIFIED_SAFE
            fprintf(diagFile_, "%lld,%.2f,%.2f,%.2f\n",
                (long long)elapsedMs, inputDbfs, (double)smoothedGainReductionDb_, encoderInputDbfs);
            fflush(diagFile_);
            FREEDV_END_VERIFIED_SAFE
        }

        inPtr += chunkSize;
        outPtr += chunkSize;
        remaining -= chunkSize;
    }

    return outputSamples_.get();
}

void PostLoopCompressorStep::reset() FREEDV_NONBLOCKING
{
    smoothedGainReductionDb_ = 0.0f;
    lookAheadPos_ = 0;
    for (int i = 0; i < lookAheadLength_; i++)
    {
        lookAheadBuffer_[i] = 0.0f;
    }
}
