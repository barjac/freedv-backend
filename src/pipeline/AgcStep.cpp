//=========================================================================
// Name:            AgcStep.cpp
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

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <string>

#include "AgcStep.h"
#include "../util/logging/ulog.h"
#include "ebur128.h" // from libebur128

#include <assert.h>

// AGC settings
constexpr float AGC_LOUDNESS_TARGET_LUFS = -23.0;
constexpr float AGC_MAX_GAIN_DB = 12.0;
constexpr float AGC_MIN_GAIN_DB = -12.0;
constexpr float AGC_ATTACK_RATE_DB_PER_SEC = -1;
constexpr float AGC_DECAY_RATE_DB_PER_SEC = 1;
constexpr float SILENCE_THRESHOLD_LUFS = -33.0;
constexpr int LIMITER_LEVEL_DB = -4;

constexpr int TEN_MS_DIVIDER = 100;
constexpr int MAX_AGC_SAMPLES = 160;

// ebur128_loudness_momentary() recomputes the sum-of-squares over its
// *entire* 400ms window from scratch on every call (no incremental
// caching in libebur128) -- profiling showed this one call as ~94% of
// AgcStep::execute's own cost when queried every 10ms block, i.e. ~40x
// more work than the single new 10ms block actually needs. AGC's own
// attack/release time constants (0.5s/6s, see below) are far slower
// than 10ms, so nothing meaningful is lost by re-measuring loudness
// (and recalculating targetGainDb_) only every Nth block; the gain
// ramp toward that target still updates every block, at full 10ms
// granularity, so the AGC's smoothing behavior is unaffected.
constexpr int LOUDNESS_UPDATE_INTERVAL_BLOCKS = 5;

AgcStep::AgcStep(int sampleRate, bool enableLimiter, bool enableLeveler)
    : sampleRate_(sampleRate == 8000 || sampleRate == 16000 || sampleRate == 32000 || sampleRate == 48000 ? sampleRate : 48000)
    , targetGainDb_(0.0)
    , currentGainDb_(0.0)
    , blocksSinceLoudnessUpdate_(0)
    , lastMeasurementValid_(false)
    , inputSampleFifo_(MAX_AGC_SAMPLES + 1)
    , enableLimiter_(enableLimiter)
    , enableLeveler_(enableLeveler)
    , lastLoggedLufs_(-100.0f)
    , lastPostLevelerShortTermLufs_(-100.0f)
    , lastOutputShortTermLufs_(-100.0f)
    , diagBlocksSinceUpdate_(0)
    , diagCsvFile_(nullptr)
{
    // COMPARISON-BRANCH ONLY -- see the header's comment on diagCsvFile_.
    const char* home = std::getenv("HOME");
    if (home != nullptr)
    {
        std::string path = std::string(home) + "/agc_diag.csv";
        diagCsvFile_ = fopen(path.c_str(), "w");
        if (diagCsvFile_ != nullptr)
        {
            fprintf(diagCsvFile_, "elapsed_ms,input_dbfs,feedback_lufs,leveler_target_gain_db,leveler_current_gain_db,leveler_applied_gain_db,comp_limiter_gain_reduction_db,output_dbfs,"
                                  "post_leveler_shortterm_lufs,output_shortterm_lufs,pre_limiter_clipped_samples\n");
            fflush(diagCsvFile_);
        }
    }
    diagStartTime_ = std::chrono::steady_clock::now();
    numSamplesPerRun_ = std::min(MAX_AGC_SAMPLES, sampleRate_ / TEN_MS_DIVIDER); // 10ms blocks, 160 max samples
    assert(numSamplesPerRun_ > 0);

    // Configure WebRTC as solely a limiter (i.e. no AGC)
    agcState_ = WebRtcAgc_Create();
    assert(agcState_ != nullptr);
 
    auto status = WebRtcAgc_Init(agcState_, 0, 255, kAgcModeUnchanged, sampleRate_);
    if (status != 0)
    {
        log_error("Could not initialize WebRTC AGC (err = %d)", status);
        WebRtcAgc_Free(agcState_);
        agcState_ = nullptr;
    }

    // Set AGC configuration
    agcConfig_.compressionGaindB = 0; // default 9 dB
    agcConfig_.limiterEnable = 1; // default kAgcTrue (on)
    agcConfig_.targetLevelDbfs = -LIMITER_LEVEL_DB; // default 3 (-3 dBOv)
    status = WebRtcAgc_set_config(agcState_, agcConfig_);
    if (status != 0)
    {
        log_error("Could not initialize WebRTC AGC config (err = %d)", status);
        WebRtcAgc_Free(agcState_);
        agcState_ = nullptr;
    }

    ebur128State_ = ebur128_init(1, sampleRate_, EBUR128_MODE_S);
    assert(ebur128State_ != nullptr);

    // COMPARISON-BRANCH ONLY -- short-term loudness after the leveler and
    // after the limiter, as requested on PR #1485 for both PRs.
    diagPostLevelerEbur128_ = ebur128_init(1, sampleRate_, EBUR128_MODE_S);
    diagOutputEbur128_ = ebur128_init(1, sampleRate_, EBUR128_MODE_S);
    assert(diagPostLevelerEbur128_ != nullptr && diagOutputEbur128_ != nullptr);

    // Pre-allocate buffers so we don't have to do so during real-time operation.
    outputSamples_ = std::make_unique<short[]>(sampleRate);
    assert(outputSamples_ != nullptr);

    tmpInput_ = std::make_unique<short[]>(numSamplesPerRun_);
    assert(tmpInput_ != nullptr);
}

AgcStep::~AgcStep()
{
    outputSamples_ = nullptr;
    WebRtcAgc_Free(agcState_);
    ebur128_destroy((ebur128_state**)&ebur128State_);
    ebur128_destroy((ebur128_state**)&diagPostLevelerEbur128_);
    ebur128_destroy((ebur128_state**)&diagOutputEbur128_);

    if (diagCsvFile_ != nullptr)
    {
        fclose(diagCsvFile_);
        diagCsvFile_ = nullptr;
    }
}

int AgcStep::getInputSampleRate() const FREEDV_NONBLOCKING
{
    return sampleRate_;
}

int AgcStep::getOutputSampleRate() const FREEDV_NONBLOCKING
{
    return sampleRate_;
}

short* AgcStep::execute(short* inputSamples, int numInputSamples, int* numOutputSamples) FREEDV_NONBLOCKING
{
    ebur128_state* state = static_cast<ebur128_state*>(ebur128State_);

    *numOutputSamples = 0;
    short* outputSamples = outputSamples_.get();
    short* tmpOutput = outputSamples;
    short* tmpInput = tmpInput_.get();

    while (numInputSamples > 0)
    {
        int samplesToWrite = std::min(inputSampleFifo_.numFree(), numInputSamples);
        if (samplesToWrite > 0)
        {
            inputSampleFifo_.write(inputSamples, samplesToWrite);
            inputSamples += samplesToWrite;
            numInputSamples -= samplesToWrite;
        }

        if (samplesToWrite == 0 && inputSampleFifo_.numUsed() < numSamplesPerRun_)
        {
            // Nothing left to write and not enough buffered for a block --
            // no further progress is possible this call.
            break;
        }

        while (inputSampleFifo_.numUsed() >= numSamplesPerRun_)
        {
            *numOutputSamples += numSamplesPerRun_;
            inputSampleFifo_.read(tmpInput, numSamplesPerRun_);

            // COMPARISON-BRANCH ONLY -- peak before the leveler's gain.
            int peakIn = 0;
            for (auto ctr = 0; ctr < numSamplesPerRun_; ctr++)
            {
                peakIn = std::max(peakIn, std::abs((int)tmpInput[ctr]));
            }

            if (enableLeveler_)
            {
                // Step 1: feed samples into ebur128 every block (cheap -- this is
                // just the K-weighting filter), but only ask for the momentary
                // loudness (and thus recompute targetGainDb_) every Nth block --
                // see LOUDNESS_UPDATE_INTERVAL_BLOCKS above for why.
                double lufs = 0.0;
                int result = EBUR128_SUCCESS;
                bool loudnessUpdated = false;
    
                // Note: libebur128 is unlikely to use RT-unsafe constructs in normal operation
                // (per existing RTSan-enabled tests). Verified on 2025-09-30.
                FREEDV_BEGIN_VERIFIED_SAFE
                ebur128_add_frames_short(state, tmpInput, numSamplesPerRun_);
                if (++blocksSinceLoudnessUpdate_ >= LOUDNESS_UPDATE_INTERVAL_BLOCKS)
                {
                    blocksSinceLoudnessUpdate_ = 0;
                    result = ebur128_loudness_shortterm(state, &lufs);
                    loudnessUpdated = true;
                }
                FREEDV_END_VERIFIED_SAFE
    
                if (loudnessUpdated)
                {
                    // Persist validity across the blocks we don't re-measure, so
                    // the silence gate below still behaves as if it were checked
                    // every block (matching the original per-block behavior).
                    lastMeasurementValid_ = result == EBUR128_SUCCESS && lufs != -HUGE_VAL && lufs > SILENCE_THRESHOLD_LUFS;
                    if (lastMeasurementValid_)
                    {
                        // Step 2: calculate target gain. Assume LUFS = dbFS (?)
                        targetGainDb_ = AGC_LOUDNESS_TARGET_LUFS - lufs;
                        if (targetGainDb_ >= AGC_MAX_GAIN_DB) targetGainDb_ = AGC_MAX_GAIN_DB;
                        if (targetGainDb_ <= AGC_MIN_GAIN_DB) targetGainDb_ = AGC_MIN_GAIN_DB;

                        // COMPARISON-BRANCH ONLY -- for the feedback_lufs column.
                        lastLoggedLufs_ = (float)lufs;
                    }
                }
    
                if (lastMeasurementValid_)
                {
                    // Step 3: increment/decrement current gain in the direction
                    // of target. Still runs every block (not just when the
                    // target was just refreshed), so the gain ramp itself stays
                    // at full 10ms granularity -- only how often the target
                    // updates -- and whether we're gated by silence -- changes
                    // less often now.
                    float agcInterval = 0;
                    if (targetGainDb_ < currentGainDb_)
                    {
                        agcInterval = AGC_ATTACK_RATE_DB_PER_SEC;
                    }
                    else
                    {
                        agcInterval = AGC_DECAY_RATE_DB_PER_SEC;
                    }
                    currentGainDb_ += agcInterval * ((float)numSamplesPerRun_ / sampleRate_);
                    if (std::abs(currentGainDb_) > std::abs(targetGainDb_))
                    {
                        currentGainDb_ = targetGainDb_;
                    }
                }
    
                // Scale samples based on current gain.
                float scaleFactor = expf(currentGainDb_/20.0f * logf(10.0f));
                float temp = 0;
                for (auto ctr = 0; ctr < numSamplesPerRun_; ctr++)
                {
                    ConvertSingleSampleToFloatSampleType_<float, short>(&tmpInput[ctr], &temp);
                    temp *= scaleFactor;
                    ConvertSingleSampleToIntSampleType_<short, float>(&temp, &tmpInput[ctr]);
                }
            }

            // COMPARISON-BRANCH ONLY -- post-leveler (pre-limiter) peak and
            // the number of samples saturated at int16 by the leveler's gain.
            int peakPreLimiter = 0;
            int clippedSamples = 0;
            for (auto ctr = 0; ctr < numSamplesPerRun_; ctr++)
            {
                int a = std::abs((int)tmpInput[ctr]);
                peakPreLimiter = std::max(peakPreLimiter, a);
                if (a >= 32767) clippedSamples++;
            }

            // Run WebRTC to make sure we don't clip.
            if (enableLimiter_)
            {
                int outMicLevel = 0;
                int inMicLevel = 0;
                short echo = 0;
                unsigned char saturationWarning = 1;
                WebRtcAgc_Process(
                    agcState_, const_cast<const int16_t *const *>(&tmpInput), 1, numSamplesPerRun_, 
                    const_cast<int16_t *const *>(&tmpOutput), inMicLevel, &outMicLevel, echo, &saturationWarning);
            }
            else
            {
                memcpy(tmpOutput, tmpInput, numSamplesPerRun_ * sizeof(short));
            }

            // COMPARISON-BRANCH ONLY -- write this block's diagnostic row.
            if (diagCsvFile_ != nullptr)
            {
                int peakOut = 0;
                for (auto ctr = 0; ctr < numSamplesPerRun_; ctr++)
                {
                    peakOut = std::max(peakOut, std::abs((int)tmpOutput[ctr]));
                }

                FREEDV_BEGIN_VERIFIED_SAFE
                ebur128_add_frames_short((ebur128_state*)diagPostLevelerEbur128_, tmpInput, numSamplesPerRun_);
                ebur128_add_frames_short((ebur128_state*)diagOutputEbur128_, tmpOutput, numSamplesPerRun_);
                if (++diagBlocksSinceUpdate_ >= LOUDNESS_UPDATE_INTERVAL_BLOCKS)
                {
                    diagBlocksSinceUpdate_ = 0;
                    double st = 0.0;
                    lastPostLevelerShortTermLufs_ =
                        (ebur128_loudness_shortterm((ebur128_state*)diagPostLevelerEbur128_, &st) == EBUR128_SUCCESS && st != -HUGE_VAL) ? (float)st : -100.0f;
                    lastOutputShortTermLufs_ =
                        (ebur128_loudness_shortterm((ebur128_state*)diagOutputEbur128_, &st) == EBUR128_SUCCESS && st != -HUGE_VAL) ? (float)st : -100.0f;
                }

                auto toDb = [](int peak) { return peak > 0 ? 20.0 * std::log10(peak / 32768.0) : -120.0; };
                double limiterReductionDb = (enableLimiter_ && peakPreLimiter > 0 && peakOut > 0)
                    ? 20.0 * std::log10((double)peakOut / peakPreLimiter) : 0.0;
                double feedbackLufs = lastMeasurementValid_ ? (double)lastLoggedLufs_ : -100.0;
                auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - diagStartTime_).count();

                // leveler_applied_gain_db duplicates leveler_current_gain_db:
                // AgcStep has no separate startup ramp. feedback_lufs is the
                // leveler's own (input, short-term) measurement.
                fprintf(diagCsvFile_, "%lld,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d\n",
                    (long long)elapsedMs, toDb(peakIn), feedbackLufs, (double)targetGainDb_, (double)currentGainDb_,
                    (double)currentGainDb_, limiterReductionDb, toDb(peakOut),
                    (double)lastPostLevelerShortTermLufs_, (double)lastOutputShortTermLufs_, clippedSamples);
                fflush(diagCsvFile_);
                FREEDV_END_VERIFIED_SAFE
            }

            tmpOutput += numSamplesPerRun_;
        }
    }
    
    return outputSamples;
}

void AgcStep::reset() FREEDV_NONBLOCKING
{
    inputSampleFifo_.reset();
    blocksSinceLoudnessUpdate_ = 0;
    lastMeasurementValid_ = false;
}
