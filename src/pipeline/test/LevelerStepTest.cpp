#include <algorithm>
#include <atomic>
#include <cmath>
#include <memory>
#include <vector>

#include "LevelerStep.h"
#include "CompressorLimiterStep.h"
#include "PipelineTestCommon.h"

// Unit checks of the gain-control logic against a synthetic feedback value.
// End-to-end behaviour (in the real feedback loop with
// CompressorLimiterStep, on real speech) is verified with the diagnostic CSV
// logging instead.

namespace {

// LevelerStep hands its output to CompressorLimiterStep INPUT_HEADROOM_DB
// below true level; add it back when measuring the leveler's gain.
constexpr double HANDOVER_DB = CompressorLimiterStep::INPUT_HEADROOM_DB;

// realtime_fp<float()> can only hold a plain captureless function pointer
// (see LevelAdjustStep's own usage precedent), so the synthetic feedback
// value is file-scope state the test sets before each run.
std::atomic<float> g_testFeedbackLufs{-23.0f};

float testFeedbackFn() FREEDV_NONBLOCKING
{
    return g_testFeedbackLufs.load(std::memory_order_relaxed);
}

// Same idiom, for the RNNoise-enabled state LevelerStep's silence threshold
// depends on.
std::atomic<bool> g_testNoiseReductionEnabled{true};

bool testNoiseReductionEnabledFn() FREEDV_NONBLOCKING
{
    return g_testNoiseReductionEnabled.load(std::memory_order_relaxed);
}

// Streams the given signal through the leveler in small chunks, mimicking
// real-time usage, and returns the concatenated output.
std::vector<short> runThroughLeveler(LevelerStep& step, std::vector<short>& input, int chunkSize)
{
    std::vector<short> output;
    output.reserve(input.size());

    for (std::size_t offset = 0; offset < input.size(); offset += chunkSize)
    {
        int numToWrite = static_cast<int>(std::min<std::size_t>(chunkSize, input.size() - offset));
        int numOutputSamples = 0;
        short* result = step.execute(&input[offset], numToWrite, &numOutputSamples);
        output.insert(output.end(), result, result + numOutputSamples);
    }

    return output;
}

double measureRms(const std::vector<short>& samples)
{
    double sumSq = 0.0;
    for (short s : samples)
    {
        sumSq += (double)s * s;
    }
    return std::sqrt(sumSq / samples.size());
}

} // namespace

// Simulates the closed loop: feedback for the next chunk is the fixed true
// input level plus the gain actually applied. (A constant feedback value
// can't exercise the PI controller, which relies on feedback responding to
// applied gain.) Gain should converge on the full correction,
// -23 - trueInputLufs.
bool levelerConvergesTowardExpectedGainForQuietFeedback()
{
    constexpr int sampleRate = 8000;
    constexpr double TOLERANCE_DB = 1.0;

    constexpr float trueInputLufs = -33.0f + 0.01f;
    g_testFeedbackLufs.store(trueInputLufs); // first chunk: no gain applied yet

    LevelerStep step(sampleRate, +testFeedbackFn, std::make_shared<DiagnosticCsvLogger>());

    std::unique_ptr<short[]> rawInput(generateOneSecondSineWave(1000.0f, sampleRate));
    std::vector<short> inputVec(rawInput.get(), rawInput.get() + sampleRate);
    double inputRms = measureRms(inputVec);

    std::vector<short> lastOutput;
    for (int second = 0; second < 60; second++) // several time constants (10s each) to converge
    {
        lastOutput = runThroughLeveler(step, inputVec, sampleRate / 10);

        double chunkGainDb = 20.0 * std::log10(measureRms(lastOutput) / inputRms) + HANDOVER_DB;
        g_testFeedbackLufs.store((float)(trueInputLufs + chunkGainDb));
    }

    double gainDb = 20.0 * std::log10(measureRms(lastOutput) / inputRms) + HANDOVER_DB;
    double expectedGainDb = -23.0 - trueInputLufs;

    if (std::abs(gainDb - expectedGainDb) > TOLERANCE_DB)
    {
        std::cerr << "[gain=" << gainDb << "dB, expected ~" << expectedGainDb << "dB]...";
        return false;
    }

    return true;
}

// A non-default targetLufs should change what the closed loop converges to.
bool levelerConvergesTowardConfigurableTarget()
{
    constexpr int sampleRate = 8000;
    constexpr double TOLERANCE_DB = 1.0;
    constexpr float customTargetLufs = -28.0f; // deliberately not the -23.0f default

    constexpr float trueInputLufs = -33.0f + 0.01f;
    g_testFeedbackLufs.store(trueInputLufs);

    LevelerStep step(sampleRate, +testFeedbackFn, std::make_shared<DiagnosticCsvLogger>(),
                      0.0f, 0.0f, customTargetLufs);

    std::unique_ptr<short[]> rawInput(generateOneSecondSineWave(1000.0f, sampleRate));
    std::vector<short> inputVec(rawInput.get(), rawInput.get() + sampleRate);
    double inputRms = measureRms(inputVec);

    std::vector<short> lastOutput;
    for (int second = 0; second < 60; second++)
    {
        lastOutput = runThroughLeveler(step, inputVec, sampleRate / 10);

        double chunkGainDb = 20.0 * std::log10(measureRms(lastOutput) / inputRms) + HANDOVER_DB;
        g_testFeedbackLufs.store((float)(trueInputLufs + chunkGainDb));
    }

    double gainDb = 20.0 * std::log10(measureRms(lastOutput) / inputRms) + HANDOVER_DB;
    double expectedGainDb = customTargetLufs - trueInputLufs;

    if (std::abs(gainDb - expectedGainDb) > TOLERANCE_DB)
    {
        std::cerr << "[gain=" << gainDb << "dB, expected ~" << expectedGainDb
                   << "dB (converging toward the custom " << customTargetLufs << " LUFS target, not the -23.0f default)]...";
        return false;
    }

    return true;
}

// As soon as feedback drops below the silence threshold, gain should hold
// exactly where it is: no further movement from the first 10ms block of a
// pause onward, even while a correction is in progress.
bool levelerFreezesGainWhenFeedbackBelowSilenceThreshold()
{
    constexpr int sampleRate = 8000;
    constexpr int tenMsSamples = sampleRate / 100;
    constexpr double TOLERANCE_DB = 0.01;

    g_testFeedbackLufs.store(-25.0f); // below target -- a correction in progress
    LevelerStep step(sampleRate, +testFeedbackFn, std::make_shared<DiagnosticCsvLogger>());

    std::unique_ptr<short[]> rawInput(generateOneSecondSineWave(1000.0f, sampleRate));
    std::vector<short> inputVec(rawInput.get(), rawInput.get() + sampleRate);

    auto beforePause = runThroughLeveler(step, inputVec, sampleRate / 10);
    std::vector<short> lastBlockBefore(beforePause.end() - tenMsSamples, beforePause.end());

    g_testFeedbackLufs.store(-40.0f); // below the -33 threshold -- a pause begins
    auto duringPause = runThroughLeveler(step, inputVec, sampleRate / 10);
    std::vector<short> firstBlockDuring(duringPause.begin(), duringPause.begin() + tenMsSamples);
    std::vector<short> lastBlockDuring(duringPause.end() - tenMsSamples, duringPause.end());

    double firstDiffDb = 20.0 * std::log10(measureRms(firstBlockDuring) / measureRms(lastBlockBefore));
    double laterDiffDb = 20.0 * std::log10(measureRms(lastBlockDuring) / measureRms(lastBlockBefore));
    if (std::abs(firstDiffDb) > TOLERANCE_DB || std::abs(laterDiffDb) > TOLERANCE_DB)
    {
        std::cerr << "[gain moved " << firstDiffDb << "dB in the first 10ms of a pause and " << laterDiffDb
                   << "dB after 1s, expected it held from the start]...";
        return false;
    }

    return true;
}

// The RNNoise-on and -off silence thresholds are currently both -33 LUFS.
// Checks that noiseReductionEnabledFn is consulted and that both states
// update above, and hold below, -33.
bool levelerThresholdBehavesTheSameBothWaysNow()
{
    constexpr int sampleRate = 8000;
    constexpr double TOLERANCE_DB = 0.01;
    constexpr float aboveThresholdLufs = -30.0f; // above -33 -- should update
    constexpr float belowThresholdLufs = -40.0f; // below -33 -- should freeze

    std::unique_ptr<short[]> rawInput(generateOneSecondSineWave(1000.0f, sampleRate));
    std::vector<short> inputVec(rawInput.get(), rawInput.get() + sampleRate);

    for (bool noiseReductionOn : {true, false})
    {
        g_testNoiseReductionEnabled.store(noiseReductionOn);
        const char* label = noiseReductionOn ? "ON" : "OFF";

        g_testFeedbackLufs.store(aboveThresholdLufs);
        LevelerStep stepAbove(sampleRate, +testFeedbackFn, std::make_shared<DiagnosticCsvLogger>(), 0.0f, 0.0f, -23.0f, +testNoiseReductionEnabledFn);
        auto aboveSnapshot1 = runThroughLeveler(stepAbove, inputVec, sampleRate / 10);
        auto aboveSnapshot2 = runThroughLeveler(stepAbove, inputVec, sampleRate / 10);
        double aboveGainDiffDb = 20.0 * std::log10(measureRms(aboveSnapshot2) / measureRms(aboveSnapshot1));
        if (std::abs(aboveGainDiffDb) < TOLERANCE_DB)
        {
            std::cerr << "[gain didn't move at all with RNNoise reported " << label
                       << " and feedback above the -33 threshold]...";
            g_testNoiseReductionEnabled.store(true);
            return false;
        }

        g_testFeedbackLufs.store(belowThresholdLufs);
        LevelerStep stepBelow(sampleRate, +testFeedbackFn, std::make_shared<DiagnosticCsvLogger>(), 0.0f, 0.0f, -23.0f, +testNoiseReductionEnabledFn);
        auto belowSnapshot1 = runThroughLeveler(stepBelow, inputVec, sampleRate / 10);
        auto belowSnapshot2 = runThroughLeveler(stepBelow, inputVec, sampleRate / 10);
        double belowGainDiffDb = 20.0 * std::log10(measureRms(belowSnapshot2) / measureRms(belowSnapshot1));
        if (std::abs(belowGainDiffDb) > TOLERANCE_DB)
        {
            std::cerr << "[gain drifted by " << belowGainDiffDb << "dB with RNNoise reported " << label
                       << " and feedback below the -33 threshold, expected it frozen]...";
            g_testNoiseReductionEnabled.store(true);
            return false;
        }
    }

    g_testNoiseReductionEnabled.store(true); // reset shared global -- no other test reads it, but keep tidy
    return true;
}

// reset() is called at the start of every transmission and should leave
// gain unchanged, so each transmission doesn't re-climb from 0dB.
bool levelerResetPreservesGain()
{
    constexpr int sampleRate = 8000;
    constexpr double TOLERANCE_DB = 0.5;

    constexpr float feedbackLufs = -33.0f + 0.01f;
    g_testFeedbackLufs.store(feedbackLufs);

    LevelerStep step(sampleRate, +testFeedbackFn, std::make_shared<DiagnosticCsvLogger>());

    std::unique_ptr<short[]> rawInput(generateOneSecondSineWave(1000.0f, sampleRate));
    std::vector<short> inputVec(rawInput.get(), rawInput.get() + sampleRate);

    std::vector<short> beforeReset;
    for (int second = 0; second < 5; second++)
    {
        beforeReset = runThroughLeveler(step, inputVec, sampleRate / 10);
    }
    double gainBeforeDb = 20.0 * std::log10(measureRms(beforeReset) / measureRms(inputVec)) + HANDOVER_DB;

    step.reset();

    int numOutputSamples = 0;
    short* result = step.execute(inputVec.data(), sampleRate / 10, &numOutputSamples);
    std::vector<short> output(result, result + numOutputSamples);

    double gainAfterDb = 20.0 * std::log10(measureRms(output) / measureRms(inputVec)) + HANDOVER_DB;
    if (std::abs(gainAfterDb - gainBeforeDb) > TOLERANCE_DB)
    {
        std::cerr << "[gain was " << gainBeforeDb << "dB before reset(), " << gainAfterDb
                   << "dB right after -- expected reset() to leave gain unchanged]...";
        return false;
    }

    return true;
}

// A leveler seeded with saved gain/integral state should report it back,
// ramp the applied gain in at startup, then settle at the seeded gain.
bool levelerCanBeSeededWithSavedGain()
{
    constexpr int sampleRate = 8000;
    constexpr float seededGainDb = 7.5f;
    // Consistent with seededGainDb when feedback is held at target
    // (targetGainDb_ == integralErrorDb_ / LEVELER_INTEGRAL_TIME_CONSTANT_SEC,
    // i.e. 7.5 * 4.0). Must be kept in sync by hand with that constant in
    // LevelerStep.cpp.
    constexpr float seededIntegralErrorDb = 30.0f;
    constexpr double TOLERANCE_DB = 0.5;

    g_testFeedbackLufs.store(-23.0f); // exactly at target -- gain shouldn't drift from the seeded value
    LevelerStep step(sampleRate, +testFeedbackFn, std::make_shared<DiagnosticCsvLogger>(), seededGainDb, seededIntegralErrorDb);

    if (std::abs(step.getCurrentGainDb() - seededGainDb) > TOLERANCE_DB ||
        std::abs(step.getIntegralErrorDb() - seededIntegralErrorDb) > TOLERANCE_DB)
    {
        std::cerr << "[getters reported gain=" << step.getCurrentGainDb() << "dB, integralError="
                   << step.getIntegralErrorDb() << "dB right after construction, expected " << seededGainDb
                   << "dB/" << seededIntegralErrorDb << "dB]...";
        return false;
    }

    // Amplitude 10000 (~-10.3dBFS) to clear REAL_AUDIO_PEAK_THRESHOLD
    // (-20dBFS) and start the ramp-in. The other tests use 1000
    // (~-30.3dBFS), which stays below it.
    std::unique_ptr<short[]> rawInput(generateOneSecondSineWave(10000.0f, sampleRate));
    std::vector<short> inputVec(rawInput.get(), rawInput.get() + sampleRate);
    double inputRms = measureRms(inputVec);

    // The first 100ms block falls within the 300ms ramp-in, so applied gain
    // should be well below the seeded value.
    int numOutputSamples = 0;
    short* result = step.execute(inputVec.data(), sampleRate / 10, &numOutputSamples);
    double firstBlockGainDb = 20.0 * std::log10(measureRms(std::vector<short>(result, result + numOutputSamples)) / inputRms) + HANDOVER_DB;
    if (firstBlockGainDb > seededGainDb - 3.0)
    {
        std::cerr << "[first (ramping-in) block's gain was " << firstBlockGainDb << "dB, expected it well below the seeded "
                   << seededGainDb << "dB -- ramp-in doesn't seem to be reducing applied gain at startup]...";
        return false;
    }

    // Well past the ramp-in, gain should be back at the seeded value since
    // feedback has been held at target throughout.
    std::vector<short> lastOutput;
    for (int block = 0; block < 5; block++)
    {
        result = step.execute(inputVec.data(), sampleRate / 10, &numOutputSamples);
        lastOutput.assign(result, result + numOutputSamples);
    }
    double convergedGainDb = 20.0 * std::log10(measureRms(lastOutput) / inputRms) + HANDOVER_DB;
    if (std::abs(convergedGainDb - seededGainDb) > TOLERANCE_DB)
    {
        std::cerr << "[gain after the ramp window was " << convergedGainDb << "dB, expected ~" << seededGainDb
                   << "dB from the seeded starting point]...";
        return false;
    }

    return true;
}

// The startup ramp-in must be counted from the first real audio, not from
// construction: in use there is idle time between pressing Start and
// speaking. Simulates several seconds of silence first, then checks the
// first block of real audio is still ramping in.
bool levelerRampInWaitsForRealAudioNotJustElapsedTime()
{
    constexpr int sampleRate = 8000;
    constexpr float seededGainDb = 7.5f;
    constexpr float seededIntegralErrorDb = 30.0f; // see levelerCanBeSeededWithSavedGain

    g_testFeedbackLufs.store(-23.0f);
    LevelerStep step(sampleRate, +testFeedbackFn, std::make_shared<DiagnosticCsvLogger>(), seededGainDb, seededIntegralErrorDb);

    // 3s of silence, simulating idle time before PTT.
    std::vector<short> silence(sampleRate / 10, 0);
    for (int block = 0; block < 30; block++) // 3 seconds
    {
        int numOutputSamples = 0;
        step.execute(silence.data(), (int)silence.size(), &numOutputSamples);
    }

    // First real audio (amplitude chosen as in levelerCanBeSeededWithSavedGain).
    std::unique_ptr<short[]> rawInput(generateOneSecondSineWave(10000.0f, sampleRate));
    std::vector<short> inputVec(rawInput.get(), rawInput.get() + sampleRate);
    double inputRms = measureRms(inputVec);

    int numOutputSamples = 0;
    short* result = step.execute(inputVec.data(), sampleRate / 10, &numOutputSamples);
    double firstRealBlockGainDb = 20.0 * std::log10(measureRms(std::vector<short>(result, result + numOutputSamples)) / inputRms) + HANDOVER_DB;
    if (firstRealBlockGainDb > seededGainDb - 3.0)
    {
        std::cerr << "[first block of real audio (after 3s of prior silence) had gain " << firstRealBlockGainDb
                   << "dB, expected it well below the seeded " << seededGainDb
                   << "dB -- ramp-in appears to be keyed to elapsed time rather than real audio arriving]...";
        return false;
    }

    return true;
}

int main()
{
    TEST_CASE(levelerConvergesTowardExpectedGainForQuietFeedback);
    TEST_CASE(levelerConvergesTowardConfigurableTarget);
    TEST_CASE(levelerFreezesGainWhenFeedbackBelowSilenceThreshold);
    TEST_CASE(levelerThresholdBehavesTheSameBothWaysNow);
    TEST_CASE(levelerResetPreservesGain);
    TEST_CASE(levelerCanBeSeededWithSavedGain);
    TEST_CASE(levelerRampInWaitsForRealAudioNotJustElapsedTime);
    return 0;
}
