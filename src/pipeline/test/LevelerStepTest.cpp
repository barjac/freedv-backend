#include <algorithm>
#include <atomic>
#include <cmath>
#include <memory>
#include <vector>

#include "LevelerStep.h"
#include "PipelineTestCommon.h"

// Unit checks of the gain-control logic against a synthetic feedback value.
// End-to-end behaviour (in the real feedback loop with
// CompressorLimiterStep, on real speech) is verified with the diagnostic CSV
// logging instead.

namespace {

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

        double chunkGainDb = 20.0 * std::log10(measureRms(lastOutput) / inputRms);
        g_testFeedbackLufs.store((float)(trueInputLufs + chunkGainDb));
    }

    double gainDb = 20.0 * std::log10(measureRms(lastOutput) / inputRms);
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

        double chunkGainDb = 20.0 * std::log10(measureRms(lastOutput) / inputRms);
        g_testFeedbackLufs.store((float)(trueInputLufs + chunkGainDb));
    }

    double gainDb = 20.0 * std::log10(measureRms(lastOutput) / inputRms);
    double expectedGainDb = customTargetLufs - trueInputLufs;

    if (std::abs(gainDb - expectedGainDb) > TOLERANCE_DB)
    {
        std::cerr << "[gain=" << gainDb << "dB, expected ~" << expectedGainDb
                   << "dB (converging toward the custom " << customTargetLufs << " LUFS target, not the -23.0f default)]...";
        return false;
    }

    return true;
}

// Once feedback stays below the silence threshold for longer than the pause
// grace period, gain should hold exactly where it is. (Shorter gaps are
// covered by levelerBridgesShortGapsViaGracePeriod.)
bool levelerFreezesGainWhenFeedbackBelowSilenceThreshold()
{
    constexpr int sampleRate = 8000;
    constexpr double TOLERANCE_DB = 0.01;

    g_testFeedbackLufs.store(-25.0f); // below target -- let a little gain drift accumulate
    LevelerStep step(sampleRate, +testFeedbackFn, std::make_shared<DiagnosticCsvLogger>());

    std::unique_ptr<short[]> rawInput(generateOneSecondSineWave(1000.0f, sampleRate));
    std::vector<short> inputVec(rawInput.get(), rawInput.get() + sampleRate);

    runThroughLeveler(step, inputVec, sampleRate / 10);

    // Switch to a reading below the silence gate and run a full second
    // through it first -- generously longer than the ~300ms grace period,
    // so this burn-in absorbs the grace period's own continued drift
    // before the snapshots below, which then only ever see genuinely
    // frozen behavior.
    g_testFeedbackLufs.store(-40.0f);
    runThroughLeveler(step, inputVec, sampleRate / 10);

    auto snapshot1 = runThroughLeveler(step, inputVec, sampleRate / 10);
    auto snapshot2 = runThroughLeveler(step, inputVec, sampleRate / 10);

    double gainDiffDb = 20.0 * std::log10(measureRms(snapshot2) / measureRms(snapshot1));
    if (std::abs(gainDiffDb) > TOLERANCE_DB)
    {
        std::cerr << "[gain drifted by " << gainDiffDb << "dB while feedback was below the silence threshold (well past the grace period), expected ~0]...";
        return false;
    }

    return true;
}

// A gap shorter than the pause grace period should not freeze gain: while a
// sizeable correction is in progress, gain should keep moving toward the
// target established before the gap.
bool levelerBridgesShortGapsViaGracePeriod()
{
    constexpr int sampleRate = 8000;
    constexpr double TOLERANCE_DB = 0.05;

    std::unique_ptr<short[]> rawInput(generateOneSecondSineWave(1000.0f, sampleRate));
    std::vector<short> inputVec(rawInput.get(), rawInput.get() + sampleRate);
    std::vector<short> shortInputVec(rawInput.get(), rawInput.get() + sampleRate / 10); // 100ms

    g_testFeedbackLufs.store(-25.0f); // below target -- a real, sizeable ongoing correction
    LevelerStep step(sampleRate, +testFeedbackFn, std::make_shared<DiagnosticCsvLogger>());
    runThroughLeveler(step, inputVec, sampleRate / 10); // 1s warm-up, short of full convergence

    auto gapStartSnapshot = runThroughLeveler(step, shortInputVec, sampleRate / 10);
    g_testFeedbackLufs.store(-40.0f); // below the -33 threshold -- a gap begins
    auto gapEndSnapshot = runThroughLeveler(step, shortInputVec, sampleRate / 10);

    double gainDiffDb = 20.0 * std::log10(measureRms(gapEndSnapshot) / measureRms(gapStartSnapshot));
    if (std::abs(gainDiffDb) < TOLERANCE_DB)
    {
        std::cerr << "[gain barely moved (" << gainDiffDb << "dB) across a 100ms gap, well under the ~300ms grace "
                   << "period -- expected it to keep smoothing toward the already-established target, not freeze immediately]...";
        return false;
    }

    return true;
}

// During the grace period the target must not be recomputed from the last
// valid reading, which is often a word's quiet tail. Checked by the shape of
// movement within a gap: sampled in four equal slices, movement should
// shrink slice to slice (smoothing toward a fixed target). A target still
// being recomputed and growing would give equal or larger movement.
bool levelerDoesNotKeepChasingATrailingOffSampleDuringGracePeriod()
{
    constexpr int sampleRate = 8000;

    g_testFeedbackLufs.store(-30.0f); // well below target -- a real, sizeable ongoing correction
    LevelerStep step(sampleRate, +testFeedbackFn, std::make_shared<DiagnosticCsvLogger>());

    std::unique_ptr<short[]> rawInput(generateOneSecondSineWave(1000.0f, sampleRate));
    std::vector<short> inputVec(rawInput.get(), rawInput.get() + sampleRate);
    double inputRms = measureRms(inputVec);

    runThroughLeveler(step, inputVec, sampleRate / 10); // 1s warm-up, short of full convergence
    double gainAtGapStart = 20.0 * std::log10(measureRms(runThroughLeveler(step, inputVec, sampleRate / 10)) / inputRms);

    // Go invalid for a 200ms gap (well under the default 300ms grace
    // period), sampled in four 50ms slices to see the shape of movement
    // *within* the gap, not just its start/end.
    g_testFeedbackLufs.store(-40.0f);
    std::vector<short> sliceInputVec(rawInput.get(), rawInput.get() + sampleRate / 20); // 50ms

    double sliceGainsDb[4];
    double prevGain = gainAtGapStart;
    double absDeltas[4];
    for (int i = 0; i < 4; i++)
    {
        auto sliceOutput = runThroughLeveler(step, sliceInputVec, sampleRate / 10);
        sliceGainsDb[i] = 20.0 * std::log10(measureRms(sliceOutput) / inputRms);
        absDeltas[i] = std::abs(sliceGainsDb[i] - prevGain);
        prevGain = sliceGainsDb[i];
    }

    // The last slice's movement should be clearly smaller than the
    // first's -- smoothing toward a target that stopped moving after the
    // gap began, not one still growing away from a repeatedly re-derived
    // stale sample.
    if (absDeltas[3] >= absDeltas[0])
    {
        std::cerr << "[gain movement within the gap didn't shrink over time (first slice " << absDeltas[0]
                   << "dB, last slice " << absDeltas[3] << "dB) -- expected smoothing toward a fixed target, "
                   << "not one still being re-derived and growing away from a stale sample]...";
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
    double gainBeforeDb = 20.0 * std::log10(measureRms(beforeReset) / measureRms(inputVec));

    step.reset();

    int numOutputSamples = 0;
    short* result = step.execute(inputVec.data(), sampleRate / 10, &numOutputSamples);
    std::vector<short> output(result, result + numOutputSamples);

    double gainAfterDb = 20.0 * std::log10(measureRms(output) / measureRms(inputVec));
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
    double firstBlockGainDb = 20.0 * std::log10(measureRms(std::vector<short>(result, result + numOutputSamples)) / inputRms);
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
    double convergedGainDb = 20.0 * std::log10(measureRms(lastOutput) / inputRms);
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
    double firstRealBlockGainDb = 20.0 * std::log10(measureRms(std::vector<short>(result, result + numOutputSamples)) / inputRms);
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
    TEST_CASE(levelerBridgesShortGapsViaGracePeriod);
    TEST_CASE(levelerDoesNotKeepChasingATrailingOffSampleDuringGracePeriod);
    TEST_CASE(levelerThresholdBehavesTheSameBothWaysNow);
    TEST_CASE(levelerResetPreservesGain);
    TEST_CASE(levelerCanBeSeededWithSavedGain);
    TEST_CASE(levelerRampInWaitsForRealAudioNotJustElapsedTime);
    return 0;
}
