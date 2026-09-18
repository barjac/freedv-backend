#include <algorithm>
#include <atomic>
#include <cmath>
#include <memory>
#include <vector>

#include "LevelerStep.h"
#include "PipelineTestCommon.h"

// Sanity/math checks only -- LevelerStep only behaves correctly wired
// together with CompressorLimiterStep in their real feedback loop (see the
// "Replace AgcStep with a Leveler + Compressor/Limiter pair" plan's
// Verification section), so these tests exercise the gain-convergence
// formula and silence-gating logic against a synthetic feedback value, not
// real end-to-end behavior. Live A/B testing via the diagnostic CSV/plot
// tooling is the primary verification method.

namespace {

// realtime_fp<float()> can only hold a plain captureless function pointer
// (see LevelAdjustStep's own usage precedent), so the synthetic feedback
// value is file-scope state the test sets before each run.
std::atomic<float> g_testFeedbackLufs{-23.0f};

float testFeedbackFn() FREEDV_NONBLOCKING
{
    return g_testFeedbackLufs.load(std::memory_order_relaxed);
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

// Simulates the real closed loop: feedback for the next chunk is derived
// from a fixed true input level plus whatever gain was actually just
// applied, rather than a constant value that never reacts to gain (a
// non-reactive constant can't meaningfully exercise targetGainDb_'s
// formula -- see LevelerStep.cpp's 2026-09-18 fix -- since that formula
// specifically relies on feedback responding to applied gain in order to
// converge to the full correction instead of getting stuck at half of it).
// With this properly closed loop, gain should converge toward the full
// correction -23 - trueInputLufs, following the proportional/time-constant
// formula (not a fixed dB/sec ramp).
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

// Once the feedback reading drops below SILENCE_THRESHOLD_LUFS, gain should
// freeze exactly where it was rather than continuing to update -- mirrors
// EBU R128's own gating behavior during silence.
bool levelerFreezesGainWhenFeedbackBelowSilenceThreshold()
{
    constexpr int sampleRate = 8000;
    constexpr double TOLERANCE_DB = 0.01;

    g_testFeedbackLufs.store(-25.0f); // below target -- let a little gain drift accumulate
    LevelerStep step(sampleRate, +testFeedbackFn, std::make_shared<DiagnosticCsvLogger>());

    std::unique_ptr<short[]> rawInput(generateOneSecondSineWave(1000.0f, sampleRate));
    std::vector<short> inputVec(rawInput.get(), rawInput.get() + sampleRate);

    runThroughLeveler(step, inputVec, sampleRate / 10);

    // Now switch to a reading below the silence gate -- gain should freeze from here.
    g_testFeedbackLufs.store(-40.0f);
    auto snapshot1 = runThroughLeveler(step, inputVec, sampleRate / 10);
    auto snapshot2 = runThroughLeveler(step, inputVec, sampleRate / 10);

    double gainDiffDb = 20.0 * std::log10(measureRms(snapshot2) / measureRms(snapshot1));
    if (std::abs(gainDiffDb) > TOLERANCE_DB)
    {
        std::cerr << "[gain drifted by " << gainDiffDb << "dB while feedback was below the silence threshold, expected ~0]...";
        return false;
    }

    return true;
}

// Reversed 2026-09-18 (Barry): reset() is called on every individual TX
// entry within a session (its only call site in either repo is "just
// entered TX from RX"), not just once at Start -- so zeroing gain here
// meant every single PTT press re-ran the leveler's full climb from 0dB.
// reset() now leaves gain untouched (persists across transmissions);
// gain still starts at 0dB per-session via the constructor.
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

int main()
{
    TEST_CASE(levelerConvergesTowardExpectedGainForQuietFeedback);
    TEST_CASE(levelerFreezesGainWhenFeedbackBelowSilenceThreshold);
    TEST_CASE(levelerResetPreservesGain);
    return 0;
}
