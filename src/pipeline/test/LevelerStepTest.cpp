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
// non-reactive constant can't meaningfully exercise targetGainDb_'s PI
// formula -- see LevelerStep.cpp's 2026-09-18 comments -- since it
// specifically relies on feedback responding to applied gain in order to
// converge to the full correction). For a genuinely constant input like
// this test's, the PI controller's proportional and integral terms both
// eventually converge, and true equilibrium requires feedbackLufs to reach
// exactly -23 LUFS regardless of either term's own gain -- so gain should
// converge toward the full correction -23 - trueInputLufs.
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

// 2026-09-20: LevelerStep can be seeded with a saved gain/integral-error
// pair (e.g. restored from a config file at the start of a new session,
// see the constructor's own comment) instead of always cold-starting at
// 0dB -- verify the getters report the seeded state back correctly, and
// that it's what execute() actually converges to once the startup ramp-in
// (see STARTUP_RAMP_SEC in LevelerStep.cpp) has finished.
bool levelerCanBeSeededWithSavedGain()
{
    constexpr int sampleRate = 8000;
    constexpr float seededGainDb = 7.5f;
    // Chosen to be self-consistent with seededGainDb under feedback held
    // exactly at target (instantErrorDb == 0, so targetGainDb_ ==
    // integralErrorDb_/LEVELER_INTEGRAL_TIME_CONSTANT_SEC): 7.5 * 4.0.
    // LEVELER_INTEGRAL_TIME_CONSTANT_SEC is file-scope in LevelerStep.cpp
    // (not exposed via the header), so like LEVELER_TARGET_LUFS elsewhere
    // in this file, this constant must be kept in sync by hand if it
    // changes -- otherwise this test would see genuine (correct) gain
    // drift toward a mismatched target and start failing for a reason
    // unrelated to what it's actually checking.
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

    std::unique_ptr<short[]> rawInput(generateOneSecondSineWave(1000.0f, sampleRate));
    std::vector<short> inputVec(rawInput.get(), rawInput.get() + sampleRate);
    double inputRms = measureRms(inputVec);

    // This test's sine wave is real (non-silent) audio from sample one, so
    // the ramp-in starts counting immediately, same as if this were the
    // very first real speech of a session. First block (100ms) falls
    // entirely within the 300ms ramp-in window -- actual applied gain
    // should be well below the seeded value here. This is the specific
    // real-world failure this ramp fixes: a persisted gain applied in
    // full from sample one, stacking with CompressorLimiterStep's own
    // gain-reduction envelope also starting cold, was confirmed (via a
    // real capture) to push an otherwise-safe first syllable to true
    // 0dBFS.
    int numOutputSamples = 0;
    short* result = step.execute(inputVec.data(), sampleRate / 10, &numOutputSamples);
    double firstBlockGainDb = 20.0 * std::log10(measureRms(std::vector<short>(result, result + numOutputSamples)) / inputRms);
    if (firstBlockGainDb > seededGainDb - 3.0)
    {
        std::cerr << "[first (ramping-in) block's gain was " << firstBlockGainDb << "dB, expected it well below the seeded "
                   << seededGainDb << "dB -- ramp-in doesn't seem to be reducing applied gain at startup]...";
        return false;
    }

    // Run well past the 300ms ramp window (five more 100ms blocks) -- gain
    // should have converged back to (approximately) the seeded value,
    // since feedback has been held exactly at target throughout.
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

// Regression test, 2026-09-20: a first version of the ramp-in above keyed
// it to wall-clock time since construction rather than to real audio
// actually arriving. Barry caught it from the diagnostic graph: "that is
// happening immediately after TX starts, long before the first syllable"
// -- in real use there's always some delay between the leveler being
// constructed (pressing Start) and a real transmission actually beginning
// (pressing PTT and speaking), so a construction-time ramp had always
// already finished by the time real audio arrived, protecting nothing.
// This simulates exactly that gap: several seconds of silence (as if idle
// in RX) before any real signal, then checks the first block of *real*
// audio is still ramping in, not already at full seeded gain.
bool levelerRampInWaitsForRealAudioNotJustElapsedTime()
{
    constexpr int sampleRate = 8000;
    constexpr float seededGainDb = 7.5f;
    constexpr float seededIntegralErrorDb = 30.0f; // see levelerCanBeSeededWithSavedGain's own comment

    g_testFeedbackLufs.store(-23.0f);
    LevelerStep step(sampleRate, +testFeedbackFn, std::make_shared<DiagnosticCsvLogger>(), seededGainDb, seededIntegralErrorDb);

    // Several seconds of digital silence, well past the 300ms ramp window
    // if it were (wrongly) keyed to elapsed time alone -- simulates idle
    // RX time between Start and the operator actually keying PTT.
    std::vector<short> silence(sampleRate / 10, 0);
    for (int block = 0; block < 30; block++) // 3 seconds
    {
        int numOutputSamples = 0;
        step.execute(silence.data(), (int)silence.size(), &numOutputSamples);
    }

    // Now real audio arrives for the first time -- this is the moment
    // that actually needs protecting, regardless of how much idle time
    // came before it.
    std::unique_ptr<short[]> rawInput(generateOneSecondSineWave(1000.0f, sampleRate));
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
    TEST_CASE(levelerFreezesGainWhenFeedbackBelowSilenceThreshold);
    TEST_CASE(levelerResetPreservesGain);
    TEST_CASE(levelerCanBeSeededWithSavedGain);
    TEST_CASE(levelerRampInWaitsForRealAudioNotJustElapsedTime);
    return 0;
}
