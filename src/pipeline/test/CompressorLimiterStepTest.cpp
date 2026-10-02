#include <algorithm>
#include <atomic>
#include <cmath>
#include <memory>
#include <vector>

#include "CompressorLimiterStep.h"
#include "LevelerStep.h"
#include "PipelineTestCommon.h"

// Unit checks of the static gain curve, reset behaviour and silence-floor
// selection against synthetic tones. End-to-end behaviour (together with
// LevelerStep on real speech) is verified with the diagnostic CSV logging
// instead.

namespace {

// CompressorLimiterStep expects its input INPUT_HEADROOM_DB below true
// level (see its header). Tests generate signals at a "true" level and
// scale them by this factor before feeding them in.
const double HANDOVER_SCALE = std::pow(10.0, -CompressorLimiterStep::INPUT_HEADROOM_DB / 20.0);

std::vector<short> generateSineWave(double amplitude, double freqHz, double durationSec, int sampleRate)
{
    int numSamples = static_cast<int>(durationSec * sampleRate);
    std::vector<short> result(numSamples);
    for (int n = 0; n < numSamples; n++)
    {
        result[n] = static_cast<short>(amplitude * std::cos(2.0 * M_PI * freqHz * n / sampleRate));
    }
    return result;
}

std::vector<short> runThroughStep(CompressorLimiterStep& step, std::vector<short>& input, int chunkSize)
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

// Peak level in dBFS over the given range of a sample vector.
double measurePeakDbfs(const std::vector<short>& samples, std::size_t startIndex = 0)
{
    double peak = 0.0;
    for (std::size_t i = startIndex; i < samples.size(); i++)
    {
        double absVal = std::abs((double)samples[i]) / 32768.0;
        if (absVal > peak) peak = absVal;
    }
    return peak > 0.0 ? 20.0 * std::log10(peak) : -100.0;
}

} // namespace

// A signal well below the limiter's threshold should pass through with
// ~0dB gain reduction: ordinary speech must not be compressed.
bool compressorLimiterLeavesQuietSignalUnaffected()
{
    constexpr int sampleRate = 8000;
    constexpr double TOLERANCE_DB = 0.5;

    CompressorLimiterStep step(sampleRate, std::make_shared<DiagnosticCsvLogger>());

    double amplitude = 32767.0 * std::pow(10.0, -20.0 / 20.0) * HANDOVER_SCALE; // true -20dBFS peak, well below the limiter's threshold
    auto input = generateSineWave(amplitude, 1000.0, 1.0, sampleRate);
    auto output = runThroughStep(step, input, sampleRate / 10);

    double inputPeakDb = measurePeakDbfs(input) + CompressorLimiterStep::INPUT_HEADROOM_DB;
    double outputPeakDb = measurePeakDbfs(output);

    if (std::abs(outputPeakDb - inputPeakDb) > TOLERANCE_DB)
    {
        std::cerr << "[input=" << inputPeakDb << "dBFS, output=" << outputPeakDb
                   << "dBFS, expected ~0dB difference]...";
        return false;
    }

    return true;
}

// A signal at full scale (well above the limiter's threshold) should be
// pulled down meaningfully once the envelope follower settles -- confirms
// the limiter actually engages rather than just being a pass-through.
bool compressorLimiterReducesGainForLoudSignal()
{
    constexpr int sampleRate = 8000;
    constexpr double MIN_EXPECTED_REDUCTION_DB = 0.5; // conservative lower bound

    CompressorLimiterStep step(sampleRate, std::make_shared<DiagnosticCsvLogger>());

    auto input = generateSineWave(32767.0 * HANDOVER_SCALE, 1000.0, 1.0, sampleRate); // true 0dBFS peak
    auto output = runThroughStep(step, input, sampleRate / 10);

    // Skip the first 100ms (attack transient) before measuring steady state.
    std::size_t skipSamples = sampleRate / 10;
    double inputPeakDb = measurePeakDbfs(input, skipSamples) + CompressorLimiterStep::INPUT_HEADROOM_DB;
    double outputPeakDb = measurePeakDbfs(output, skipSamples);
    double reductionDb = outputPeakDb - inputPeakDb;

    if (reductionDb > -MIN_EXPECTED_REDUCTION_DB)
    {
        std::cerr << "[reduction=" << reductionDb << "dB, expected at least "
                   << MIN_EXPECTED_REDUCTION_DB << "dB of gain reduction on a full-scale tone]...";
        return false;
    }

    return true;
}

// reset() should clear the smoothed gain-reduction state (and look-ahead
// buffer) so a subsequent quiet signal isn't affected by leftover state
// from a prior loud one.
bool compressorLimiterResetClearsGainReductionState()
{
    constexpr int sampleRate = 8000;
    constexpr double TOLERANCE_DB = 0.5;

    CompressorLimiterStep step(sampleRate, std::make_shared<DiagnosticCsvLogger>());

    // Drive the limiter hard first.
    auto loudInput = generateSineWave(32767.0 * HANDOVER_SCALE, 1000.0, 1.0, sampleRate);
    runThroughStep(step, loudInput, sampleRate / 10);

    step.reset();

    // A quiet signal right after reset() should show ~0dB reduction, not
    // still be affected by the just-cleared gain-reduction state.
    double amplitude = 32767.0 * std::pow(10.0, -20.0 / 20.0) * HANDOVER_SCALE;
    auto quietInput = generateSineWave(amplitude, 1000.0, 1.0, sampleRate);
    auto output = runThroughStep(step, quietInput, sampleRate / 10);

    double inputPeakDb = measurePeakDbfs(quietInput) + CompressorLimiterStep::INPUT_HEADROOM_DB;
    double outputPeakDb = measurePeakDbfs(output);

    if (std::abs(outputPeakDb - inputPeakDb) > TOLERANCE_DB)
    {
        std::cerr << "[input=" << inputPeakDb << "dBFS, output=" << outputPeakDb
                   << "dBFS right after reset(), expected ~0dB difference]...";
        return false;
    }

    return true;
}

// The same quiet-but-real signal should be rejected by the RNNoise-on
// silence floor (-70 LUFS) but accepted by the lower RNNoise-off floor
// (-85 LUFS). Without RNNoise, gaps between words can measure quieter than
// RNNoise's residual noise floor, so the higher floor would reject real
// speech.
bool compressorLimiterUsesLooserSilenceFloorWithNoiseReductionOff()
{
    constexpr int sampleRate = 8000;

    // A 1kHz full-scale sine's EBU R128 loudness is ~-3.01 LUFS (RMS is
    // 3dB below peak; K-weighting is close to unity at 1kHz), so scaling
    // amplitude by 10^((targetLufs - (-3.01))/20) gives roughly that
    // target LUFS. -78 LUFS sits roughly in the middle of the two floors
    // (-70 RNNoise-on, -85 RNNoise-off) with a comfortable margin either
    // side, so this doesn't depend on that approximation being exact.
    constexpr double approxTargetLufs = -78.0;
    double amplitude = 32767.0 * std::pow(10.0, (approxTargetLufs - (-3.01)) / 20.0) * HANDOVER_SCALE;
    auto input = generateSineWave(amplitude, 1000.0, 2.0, sampleRate); // long enough for EBU R128's 400ms window to fill

    // RNNoise reported ON -- the original, stricter -70 floor should
    // reject this quiet signal (getLastOutputLoudnessLufs() stays at its
    // -100.0f "invalid" default).
    {
        CompressorLimiterStep stepOn(sampleRate, std::make_shared<DiagnosticCsvLogger>());
        runThroughStep(stepOn, input, sampleRate / 10);
        float lufsOn = CompressorLimiterStep::getLastOutputLoudnessLufs();
        if (lufsOn > -99.0f)
        {
            std::cerr << "[with RNNoise reported ON, getLastOutputLoudnessLufs()=" << lufsOn
                       << " -- expected it rejected (~-100) by the stricter -70 floor]...";
            return false;
        }
    }

    // RNNoise reported OFF -- the looser -85 floor should accept the same
    // signal as a genuine (if quiet) reading.
    {
        CompressorLimiterStep stepOff(sampleRate, std::make_shared<DiagnosticCsvLogger>(), +[]() FREEDV_NONBLOCKING { return false; });
        runThroughStep(stepOff, input, sampleRate / 10);
        float lufsOff = CompressorLimiterStep::getLastOutputLoudnessLufs();
        if (lufsOff < -85.0f || lufsOff > -50.0f)
        {
            std::cerr << "[with RNNoise reported OFF, getLastOutputLoudnessLufs()=" << lufsOff
                       << " -- expected a genuine reading around " << approxTargetLufs << " LUFS, accepted by the looser -85 floor]...";
            return false;
        }
    }

    return true;
}

namespace {

std::atomic<float> g_chainTestFeedbackLufs{-23.0f};

float chainTestFeedbackFn() FREEDV_NONBLOCKING
{
    return g_chainTestFeedbackLufs.load(std::memory_order_relaxed);
}

// Crest factor (peak/RMS) in dB. A pure sine is ~3.01dB; a clipped one is
// noticeably lower.
double measureCrestFactorDb(const std::vector<short>& samples, std::size_t startIndex)
{
    double peak = 0.0;
    double sumSq = 0.0;
    for (std::size_t i = startIndex; i < samples.size(); i++)
    {
        double v = std::abs((double)samples[i]);
        if (v > peak) peak = v;
        sumSq += v * v;
    }
    double rms = std::sqrt(sumSq / (samples.size() - startIndex));
    return 20.0 * std::log10(peak / rms);
}

} // namespace

// Leveler gain that pushes peaks above full scale must not clip in the
// handover to the limiter: the limiter has to see the true peak and limit
// it smoothly. A -2dBFS sine with +6dB of leveler gain (+4dBFS true) should
// come out as an undistorted sine at the limiter's ceiling. If the peaks
// were clipped before the limiter, the output would still sit near the
// ceiling but with a much lower crest factor.
bool levelerGainAboveFullScaleIsLimitedNotClipped()
{
    constexpr int sampleRate = 8000;
    constexpr float seededGainDb = 6.0f;
    constexpr float seededIntegralErrorDb = 24.0f; // holds target at seededGainDb with feedback at target (6.0 * 4.0)
    constexpr double SINE_CREST_DB = 3.01;
    constexpr double CREST_TOLERANCE_DB = 0.5;

    g_chainTestFeedbackLufs.store(-23.0f);
    auto diagLogger = std::make_shared<DiagnosticCsvLogger>();
    LevelerStep leveler(sampleRate, +chainTestFeedbackFn, diagLogger, seededGainDb, seededIntegralErrorDb);
    CompressorLimiterStep limiter(sampleRate, diagLogger);

    auto input = generateSineWave(32767.0 * std::pow(10.0, -2.0 / 20.0), 1000.0, 2.0, sampleRate);
    std::vector<short> output;
    int chunkSize = sampleRate / 10;
    for (std::size_t offset = 0; offset < input.size(); offset += chunkSize)
    {
        int n = static_cast<int>(std::min<std::size_t>(chunkSize, input.size() - offset));
        int levelerOut = 0;
        short* levelerResult = leveler.execute(&input[offset], n, &levelerOut);
        int limiterOut = 0;
        short* limiterResult = limiter.execute(levelerResult, levelerOut, &limiterOut);
        output.insert(output.end(), limiterResult, limiterResult + limiterOut);
    }

    // Skip the first second (startup ramp-in and limiter attack).
    std::size_t skipSamples = sampleRate;
    double outputPeakDb = measurePeakDbfs(output, skipSamples);
    double crestDb = measureCrestFactorDb(output, skipSamples);

    if (outputPeakDb > -1.0 || outputPeakDb < -3.0 || std::abs(crestDb - SINE_CREST_DB) > CREST_TOLERANCE_DB)
    {
        std::cerr << "[output peak=" << outputPeakDb << "dBFS, crest factor=" << crestDb
                   << "dB -- expected ~-1.2dBFS with a sine's ~3.0dB crest factor (lower crest means clipping before the limiter)]...";
        return false;
    }

    return true;
}

// A tone that starts abruptly above full scale must be limited from its
// very first cycle: the output peak must not exceed the knee curve's
// output for that level (-1.5dBFS + overshoot/20), and no samples may hit
// the int16 saturation backstop. Covers low/high voice frequencies, sample
// rates, and levels up to the leveler's +12dB maximum gain on a full-scale
// input.
bool compressorLimiterCatchesSuddenOnsets()
{
    constexpr double THRESHOLD_DB = -1.5;
    constexpr double RATIO = 20.0;
    constexpr double TOLERANCE_DB = 0.1;

    for (int sampleRate : {8000, 16000, 48000})
    for (double freqHz : {150.0, 300.0, 1000.0})
    for (double truePeakDb : {2.1, 6.0, 12.0})
    {
        CompressorLimiterStep step(sampleRate, std::make_shared<DiagnosticCsvLogger>());

        // 200ms of silence, then the tone starting at a positive peak.
        std::vector<short> input(sampleRate / 5, 0);
        double amplitude = 32767.0 * std::pow(10.0, truePeakDb / 20.0) * HANDOVER_SCALE;
        for (int n = 0; n < sampleRate / 5; n++)
        {
            input.push_back(static_cast<short>(amplitude * std::cos(2.0 * M_PI * freqHz * n / sampleRate)));
        }
        auto output = runThroughStep(step, input, sampleRate / 100);

        int saturated = 0;
        for (short v : output)
        {
            if (v >= 32767 || v <= -32767) saturated++;
        }
        double outputPeakDb = measurePeakDbfs(output);
        double expectedMaxDb = THRESHOLD_DB + (truePeakDb - THRESHOLD_DB) / RATIO;

        if (saturated > 0 || outputPeakDb > expectedMaxDb + TOLERANCE_DB)
        {
            std::cerr << "[" << freqHz << "Hz at " << sampleRate << "Hz, true peak +" << truePeakDb
                       << "dBFS: output peak " << outputPeakDb << "dBFS (limit " << expectedMaxDb
                       << "), " << saturated << " saturated samples]...";
            return false;
        }
    }

    return true;
}

namespace {

bool disabledFn() FREEDV_NONBLOCKING
{
    return false;
}

// Runs input through a leveler -> limiter chain in 10ms chunks.
std::vector<short> runThroughChain(LevelerStep& leveler, CompressorLimiterStep& limiter, std::vector<short>& input, int sampleRate)
{
    std::vector<short> output;
    int chunkSize = sampleRate / 100;
    for (std::size_t offset = 0; offset < input.size(); offset += chunkSize)
    {
        int n = static_cast<int>(std::min<std::size_t>(chunkSize, input.size() - offset));
        int levelerOut = 0;
        short* levelerResult = leveler.execute(&input[offset], n, &levelerOut);
        int limiterOut = 0;
        short* limiterResult = limiter.execute(levelerResult, levelerOut, &limiterOut);
        output.insert(output.end(), limiterResult, limiterResult + limiterOut);
    }
    return output;
}

} // namespace

// With levelling disabled, a quiet signal should come out of the chain at
// unity gain (the handover headroom still cancels), and the leveler's saved
// gain must be left untouched for when it's re-enabled.
bool levelerDisabledPassesAudioAtUnityAndKeepsState()
{
    constexpr int sampleRate = 8000;
    constexpr float seededGainDb = 6.0f;
    constexpr double TOLERANCE_DB = 0.2;

    g_chainTestFeedbackLufs.store(-40.0f); // would move gain if the leveler were active
    auto diagLogger = std::make_shared<DiagnosticCsvLogger>();
    LevelerStep leveler(sampleRate, +chainTestFeedbackFn, diagLogger, seededGainDb, 24.0f, -23.0f,
                        +[]() FREEDV_NONBLOCKING { return true; }, +disabledFn);
    CompressorLimiterStep limiter(sampleRate, diagLogger);

    auto input = generateSineWave(32767.0 * std::pow(10.0, -20.0 / 20.0), 1000.0, 1.0, sampleRate);
    auto output = runThroughChain(leveler, limiter, input, sampleRate);

    double gainDb = measurePeakDbfs(output, sampleRate / 10) - measurePeakDbfs(input, sampleRate / 10);
    if (std::abs(gainDb) > TOLERANCE_DB || leveler.getCurrentGainDb() != seededGainDb)
    {
        std::cerr << "[chain gain " << gainDb << "dB (expected ~0), leveler gain state " << leveler.getCurrentGainDb()
                   << "dB (expected unchanged " << seededGainDb << ")]...";
        return false;
    }

    return true;
}

// The limiter must stay effective with levelling disabled: a sudden
// full-scale onset must stay within the knee curve with no saturated
// samples.
bool limiterStaysActiveWithLevelerDisabled()
{
    constexpr int sampleRate = 48000;
    constexpr double EXPECTED_MAX_DB = -1.5 + (0.0 - -1.5) / 20.0;
    constexpr double TOLERANCE_DB = 0.1;

    auto diagLogger = std::make_shared<DiagnosticCsvLogger>();
    LevelerStep leveler(sampleRate, +chainTestFeedbackFn, diagLogger, 0.0f, 0.0f, -23.0f,
                        +[]() FREEDV_NONBLOCKING { return true; }, +disabledFn);
    CompressorLimiterStep limiter(sampleRate, diagLogger);

    std::vector<short> input(sampleRate / 5, 0);
    for (int n = 0; n < sampleRate / 5; n++)
    {
        input.push_back(static_cast<short>(32767.0 * std::cos(2.0 * M_PI * 300.0 * n / sampleRate)));
    }
    auto output = runThroughChain(leveler, limiter, input, sampleRate);

    int saturated = 0;
    for (short v : output)
    {
        if (v >= 32767 || v <= -32767) saturated++;
    }
    double outputPeakDb = measurePeakDbfs(output);
    if (saturated > 0 || outputPeakDb > EXPECTED_MAX_DB + TOLERANCE_DB)
    {
        std::cerr << "[output peak " << outputPeakDb << "dBFS (limit " << EXPECTED_MAX_DB << "), "
                   << saturated << " saturated samples, with the leveler disabled]...";
        return false;
    }

    return true;
}

int main()
{
    TEST_CASE(compressorLimiterLeavesQuietSignalUnaffected);
    TEST_CASE(compressorLimiterReducesGainForLoudSignal);
    TEST_CASE(compressorLimiterResetClearsGainReductionState);
    TEST_CASE(compressorLimiterUsesLooserSilenceFloorWithNoiseReductionOff);
    TEST_CASE(levelerGainAboveFullScaleIsLimitedNotClipped);
    TEST_CASE(compressorLimiterCatchesSuddenOnsets);
    TEST_CASE(levelerDisabledPassesAudioAtUnityAndKeepsState);
    TEST_CASE(limiterStaysActiveWithLevelerDisabled);
    return 0;
}
