#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include "PostLoopCompressorStep.h"
#include "PipelineTestCommon.h"

// Sanity/math checks only -- mirrors CompressorLimiterStepTest.cpp's own
// approach and rationale. This class is deliberately standalone (no shared
// state with CompressorLimiterStep/LevelerStep at all -- see
// PostLoopCompressorStep.h's own comment), so there's no feedback loop to
// test here, unlike that pair.

namespace {

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

std::vector<short> runThroughStep(PostLoopCompressorStep& step, std::vector<short>& input, int chunkSize)
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

// A signal well below knee A's threshold should pass through with ~0dB
// gain reduction -- same RADE-encoder-safety constraint as
// CompressorLimiterStep: ordinary/quiet speech must not be measurably
// compressed.
bool postLoopCompressorLeavesQuietSignalUnaffected()
{
    constexpr int sampleRate = 8000;
    constexpr double TOLERANCE_DB = 0.5;

    PostLoopCompressorStep step(sampleRate);

    double amplitude = 32767.0 * std::pow(10.0, -20.0 / 20.0); // -20dBFS peak, well below knee A's -6dBFS
    auto input = generateSineWave(amplitude, 1000.0, 1.0, sampleRate);
    auto output = runThroughStep(step, input, sampleRate / 10);

    double inputPeakDb = measurePeakDbfs(input);
    double outputPeakDb = measurePeakDbfs(output);

    if (std::abs(outputPeakDb - inputPeakDb) > TOLERANCE_DB)
    {
        std::cerr << "[input=" << inputPeakDb << "dBFS, output=" << outputPeakDb
                   << "dBFS, expected ~0dB difference]...";
        return false;
    }

    return true;
}

// A signal between the two knees (-3dBFS: above knee A's -6dBFS, below
// knee B's -1.5dBFS) should show moderate reduction from knee A's 2:1
// ratio alone -- confirms knee A is actually engaging on its own, not
// just riding along with knee B.
bool postLoopCompressorKneeAReducesModerateSignal()
{
    constexpr int sampleRate = 8000;
    constexpr double MIN_EXPECTED_REDUCTION_DB = 0.5;
    constexpr double MAX_EXPECTED_REDUCTION_DB = 3.0; // 2:1 on a few dB of overshoot -- shouldn't be dramatic

    PostLoopCompressorStep step(sampleRate);

    double amplitude = 32767.0 * std::pow(10.0, -3.0 / 20.0); // -3dBFS peak
    auto input = generateSineWave(amplitude, 1000.0, 1.0, sampleRate);
    auto output = runThroughStep(step, input, sampleRate / 10);

    std::size_t skipSamples = sampleRate / 10; // skip attack transient
    double inputPeakDb = measurePeakDbfs(input, skipSamples);
    double outputPeakDb = measurePeakDbfs(output, skipSamples);
    double reductionDb = outputPeakDb - inputPeakDb;

    if (reductionDb > -MIN_EXPECTED_REDUCTION_DB || reductionDb < -MAX_EXPECTED_REDUCTION_DB)
    {
        std::cerr << "[reduction=" << reductionDb << "dB at -3dBFS, expected a moderate "
                   << MIN_EXPECTED_REDUCTION_DB << "-" << MAX_EXPECTED_REDUCTION_DB << "dB from knee A's 2:1 ratio]...";
        return false;
    }

    return true;
}

// A full-scale signal (well above both knees) should be pulled down much
// more heavily than the between-knees case above -- confirms knee B's
// near-infinite ratio is doing real additional work in series with knee A,
// not just duplicating it.
bool postLoopCompressorKneeBReducesLoudSignalHeavily()
{
    constexpr int sampleRate = 8000;
    constexpr double MIN_EXPECTED_REDUCTION_DB = 4.0; // clearly more than knee-A-alone's ~3dB ceiling

    PostLoopCompressorStep step(sampleRate);

    auto input = generateSineWave(32767.0, 1000.0, 1.0, sampleRate); // 0dBFS peak
    auto output = runThroughStep(step, input, sampleRate / 10);

    std::size_t skipSamples = sampleRate / 10;
    double inputPeakDb = measurePeakDbfs(input, skipSamples);
    double outputPeakDb = measurePeakDbfs(output, skipSamples);
    double reductionDb = outputPeakDb - inputPeakDb;

    if (reductionDb > -MIN_EXPECTED_REDUCTION_DB)
    {
        std::cerr << "[reduction=" << reductionDb << "dB on a full-scale tone, expected at least "
                   << MIN_EXPECTED_REDUCTION_DB << "dB once both knees are engaging]...";
        return false;
    }

    return true;
}

// Explicit check for Barry's stated requirement (2026-09-21): "make sure
// it adds no make-up gain." Across a range of levels from very quiet to
// full-scale, output peak must never exceed input peak by any measurable
// amount -- this stage must only ever attenuate, never boost.
bool postLoopCompressorNeverAddsGain()
{
    constexpr int sampleRate = 8000;
    constexpr double TOLERANCE_DB = 0.1; // allow for float rounding only

    double testLevelsDbfs[] = { -40.0, -20.0, -10.0, -6.0, -3.0, -1.5, 0.0 };
    for (double levelDbfs : testLevelsDbfs)
    {
        PostLoopCompressorStep step(sampleRate); // fresh instance per level -- no envelope carryover between cases
        double amplitude = 32767.0 * std::pow(10.0, levelDbfs / 20.0);
        auto input = generateSineWave(amplitude, 1000.0, 1.0, sampleRate);
        auto output = runThroughStep(step, input, sampleRate / 10);

        double inputPeakDb = measurePeakDbfs(input);
        double outputPeakDb = measurePeakDbfs(output);

        if (outputPeakDb > inputPeakDb + TOLERANCE_DB)
        {
            std::cerr << "[at " << levelDbfs << "dBFS input, output peak (" << outputPeakDb
                       << "dBFS) exceeded input peak (" << inputPeakDb << "dBFS) -- this stage must never add gain]...";
            return false;
        }
    }

    return true;
}

// reset() should clear the smoothed gain-reduction state (and look-ahead
// buffer) so a subsequent quiet signal isn't affected by leftover state
// from a prior loud one.
bool postLoopCompressorResetClearsGainReductionState()
{
    constexpr int sampleRate = 8000;
    constexpr double TOLERANCE_DB = 0.5;

    PostLoopCompressorStep step(sampleRate);

    // Drive it hard first.
    auto loudInput = generateSineWave(32767.0, 1000.0, 1.0, sampleRate);
    runThroughStep(step, loudInput, sampleRate / 10);

    step.reset();

    // A quiet signal right after reset() should show ~0dB reduction, not
    // still be affected by the just-cleared gain-reduction state.
    double amplitude = 32767.0 * std::pow(10.0, -20.0 / 20.0);
    auto quietInput = generateSineWave(amplitude, 1000.0, 1.0, sampleRate);
    auto output = runThroughStep(step, quietInput, sampleRate / 10);

    double inputPeakDb = measurePeakDbfs(quietInput);
    double outputPeakDb = measurePeakDbfs(output);

    if (std::abs(outputPeakDb - inputPeakDb) > TOLERANCE_DB)
    {
        std::cerr << "[input=" << inputPeakDb << "dBFS, output=" << outputPeakDb
                   << "dBFS right after reset(), expected ~0dB difference]...";
        return false;
    }

    return true;
}

int main()
{
    TEST_CASE(postLoopCompressorLeavesQuietSignalUnaffected);
    TEST_CASE(postLoopCompressorKneeAReducesModerateSignal);
    TEST_CASE(postLoopCompressorKneeBReducesLoudSignalHeavily);
    TEST_CASE(postLoopCompressorNeverAddsGain);
    TEST_CASE(postLoopCompressorResetClearsGainReductionState);
    return 0;
}
