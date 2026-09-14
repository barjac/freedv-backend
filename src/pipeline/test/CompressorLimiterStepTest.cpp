#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include "CompressorLimiterStep.h"
#include "PipelineTestCommon.h"

// Sanity/math checks only -- CompressorLimiterStep only behaves correctly
// wired together with LevelerStep in their real feedback loop (see the
// "Replace AgcStep with a Leveler + Compressor/Limiter pair" plan's
// Verification section), so these tests exercise the static gain curve and
// reset behavior against synthetic tones, not real end-to-end behavior.
// Live A/B testing via the diagnostic CSV/plot tooling is the primary
// verification method -- in particular for confirming, on real speech,
// that knee 1's threshold (starting recommendation -6dBFS) doesn't engage
// during ordinary talking (see the RADE-encoder-safety constraint in the
// plan's Compressor/limiter internals section).

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

// A signal well below knee 1's threshold (-6dBFS starting recommendation)
// should pass through with ~0dB gain reduction -- the RADE-encoder-safety
// constraint (Barry, 2026-09-15): ordinary/quiet speech must not be
// measurably compressed.
bool compressorLimiterLeavesQuietSignalUnaffected()
{
    constexpr int sampleRate = 8000;
    constexpr double TOLERANCE_DB = 0.5;

    CompressorLimiterStep step(sampleRate, std::make_shared<DiagnosticCsvLogger>());

    double amplitude = 32767.0 * std::pow(10.0, -20.0 / 20.0); // -20dBFS peak, well below -6dBFS knee 1
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

// A signal at full scale (well above both knees) should be pulled down
// meaningfully once the envelope follower settles -- confirms the limiter
// actually engages rather than just being a pass-through.
bool compressorLimiterReducesGainForLoudSignal()
{
    constexpr int sampleRate = 8000;
    constexpr double MIN_EXPECTED_REDUCTION_DB = 0.5; // conservative lower bound

    CompressorLimiterStep step(sampleRate, std::make_shared<DiagnosticCsvLogger>());

    auto input = generateSineWave(32767.0, 1000.0, 1.0, sampleRate); // 0dBFS peak
    auto output = runThroughStep(step, input, sampleRate / 10);

    // Skip the first 100ms (attack transient) before measuring steady state.
    std::size_t skipSamples = sampleRate / 10;
    double inputPeakDb = measurePeakDbfs(input, skipSamples);
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
    TEST_CASE(compressorLimiterLeavesQuietSignalUnaffected);
    TEST_CASE(compressorLimiterReducesGainForLoudSignal);
    TEST_CASE(compressorLimiterResetClearsGainReductionState);
    return 0;
}
