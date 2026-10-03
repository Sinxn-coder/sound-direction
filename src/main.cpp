// Sound direction localization MVP - laptop driver.
//
// Loads a WAV file (or generates a test signal), optionally simulates a two-mic
// recording with a chosen delay / level difference, then runs the direction
// engine block by block and on the whole file.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <string>
#include <vector>

#include "audio/BufferStereoSource.h"
#include "audio/WavFile.h"
#include "direction/DirectionEngine.h"
#include "direction/TemporalSmoother.h"
#include "simulation/BinauralSimulator.h"
#include "simulation/SignalGenerator.h"
#include "utils/ConsoleViz.h"

using namespace sdl;

namespace {

struct Options {
    std::string input;           // empty -> built-in noise bursts
    bool stereo = false;         // input already contains real L/R mic channels
    std::string side = "center"; // left | right | center
    double delayMs = -1.0;       // < 0 -> not given
    double delaySamples = -1.0;
    float levelDb = 0.0f;        // far-ear attenuation
    bool useAzimuth = false;
    double azimuthDeg = 0.0;
    float snrDb = -1.0f;         // < 0 -> no noise
    double spacing = 0.18;
    int blockSize = 2048;
    std::string writeSim;
    std::string generateTest;
    double seconds = 3.0;
    int rate = 48000;
    bool quiet = false;
};

void printUsage() {
    std::puts(
        "Usage:\n"
        "  sound_direction [--input FILE.wav] [simulation options] [engine options]\n"
        "  sound_direction --input STEREO.wav --stereo        (real L/R recording)\n"
        "  sound_direction --generate-test OUT.wav [--seconds S] [--rate HZ]\n"
        "\n"
        "Simulation (mono input -> two virtual mics):\n"
        "  --side left|right|center   which side the sound comes from (default center)\n"
        "  --delay-ms X               inter-mic delay in milliseconds (default 0.30 if a side is set)\n"
        "  --delay-samples X          inter-mic delay in samples (fractional ok); overrides --delay-ms\n"
        "  --level-db X               far mic is X dB quieter (default 0)\n"
        "  --azimuth DEG              -90 (left) .. +90 (right); sets delay+level from a head model\n"
        "  --snr DB                   add independent noise to each mic\n"
        "  --write-sim OUT.wav        save the simulated stereo signal\n"
        "\n"
        "Engine:\n"
        "  --spacing M                mic spacing in meters (default 0.18)\n"
        "  --block N                  analysis block size in samples (default 2048)\n"
        "  --quiet                    only print the overall result\n"
        "\n"
        "If --input is omitted, a built-in noise-burst test signal is used.");
}

bool parseArgs(int argc, char** argv, Options& o) {
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&](const char* name) -> const char* {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "error: %s needs a value\n", name);
                std::exit(2);
            }
            return argv[++i];
        };
        if (a == "--help" || a == "-h") { printUsage(); std::exit(0); }
        else if (a == "--input") o.input = next("--input");
        else if (a == "--stereo") o.stereo = true;
        else if (a == "--side") o.side = next("--side");
        else if (a == "--delay-ms") o.delayMs = std::atof(next("--delay-ms"));
        else if (a == "--delay-samples") o.delaySamples = std::atof(next("--delay-samples"));
        else if (a == "--level-db") o.levelDb = static_cast<float>(std::atof(next("--level-db")));
        else if (a == "--azimuth") { o.useAzimuth = true; o.azimuthDeg = std::atof(next("--azimuth")); }
        else if (a == "--snr") o.snrDb = static_cast<float>(std::atof(next("--snr")));
        else if (a == "--spacing") o.spacing = std::atof(next("--spacing"));
        else if (a == "--block") o.blockSize = std::atoi(next("--block"));
        else if (a == "--write-sim") o.writeSim = next("--write-sim");
        else if (a == "--generate-test") o.generateTest = next("--generate-test");
        else if (a == "--seconds") o.seconds = std::atof(next("--seconds"));
        else if (a == "--rate") o.rate = std::atoi(next("--rate"));
        else if (a == "--quiet") o.quiet = true;
        else {
            std::fprintf(stderr, "error: unknown option '%s' (try --help)\n", a.c_str());
            return false;
        }
    }
    if (o.side != "left" && o.side != "right" && o.side != "center") {
        std::fprintf(stderr, "error: --side must be left, right or center\n");
        return false;
    }
    if (o.blockSize < 256) {
        std::fprintf(stderr, "error: --block must be >= 256\n");
        return false;
    }
    if (o.spacing <= 0.0) {
        std::fprintf(stderr, "error: --spacing must be > 0\n");
        return false;
    }
    return true;
}

void printResult(const char* label, const DirectionResult& r) {
    std::printf("%s\n", label);
    std::printf("  direction   : %s\n", toString(r.direction));
    std::printf("  delay       : %+.2f samples  (%+.4f ms)\n", r.delaySamples, r.delayMs);
    std::printf("  azimuth     : %+.1f deg (free-field estimate)\n", r.azimuthDeg);
    std::printf("  confidence  : %.2f\n", r.confidence);
    std::printf("  peak NCC    : %.3f%s\n", r.peakCorrelation, r.peakAtSearchEdge ? "  (at search edge!)" : "");
    std::printf("  level       : %.1f dBFS, right-left %+.1f dB\n", r.levelDbfs, r.levelDiffDb);
    std::printf("  search range: +-%d samples\n", r.maxLagSamples);
}

} // namespace

int main(int argc, char** argv) {
    Options opt;
    if (!parseArgs(argc, argv, opt)) return 2;
    std::string err;

    // ---- Test-signal generation mode --------------------------------------
    if (!opt.generateTest.empty()) {
        const auto sig = generateNoiseBursts(opt.rate, opt.seconds);
        if (!writeWav(opt.generateTest, {sig}, opt.rate, err)) {
            std::fprintf(stderr, "error: %s\n", err.c_str());
            return 1;
        }
        std::printf("Wrote %.1f s mono test signal @ %d Hz to %s\n", opt.seconds, opt.rate,
                    opt.generateTest.c_str());
        return 0;
    }

    // ---- Load audio -------------------------------------------------------
    std::vector<float> left, right;
    int sampleRate = opt.rate;
    bool simulated = false;
    SimulationParams sim;

    if (opt.stereo) {
        if (opt.input.empty()) {
            std::fprintf(stderr, "error: --stereo requires --input\n");
            return 2;
        }
        WavData wav;
        if (!readWav(opt.input, wav, err)) {
            std::fprintf(stderr, "error: %s\n", err.c_str());
            return 1;
        }
        if (wav.channels < 2) {
            std::fprintf(stderr, "error: --stereo needs a WAV with >= 2 channels (got %d)\n", wav.channels);
            return 1;
        }
        sampleRate = wav.sampleRate;
        left = std::move(wav.channelData[0]);
        right = std::move(wav.channelData[1]);
        std::printf("Input: %s (stereo, %d Hz, %.2f s) - channel 0 = LEFT, 1 = RIGHT\n", opt.input.c_str(),
                    sampleRate, static_cast<double>(left.size()) / sampleRate);
    } else {
        std::vector<float> mono;
        if (opt.input.empty()) {
            mono = generateNoiseBursts(sampleRate, opt.seconds);
            std::printf("Input: built-in noise bursts (%d Hz, %.2f s)\n", sampleRate, opt.seconds);
        } else {
            WavData wav;
            if (!readWav(opt.input, wav, err)) {
                std::fprintf(stderr, "error: %s\n", err.c_str());
                return 1;
            }
            sampleRate = wav.sampleRate;
            mono = downmixToMono(wav);
            std::printf("Input: %s (%d ch -> mono, %d Hz, %.2f s)\n", opt.input.c_str(), wav.channels,
                        sampleRate, static_cast<double>(mono.size()) / sampleRate);
        }

        if (opt.useAzimuth) {
            sim = paramsFromAzimuth(opt.azimuthDeg, opt.spacing, 343.0, sampleRate);
        } else {
            double d = 0.0;
            if (opt.delaySamples >= 0.0) d = opt.delaySamples;
            else if (opt.delayMs >= 0.0) d = opt.delayMs * sampleRate / 1000.0;
            else if (opt.side != "center") d = 0.30 * sampleRate / 1000.0;
            if (opt.side == "left") d = -d;
            else if (opt.side == "center") d = 0.0;
            sim.tdoaSamples = d;
            sim.farEarAttenuationDb = opt.levelDb;
        }
        if (opt.snrDb >= 0.0f) sim.snrDb = opt.snrDb;

        const SimulatedStereo s = simulateTwoMics(mono, sim);
        left = s.left;
        right = s.right;
        simulated = true;

        const char* truth = sim.tdoaSamples < 0 ? "LEFT" : (sim.tdoaSamples > 0 ? "RIGHT" : "CENTER");
        std::printf("Simulation: source %s, delay %+.2f samples (%+.4f ms), far mic -%.1f dB",
                    truth, sim.tdoaSamples, sim.tdoaSamples * 1000.0 / sampleRate, sim.farEarAttenuationDb);
        if (std::isfinite(sim.snrDb)) std::printf(", SNR %.0f dB", sim.snrDb);
        std::printf("\n");

        if (!opt.writeSim.empty()) {
            if (!writeWav(opt.writeSim, {left, right}, sampleRate, err)) {
                std::fprintf(stderr, "error: %s\n", err.c_str());
                return 1;
            }
            std::printf("Wrote simulated stereo to %s\n", opt.writeSim.c_str());
        }
    }

    // ---- Direction engine -------------------------------------------------
    DirectionConfig cfg;
    cfg.micSpacingMeters = opt.spacing;
    DirectionEngine engine(cfg);
    TemporalSmoother smoother(cfg);

    std::printf("Mic spacing %.3f m, block %d samples (%.1f ms), search range +-%d samples\n\n",
                cfg.micSpacingMeters, opt.blockSize, 1000.0 * opt.blockSize / sampleRate,
                cfg.maxLagSamples(sampleRate));

    BufferStereoSource source(left, right, sampleRate);
    StereoBlock block;
    std::map<Direction, int> votes;
    int blocks = 0;
    while (source.read(block, static_cast<std::size_t>(opt.blockSize))) {
        if (block.frames() < static_cast<std::size_t>(opt.blockSize) / 2) break; // skip short tail
        const DirectionResult raw = engine.process(block.left, block.right, block.sampleRate);
        const DirectionResult smoothed = smoother.update(raw, block.sampleRate);
        ++votes[raw.direction];
        ++blocks;
        if (!opt.quiet) std::printf("%s\n", formatResultLine(block.startTimeSec, smoothed).c_str());
    }

    std::printf("\nPer-block votes (%d blocks):", blocks);
    for (const auto& [dir, count] : votes) std::printf("  %s=%d", toString(dir), count);
    std::printf("\n\n");

    const DirectionResult overall = engine.process(left, right, sampleRate);
    printResult("OVERALL (whole signal):", overall);

    if (simulated) {
        const double err_samples = overall.delaySamples - sim.tdoaSamples;
        std::printf("  truth       : %+.2f samples -> estimation error %+.2f samples (%+.4f ms)\n",
                    sim.tdoaSamples, err_samples, err_samples * 1000.0 / sampleRate);
    }
    return 0;
}
