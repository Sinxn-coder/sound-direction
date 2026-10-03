# Sound Direction Localization – MVP

An assistive-tech prototype for deaf and hard-of-hearing users. Two microphones sit on
the left and right sides of the user's head. This engine takes the two audio streams and
estimates **where a sound came from**.

> Scope: this module **only** estimates direction. It does **not** identify what the
> sound is (that is a separate module).

**Milestone 1 (this code):** LEFT / RIGHT / CENTER, using classical signal processing
(TDOA via normalized cross-correlation). No machine learning.
FRONT / BACK / UP / DOWN are in the `Direction` enum but the engine does not output them yet.

---

## What TDOA means

**TDOA = Time Difference Of Arrival.** Sound travels at about 343 m/s. If a sound comes
from your left, it reaches the left microphone slightly *before* the right one:

```
   source (left)
        \
         \  ))))      the path to the RIGHT mic is longer
    [L mic]--------18 cm--------[R mic]
```

With 18 cm spacing the largest possible straight-line difference is
`0.18 m / 343 m/s ≈ 0.52 ms`. Around a real head the path is longer (sound bends around
it), up to about 0.67 ms. At 48 kHz that is roughly ±25 to ±33 samples.

To measure that delay we slide one channel against the other and look for the shift
(lag) where the two match best. That is **cross-correlation**. We use the **normalized**
version:

```
ncc(k) = Σ R[n]·L[n+k] / sqrt( Σ R[n]² · Σ L[n+k]² )        (sums over the overlap)
```

The result is between -1 and 1 and does not depend on how loud each mic is. The `k` with
the highest value is the delay. The sign of `k` gives the side.

### Sign convention (used everywhere)

`delay = t_arrival(LEFT mic) − t_arrival(RIGHT mic)`

| delay | meaning                   | direction |
|-------|---------------------------|-----------|
| < 0   | left mic heard it first   | LEFT      |
| ≈ 0   | both heard it together    | CENTER    |
| > 0   | right mic heard it first  | RIGHT     |

Azimuth follows the same rule: −90° is full left, +90° is full right.

### Processing pipeline (`DirectionEngine::process`)

1. **Validate** the input: equal lengths, a valid sample rate, enough samples.
2. **Remove DC offset** from each channel. MEMS mics and ADCs often have a bias.
3. **Silence gate.** If the louder channel is below −55 dBFS, the result is `SILENT`.
4. **Limit the search range** using the geometry:
   `maxLag = ceil(spacing × 1.35 / c × sampleRate) + 1`. The 1.35 factor covers the
   longer path around the head. Any lag outside this range is physically impossible, so
   we never search there. That saves CPU and prevents false peaks.
5. **Normalized cross-correlation** over `[-maxLag, +maxLag]`.
6. **Sub-sample refinement.** A parabola is fitted through the peak and its two
   neighbours.
7. **Confidence** (0..1) is the product of these factors:
   - peak NCC value (how similar the aligned channels are)
   - uniqueness (how much higher the peak is than the next local maximum)
   - margin above the silence threshold
   - a penalty if the peak sits on the edge of the search range
   - a penalty if the level difference contradicts the timing
8. **Classify.** Confidence below 0.30 gives `UNKNOWN`. |azimuth| below 8° gives
   `CENTER`. Otherwise the result is `LEFT` or `RIGHT`.

`TemporalSmoother` then stabilizes results across blocks. It applies a
confidence-weighted moving average to the delay and holds the last direction for a few
blocks of silence, so the display does not flicker between syllables.

---

## Project layout

```
src/
  main.cpp                    CLI driver: WAV in -> simulate -> engine -> console
  audio/
    AudioSource.h             IStereoSource interface + StereoBlock (input abstraction)
    BufferStereoSource.*      streams in-memory buffers block by block
    WavFile.*                 WAV reader/writer (PCM 8/16/24/32, float 32/64)
  direction/                  <-- portable core, reused on the ESP32
    DirectionTypes.h          Direction enum + DirectionResult
    DirectionEngine.*         the localization engine + DirectionConfig
    CrossCorrelation.*        normalized cross-correlation + peak finding
    TemporalSmoother.*        smoothing over time
  simulation/
    BinauralSimulator.*       mono -> two virtual mics (fractional delay, level, noise)
    SignalGenerator.*         test signals (noise bursts, tones)
  utils/
    DspUtils.*                DC removal, RMS, dBFS
    ConsoleViz.*              ASCII direction gauge
tests/
  test_main.cpp               self-contained unit tests (run with ctest)
```

CMake builds two libraries:

- **`sdl_core`** contains `direction/` and `DspUtils`. It has no file I/O and no OS
  calls, and it does not throw. This is the part that moves to the ESP32 unchanged.
- **`sdl_desktop`** contains WAV, simulation and console code. It runs on the laptop only.

### Engine API

```cpp
sdl::DirectionConfig cfg;
cfg.micSpacingMeters = 0.18;           // configurable
sdl::DirectionEngine engine(cfg);

sdl::DirectionResult r = engine.process(left, right, sampleRate);
// r.direction, r.confidence, r.delaySamples, r.delayMs, r.azimuthDeg, ...
```

---

## Build

You need CMake ≥ 3.16 and a C++17 compiler.

**Windows (Visual Studio 2022/2026):** open a *Developer PowerShell for VS*, or call
the `cmake.exe` bundled with Visual Studio by its full path:

```powershell
cmake -S . -B build
cmake --build build --config Release
ctest --test-dir build -C Release        # run the unit tests
```

The binaries are written to `build\Release\sound_direction.exe` and
`build\Release\direction_tests.exe`.

**Linux / macOS:**

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build
```

The binaries are written to `build/sound_direction` and `build/direction_tests`.
The examples below use the Windows path.

---

## Run

### 0. Get a mono test WAV

Any mono or stereo WAV works. Stereo files are downmixed to mono before simulation. You
can also generate a broadband test signal:

```powershell
.\build\Release\sound_direction.exe --generate-test test.wav
```

If you leave out `--input`, the program generates the same signal internally.

### 1. Simulated LEFT sound

```powershell
.\build\Release\sound_direction.exe --input test.wav --side left --delay-ms 0.3 --level-db 6
```

This makes the right mic hear the sound 0.3 ms later and 6 dB quieter. Expected output
(abridged):

```
   0.00s  L [.........#.....|...............] R  LEFT    delay  -14.32 smp (-0.298 ms)  az   -35 deg  conf 0.97
...
OVERALL (whole signal):
  direction   : LEFT
  delay       : -14.32 samples  (-0.2984 ms)
  confidence  : 0.98
  truth       : -14.40 samples -> estimation error +0.08 samples (+0.0016 ms)
```

### 2. Simulated RIGHT sound

```powershell
.\build\Release\sound_direction.exe --input test.wav --side right --delay-samples 12 --level-db 4
```

### More options

```powershell
# Use a head model: azimuth -90 (left) .. +90 (right) sets both delay and level
.\build\Release\sound_direction.exe --input test.wav --azimuth 45

# Add noise to each mic (SNR in dB); --quiet prints only the summary
.\build\Release\sound_direction.exe --input test.wav --side right --snr 5 --quiet

# Save the simulated stereo file, then analyze it as if it were a real recording
.\build\Release\sound_direction.exe --input test.wav --side left --write-sim left.wav
.\build\Release\sound_direction.exe --input left.wav --stereo

# Different mic spacing or block size
.\build\Release\sound_direction.exe --input test.wav --side left --spacing 0.15 --block 1024
```

Run `--help` for the full list. A real two-mic recording (channel 0 = left,
channel 1 = right) can be analyzed with `--input rec.wav --stereo`.

---

## ESP32-S3 target

`esp32/` is a PlatformIO project (Arduino framework, board `esp32-s3-devkitc-1`). It
compiles the **same** core sources from `src/`. `src/library.json` restricts the build
to `direction/*.cpp` and `utils/DspUtils.cpp`, so WAV, simulation and console code can
never end up in the firmware.

The only build difference is `SDL_CORE_FLOAT_ACCUM`. The ESP32-S3 FPU is single
precision only, so the cross-correlation dot product accumulates in `float` instead of
`double`. To run the desktop tests with the same numerics:

```powershell
cmake -S . -B build-float -DSDL_CORE_FLOAT_ACCUM=ON
cmake --build build-float --config Release
.\build-float\Release\direction_tests.exe
```

There are two firmware environments (needs `pip install platformio`):

```powershell
cd esp32
pio run -e esp32-s3 -t upload; pio device monitor            # live: 2x INMP441 mics
pio run -e esp32-s3-selftest -t upload; pio device monitor   # synthetic self-test, no mics
```

**Live mode (`esp32-s3`, default).** Two INMP441 mics share a single I2S bus:
BCLK=GPIO4, WS=GPIO5, SD=GPIO6. Mic #1 has L/R=GND and becomes LEFT; mic #2 has
L/R=3.3V and becomes RIGHT. The firmware runs at 48 kHz and analyzes 1024-frame blocks
(21 ms). Each block goes through `DirectionEngine` and then `TemporalSmoother`, and
Serial prints the direction, confidence and delay. At boot it prints the pin and
rate configuration and a per-channel level check. The I2S code is isolated in
`esp32/src/Inmp441I2sSource.*`.

**Self-test mode (`esp32-s3-selftest`).** No microphones are needed. The firmware creates known two-channel buffers in RAM
(LEFT, RIGHT, CENTER, near-maximum delay, DC offset plus noise, silence, and a 16 kHz
case), runs them through `DirectionEngine`, and prints PASS/FAIL plus the time each
`process()` call takes. It also runs a smoother sequence. The suite repeats every
10 seconds:

```
[PASS] LEFT  -14 smp, far -6 dB              -> LEFT    delay  -14.00 smp (-0.292 ms) conf 1.00 |  NNNN us (...)
...
=== 8 / 8 passed ===
```

---

## Current limitations

- **Two mics only resolve left versus right.** A sound straight ahead and one straight
  behind produce the same delay (the "cone of confusion"). That is why the engine reports
  `CENTER` rather than FRONT or BACK. Telling front from back, or up from down, needs
  more mics, head-shadow and spectral cues, or head movement.
- **Azimuth is a free-field estimate.** It uses `asin(delay·c/spacing)`. Around a real
  head the delays are larger, so angles above about 60° are overestimated and clamp at
  ±90°. The LEFT/RIGHT decision is not affected.
- **Pure tones and strongly periodic sounds are ambiguous.** A 3 kHz tone repeats every
  16 samples at 48 kHz, so several lags match equally well. The engine detects this and
  lowers its confidence, but it cannot localize such sounds reliably. Broadband sounds
  work best: claps, knocks, speech, door slams, most alarms.
- **One source at a time.** With two simultaneous sources, the louder or more broadband
  one wins.
- **Reverberation** is only simulated as uncorrelated noise. Real rooms add reflections
  that create false peaks. GCC-PHAT weighting is the standard fix and the next upgrade.
- **The two channels must be sample-synchronized.** A clock offset between the mics
  appears as a direction bias. On the ESP32 both mics must share one I2S bus (stereo
  mode, one WS/BCLK), not two independent ones.
- **The simulator is idealized:** a pure delay and gain, with no head-related filtering.
- **Thresholds are untuned:** silence −55 dBFS, minimum confidence 0.30, center ±8°. They
  are reasonable defaults but have not been calibrated on real mic data.

## Next steps

1. An `Esp32I2sSource` that implements `IStereoSource`, either on-device or as a laptop
   reader for a serial/UDP stream from the ESP32-S3. The engine stays unchanged.
2. GCC-PHAT option for reverberant rooms.
3. Calibrate thresholds and the mic-delay offset with the real hardware.
4. FRONT/BACK, UP/DOWN using additional cues or mics.
