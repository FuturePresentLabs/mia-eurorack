# Tides (v1) — Tidal Modulator

Tides is a function generator that produces smoothly curved rise-and-fall shapes from a single phase accumulator. The same core waveform can serve as an LFO, an envelope, or an audio-rate oscillator depending on the selected range and mode. Three knobs — SLOPE, SHAPE, and SMOOTHNESS — independently control the rise/fall time ratio, the curvature of each segment, and a post-processing waveshaper/filter stage.

---

## Code Architecture

### File Structure

| File | Role |
|---|---|
| `tides.cc` | Entry point. Owns all global objects, wires the ISR, drives the main loop. |
| `generator.h / generator.cc` | Core DSP engine (`class Generator`). All waveform math lives here. |
| `cv_scaler.h / cv_scaler.cc` | ADC reading, smoothing, calibration, and unit conversion to signed 16-bit values. |
| `ui.h / ui.cc` | Button handling, mode/range state machine, LED output, settings persistence. |
| `resources.h / resources.cc` | All ROM tables: pitch-to-increment, waveform shapes, waveshaper curves, wavetable banks. |
| `drivers/` | Thin STM32 peripheral wrappers: ADC (SPI), DAC (SPI), gate inputs (GPIO), gate outputs (GPIO), LEDs, switches. |
| `plotter.cc / plotter.h` | Easter-egg vector plotter, activated by a specific button sequence. |

### Clocking and Double-Buffer Architecture

The hardware timer fires at 96 kHz. Each ISR tick advances the DAC and reads the LEVEL CV at sample rate. Every other tick (48 kHz effective output rate) it calls `generator.Process(gate_input.Read())`, which dequeues one `GeneratorSample` from a double-buffered ring of two 16-sample blocks (`kNumBlocks = 2`, `kBlockSize = 16`). The main loop detects a free render block via `generator.writable_block()`, writes the current CV-scaled parameters into the generator, then calls `generator.Process()` (the batch version) to fill the next block. This ISR/main-loop split keeps audio output jitter-free while allowing relatively expensive parameter-update math in the background.

In LOW range mode `clock_divider_` is set to 4, so the DAC ISR only dequeues a new sample every four ticks, effectively running the generator at 12 kHz and buying additional CPU headroom for very slow LFO rates.

### Core Waveform Generation

The generator maintains a 32-bit unsigned phase accumulator (`phase_`) that wraps from `0` to `0xFFFFFFFF`. `phase_increment_` is computed by `ComputePhaseIncrement()`, which looks up a value from `lut_increments[]` (a pre-computed table mapping semitone pitch to per-sample phase delta) and shifts it to match the clock divider.

The raw phase space is divided at a configurable mid-point into an attack segment (phase rises from 0 to `mid_point`) and a release segment (phase continues from `mid_point` to `0xFFFFFFFF`). After the segments are defined, the linear phase value is mapped through a waveform table lookup — a `Crossfade115` call that interpolates between two adjacent 2049-sample ROM waveforms.

Two processing paths exist:

- **`ProcessAudioRate`** — used in HIGH range (audio oscillator). Operates on raw 48 kHz samples. Uses polyBLEP (Band-Limited Step) correction to suppress the aliasing transient at the waveform reset and at the attack/release crossover point. The `NextIntegratedBlepSample` and `ThisIntegratedBlepSample` inline methods compute a polynomial approximation of an integrated BLEP, scaled by the slope discontinuity magnitude.

- **`ProcessControlRate`** — used in MEDIUM and LOW ranges (LFO/envelope). Operates at the divided-down rate. Skips BLEP processing entirely (no audible aliasing at control rates) and instead uses `lut_slope_compression[]` for slope skewing. SLOPE is low-pass filtered inside the loop (`smoothed_slope_`) to prevent zipper noise when the knob is moved.

After either path, `ProcessFilterWavefolder` runs unconditionally on the resulting block.

### How SLOPE Controls Rise/Fall Asymmetry

In `ProcessControlRate`, `slope_` (a signed 16-bit value centred at 0) is first passed through `lut_slope_compression[]` via `Interpolate88` to get a `slope_offset` in the range `[0, 65536]`. This offset is used directly as `end_of_attack` (the phase value at which the attack ends):

```
decay_factor  = (32768 << kSlopeBits) / slope_offset
attack_factor = (32768 << kSlopeBits) / (65536 - slope_offset)
```

The raw phase is then remapped: during the attack half the phase is multiplied by `decay_factor`; during the release half it is scaled by `attack_factor`. The result, `skewed_phase`, is a 0-to-`0xFFFFFFFF` value where the transition point between rising and falling halves tracks the SLOPE knob. At centre SLOPE the two segments are equal length; turned fully clockwise the attack collapses to nearly zero and the waveform becomes a pure falling ramp; fully counter-clockwise gives a pure rising ramp.

In `ProcessAudioRate` the same idea is implemented differently: `end_of_attack` is set to `(slope_ + 32768) << 16` so it directly lives in 32-bit phase space, and the instantaneous slopes `slope_up` and `slope_down` are recomputed each sample as `0xFFFFFFFF / mid_point` and `0xFFFFFFFF / ~mid_point` respectively. A one-pole smoothing filter on `mid_point` (`mid_point = (mid_point >> 5)*31 + (end_of_attack >> 5)`) prevents clicks when SLOPE is modulated.

### How SHAPE Controls Waveform Curvature

SHAPE selects which pair of waveform tables to crossfade between. In HIGH range (audio), `shape_` is first attenuated by `attenuation_` (see below), then mapped to a `wave_index` starting at `WAV_INVERSE_TAN_AUDIO`. The five audio waveforms in order are: inverse-tan (log-like), inverse-sin, linear triangle, sin, and tan (exponential-like). Crossfading across them via `shape_xfade` sweeps the curvature of both the attack and release segments continuously from a sharp exponential-decay-like shape through a triangle to an inverted exponential shape.

In MEDIUM/LOW range the same mechanism uses the control-rate table bank starting at `WAV_REVERSED_CONTROL`, which contains six shapes tuned for envelope use: reversed, spiky-exp, spiky, linear, bump, bump-exp, and normal (Gaussian-like).

In both cases, the waveform lookup is a 16-bit interpolation between two ROM tables of 2049 entries each, giving effectively alias-free waveshaping at any point in the SLOPE/SHAPE space.

### How SMOOTHNESS Controls Waveshaping

SMOOTHNESS operates as a two-stage post-processor applied in `ProcessFilterWavefolder` after the main waveform generators complete:

1. **Low-pass filter**: A two-pole state-variable filter (`uni_lp_state_[2]` and `bi_lp_state_[2]`) smooths both the unipolar and bipolar outputs. The cutoff frequency is computed by `ComputeCutoffFrequency()`: at the negative extreme of SMOOTHNESS the cutoff tracks slightly below the oscillator pitch; at the midpoint it tracks at pitch; at zero (noon) it opens fully (`256 << 7`, effectively bypassed). This produces a progressive low-pass effect that rounds the waveform edges as SMOOTHNESS decreases.

2. **Wavefolding**: When SMOOTHNESS is positive, `wf_gain` grows and both outputs are run through `wav_bipolar_fold[]` or `wav_unipolar_fold[]` tables via `Interpolate1022`. The result is blended with the filtered signal by `wf_balance`. At maximum SMOOTHNESS the output folds through several cycles of the wavefolder, producing harmonically rich overtones.

### Operating Modes

Three modes are set by pressing button 0 and persisted to flash via `mode_storage`:

- **`GENERATOR_MODE_LOOPING`** (`running_` starts true): The phase accumulator wraps continuously. This is the standard LFO/oscillator mode. `FLAG_END_OF_RELEASE` fires at every wrap; `FLAG_END_OF_ATTACK` fires whenever phase crosses `end_of_attack`.

- **`GENERATOR_MODE_AD`** (attack-decay): A rising trigger on the GATE input sets `phase = 0` and `running_ = true`. When the phase wraps (completes a full cycle), `running_` is set false and the output holds at zero. The generator fires once per trigger and stops.

- **`GENERATOR_MODE_AR`** (attack-release with sustain): Like AD, but when the phase reaches `end_of_attack` (i.e. the attack peak) and `CONTROL_GATE` is still high, `phase` is held frozen at the peak (`skewed_phase = 1 << 31`). When the gate goes low, the release segment proceeds normally.

Three frequency ranges, set by button 1, select the operating band:

- **HIGH**: 8 Hz–ish to beyond audio rate. Uses `ProcessAudioRate` with BLEP anti-aliasing.
- **MEDIUM**: Approximately 0.5 Hz to 8 Hz. Uses `ProcessControlRate`.
- **LOW**: Sub-audio, very slow LFO. Uses `ProcessControlRate` with `clock_divider_ = 4`.

### Frequency and Sync

Pitch is expressed internally as a fixed-point semitone value (128 units = 1 semitone). `CvScaler::pitch()` combines the 1V/Oct input and the FM input (scaled by the attenuverter, using `lut_attenuverter_curve[]` for a non-linear response) to produce this value. `set_pitch()` adjusts the reference centre by range so that the panel pitch knob spans a useful range in each mode.

When sync mode is engaged (long-press of button 1), the CLOCK input feeds a PLL rather than a free oscillator:

- In HIGH range: a `local_osc_phase_increment_` tracks `target_phase_increment_` with a fast IIR filter (right-shift 8), and a slow phase-error correction term (`phase_error >> 13`) nudges `phase_increment` each sample. This is a classic first-order digital PLL.
- In MEDIUM/LOW range: `sync_counter_` measures the interval between rising CLOCK edges. A `PatternPredictor<32,8>` object predicts the next period for non-uniform clock sources, and `phase_increment` is set directly from the predicted period scaled by `frequency_ratio_`.

`frequency_ratio_` (p/q) allows the generator to lock at musical ratios (1:1, 5:4, 4:3, 3:2, 5:3, 2:1, 3:1, 4:1, 6:1, 8:1, 12:1, 16:1) relative to the incoming clock, selected by the pitch CV/knob position in sync mode.

### Trigger and Gate Inputs

`GateInput::Read()` samples GPIO port B each ISR tick and constructs a bitmask (`ControlBitMask`) that includes both the current level and edge-detect flags computed from the previous state:

- `CONTROL_GATE` / `CONTROL_GATE_RISING` / `CONTROL_GATE_FALLING`: The TRIG/GATE input.
- `CONTROL_CLOCK` / `CONTROL_CLOCK_RISING`: The CLOCK input used for sync.
- `CONTROL_FREEZE`: When high, the generator output is frozen at its current value; any trigger or reset is ignored.

A rising gate edge (`CONTROL_GATE_RISING`) always resets the phase to 0 and sets `running_ = true`, regardless of mode. This is the universal retrigger mechanism.

### Output Format and DAC Mapping

Each rendered `GeneratorSample` carries three fields:

- `unipolar` (uint16_t, 0–65535): The waveform mapped to `[0, +8V]` range. The LEVEL CV scales this via `uni = uni * level >> 16` before it reaches the DAC.
- `bipolar` (int16_t, -32768 to +32767): The same waveform centred at zero, covering `[-5V, +5V]`. It is negated and offset to `bi + 32768` before the DAC write, so 0 maps to the DAC midscale.
- `flags` (uint8_t): Bitmask of `FLAG_END_OF_ATTACK` and `FLAG_END_OF_RELEASE`, driven out as gate pulses on the two output jacks. The LED colour also switches between green (attack) and red (release) based on `FLAG_END_OF_ATTACK`.

### Anti-Aliasing Attenuation in Audio Mode

`ComputeAntialiasAttenuation()` computes `attenuation_` as a polynomial in pitch, slope, shape, and smoothness. This value is used to reduce the SHAPE parameter (flattening the waveform toward a triangle) and the SMOOTHNESS wavefolding gain as pitch increases. This limits the harmonic content of the signal before it can exceed Nyquist, providing a software-driven band-limiting scheme that complements the BLEP correction.

### Easter Egg

`UI_MODE_PAQUES` (French for Easter) is triggered by a specific double-long-press sequence. In this mode the main loop routes DAC output through `Plotter::Run()`, drawing Lissajous-style vector figures using a bytecode interpreter defined in `easter_egg/plotter_program.h`.
