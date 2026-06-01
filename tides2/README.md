# Tides v2

Tides v2 (2018) is a redesign of the original Tides module. Where v1 produced a single shaped slope output, v2 produces **four simultaneous, related outputs** from the same core waveform engine. Each output is derived from the same phase accumulator but with independent phase offsets, frequency ratios, or amplitude assignments, depending on the selected output mode. The module runs on an STM32F373 at 62500 Hz sample rate (`kSampleRate` in `io_buffer.h`), processes audio in blocks of 8 samples (`kBlockSize`), and uses a double-buffered I/O scheme (`IOBuffer`) to decouple DAC DMA from the DSP loop.

The SHEEP alternate firmware (a wavetable oscillator) is a separately compiled binary and is **not present** in this codebase.

---

## Code Architecture

### File Structure

| File | Role |
|---|---|
| `tides.cc` | Top-level: `main()`, interrupt handlers, `Process()` block callback |
| `io_buffer.h` | Double-buffered I/O, `Parameters` struct, block/sample constants |
| `cv_reader.cc/.h` | ADC polling, pot/CV summing, note tracking |
| `ramp_generator.h` | Phase accumulator; templated on `RampMode`, `OutputMode`, `Range` |
| `ramp_shaper.h` | Converts raw phase to skewed triangle/band-limited slope, EOA/EOR pulses |
| `poly_slope_generator.h/.cc` | Top-level DSP: drives `RampGenerator`, calls `RampShaper`/`RampWaveshaper`, produces 4 output samples per tick |
| `ramp/ramp_extractor.cc/.h` | Clock-to-ramp PLL; recovers a continuous ramp from a gate/clock input |
| `ramp/ratio.h` | `Ratio` struct: `{float ratio, int q}` for frequency-ratio locking |
| `settings.h/.cc` | Persistent state (`State`, `PersistentData`) stored in flash |
| `ui.cc/.h` | LED display, switch polling, mode changes |
| `resources.h/.cc` | Wavetable (`lut_wavetable`, 12300 int16 samples), fold LUTs |

---

### The Ramp Generator (`ramp_generator.h`)

`RampGenerator<4>` maintains a `master_phase_` accumulator and per-channel `phase_[4]`, `frequency_[4]`, and `ratio_[4]` arrays. It is fully templated on three enums so all code paths are resolved at compile time:

```
RampMode  : RAMP_MODE_AD | RAMP_MODE_LOOPING | RAMP_MODE_AR
OutputMode: OUTPUT_MODE_GATES | OUTPUT_MODE_AMPLITUDE |
            OUTPUT_MODE_SLOPE_PHASE | OUTPUT_MODE_FREQUENCY
Range     : RANGE_CONTROL | RANGE_AUDIO
```

`Step()` is called once per sample. Depending on mode:

- **AD mode** (`RAMP_MODE_AD`): Phase advances from 0 to 1 and clamps. A rising gate edge resets all active phases to 0. This is a one-shot envelope.
- **AR mode** (`RAMP_MODE_AR`): Phase sweeps 0→0.5 while the gate is high (attack), then 0.5→1 while low (release). The attack/release rate is scaled by `pw` (the SLOPE knob), creating variable-ratio A/R segments. Phase is clamped at 0.5 or 1.0 rather than wrapping.
- **Looping mode** (`RAMP_MODE_LOOPING`): `master_phase_` wraps freely at 1.0. Each channel's `phase_[i]` is derived as `(master_phase_ + wrap_counter_[i]) * ratio_[i].ratio` fractional part, giving phase-coherent sub/super harmonics. A `wrap_counter_` accumulates master-phase wraps; when it reaches `ratio_[i].q` (the least-common-multiple denominator), the ratio is updated to any pending `next_ratio_[i]`, ensuring glitch-free ratio changes at a zero-crossing.

When a clock/gate is patched to the CLOCK input, `RampExtractor` replaces the internal accumulator with a predicted ramp value passed in as the `ramp` float buffer. The `use_ramp` template bool switches `Step()` between self-clocked and externally-driven modes.

---

### The Ramp Extractor (`ramp/ramp_extractor.cc`)

When a signal is patched to the CLOCK input (input 1), `RampExtractor::Process()` converts it into a continuous 0–1 ramp. It stores a ring buffer of the last 16 `Pulse` records (on-duration, total-duration, pulse-width). On each rising edge it runs several period-prediction strategies concurrently and selects the winner by minimum squared error — an approach the code comments liken to early Scheirer/Goto beat trackers:

- **Moving average** (`predicted_period_[0]`): one-pole IIR on last-period length.
- **Periodic pattern matching** (`predicted_period_[1..8]`): for each pattern period 1–8, the predicted next period is simply the period from *i* cycles ago.

In `RANGE_AUDIO` mode (`smooth_audio_rate_tracking = true`) it switches to a PLL path: `target_frequency_` is updated per rising edge with a phase-error correction term, then `frequency_lp_` is smoothed toward it sample-by-sample via a one-pole filter whose coefficient (`lp_coefficient_`) tightens after lock. In `RANGE_CONTROL` mode it additionally uses the input gate's on-time vs. pulse width to refine the period estimate mid-cycle. The returned ramp is scaled by `Ratio.ratio` and reset every `Ratio.q` master cycles, enabling harmonic clock division/multiplication set by the FREQUENCY knob (transposition mapped through a 20-entry `kRatios` table).

---

### The Poly Slope Generator (`poly_slope_generator.h/.cc`)

`PolySlopeGenerator::Render()` is the outer DSP entry point called from `Process()` in `tides.cc`. It pre-processes parameters, dispatches via a compile-time function-pointer table (`render_fn_table_[ramp_mode][output_mode][range]`) to `RenderInternal<>`, then applies the smoothness filter.

**Parameter preprocessing:**
- `frequency` is clamped to 0.25 (Nyquist/2) to prevent aliasing.
- In `RANGE_CONTROL`, the `pw` (SLOPE) response below 0.5 is warped with a soft-clip to expand the useful attack-time range.
- `shape` and `smoothness` are attenuated at high frequencies via `Tame()`, a cubic rolloff that prevents aliasing from aggressive waveshaping harmonics.

**Per-sample loop in `RenderInternal<>`:**
1. `ParameterInterpolator` objects smooth `frequency_`, `pw_`, `shift_`, `shape_`, `fold_` across the block to prevent zipper noise.
2. `ramp_generator_.Step<>()` advances all phase accumulators.
3. The `shape` parameter is mapped to an index into `lut_wavetable` (12300 int16 samples = 12 waveforms × 1025 samples, with one-sample overlap for interpolation). `MAKE_INTEGRAL_FRACTIONAL` splits it into table index and fractional blend for smooth morphing.
4. Each of four `RampShaper` + `RampWaveshaper` instances converts the phase to a shaped output sample.

---

### The Four Outputs and How They Differ by Output Mode

The four outputs are defined by `OutputMode`, selected by a hardware button and stored in `State.output_mode`:

**`OUTPUT_MODE_GATES`** — Gates and shaped signal from a single oscillator:
- Channel 0: The fully shaped slope, multiplied by `shift` (acts as an attenuverter). Fold applied.
- Channel 1: The same raw `ramp_shaper_[0]` slope run through a fixed sinusoidal waveshaper (`lut_wavetable[8200]`), scaled ±5 V or 0–8 V depending on mode.
- Channel 2: End-of-Attack pulse (`RampShaper::EOA`), scaled ×8 to produce a gate/trigger.
- Channel 3: End-of-Ramp pulse (`RampShaper::EOR`), scaled ×8.

**`OUTPUT_MODE_AMPLITUDE`** — Single waveform panned across 4 outputs using SHIFT:
- All four channels receive the same shaped slope multiplied by a gain computed from `channel_index = |shift * 5.1|`. Each channel's gain is `max(1 - |channel - channel_index|, 0)`, a linear pan law (equal-power in audio range). SHIFT sweeps a "spotlight" across the four outputs.

**`OUTPUT_MODE_SLOPE_PHASE`** — Four phase-offset copies of the same waveform:
- All four channels run through their own `ramp_shaper_[j]` / `ramp_waveshaper_[j]`. The SHIFT knob sets `step = shift / 3` (control range) or `shift / 4` (audio range), and each channel's `phase_shift` is decremented by `step * j`. This produces four evenly-spaced phase copies: 0°, 90°, 180°, 270° at maximum SHIFT. In AR mode, each channel gets its own independently-tracked phase (all four `RampGenerator` phases are active). This mode runs at half the sample rate (`half_speed = true`) to allow per-channel processing within the CPU budget.

**`OUTPUT_MODE_FREQUENCY`** — Four independent oscillators at different frequency ratios:
- Each channel drives its own `ramp_generator_.phase_[j]` at `f0 * ratio_[j].ratio`. The SHIFT knob selects a frequency-ratio preset from either `audio_ratio_table_` or `control_ratio_table_` (21 entries each, spanning integer and just-intonation ratios from 1/8× to 8×). The four outputs thus produce harmonically related oscillators — for example, the preset at index 16 in audio mode gives ratios 1:1.5:2:3 (a fifth, octave, and twelfth). This mode also runs at half sample rate.

---

### SHAPE, SLOPE, and SMOOTHNESS at the DSP Level

- **SLOPE** (`pw`, 0–1): Controls the attack/release ratio of the waveform. In `RampShaper::SkewedRamp()`, `slope_up = 0.5 / pw` and `slope_down = 0.5 / (1 - pw)`. In AR mode the same pw gates which half of the 0–1 phase range is traversed faster. In `OUTPUT_MODE_SLOPE_PHASE`, pw is spread across channels: `per_channel_pw[j] = pw + pw_increment * j`.
- **SHAPE** (`shape`, 0–1): Selects and crossfades between 12 waveforms in `lut_wavetable`. The mapped index spans 5.0–10.999 (phasor/looping range) or 5.0–10.999 vs. 5.0–8.999 (audio range vs. control). `RampWaveshaper::Shape()` does bilinear interpolation within the wavetable: it interpolates within a row (waveform) and blends between two adjacent rows (morphing). In AR mode, `Shape()` adds extra state to re-anchor the waveshaper's output at the 0.5 phase breakpoint, keeping the attack and release segments independently shaped.
- **SMOOTHNESS** (`smoothness`, 0–1): Below 0.5 it applies a two-pole cascaded one-pole lowpass (`Filter<4>`) after waveform generation. The cutoff is proportional to the ramp frequency, creating a frequency-tracked lowpass that softens transients without changing perceived pitch. Above 0.5 it controls the `fold` amount fed to `Fold<>()`, which table-interpolates `lut_bipolar_fold` or `lut_unipolar_fold` to add wavefolding. In looping mode, folding is bipolar (±5 V); in AD/AR modes it is unipolar (0–8 V).

---

### Frequency Control

Frequency is computed in `tides.cc::Process()`. Without a CLOCK input, `frequency = kRoot[range] * SemitonesToRatio(transposition)`, where `kRoot` is `{0.125/SR, 2/SR, 130.81/SR}` for the three range settings (LFO, medium, audio, approximately 0.125 Hz, 2 Hz, and 130 Hz base frequencies). With a CLOCK input patched, frequency comes from `RampExtractor::Process()` and the FREQUENCY knob/CV acts as a clock-ratio selector through the 20-entry `kRatios` table, transposing the ramp extractor's output by integer-friendly ratios from 1/16× to 16×.

---

### Trigger and Gate Input Handling

Two gate inputs are handled per sample block:
- **Input 0 (TRIG/GATE)**: Passed as `GateFlags*` to `poly_slope_generator.Render()`. Rising edges trigger AD envelopes or reset looping ramps. High-level sustains the attack phase in AR mode.
- **Input 1 (CLOCK)**: When patched, routes through `RampExtractor` to produce the `ramp[]` float buffer. The normalization detection (`GateInputs::ReadNormalization`) checks whether input 1 is normalled to something by thresholding the FM CV ADC value.

For output modes that run at half speed (`OUTPUT_MODE_SLOPE_PHASE`, `OUTPUT_MODE_FREQUENCY`), `Process()` decimates the gate input by 2 before passing it to the DSP, and writes each computed output sample twice to the DAC buffer.

---

### Differences from Tides v1

The original Tides used a single-channel phase accumulator, a single `RampShaper`, and three fixed outputs (unipolar, bipolar, EOA/EOR). Tides v2 makes the following architectural changes:

- **Four outputs instead of three**: The output engine is built around four independent `RampShaper` + `RampWaveshaper` instances templated into a single per-sample loop.
- **Output mode switching**: The four output assignments are not fixed; they are runtime-selectable through four distinct `OutputMode` values, each with fundamentally different routing logic.
- **Wavetable-based shape morphing**: v1 used a single waveshaping curve. v2 stores 12 waveforms in `lut_wavetable` and continuously crossfades between them using bilinear interpolation, driven by the SHAPE knob.
- **Frequency-ratio outputs**: `OUTPUT_MODE_FREQUENCY` is entirely new — four independent oscillators at just-intonation or integer frequency ratios, tracking the same master clock.
- **Clock extraction PLL**: v1's clock sync was simpler. v2's `RampExtractor` runs multiple prediction strategies concurrently and selects between them dynamically, with a dedicated PLL path for audio-rate clock inputs.
- **Band-limited synthesis in audio range**: In `RANGE_AUDIO` looping mode, `BandLimitedSlope()` and `BandLimitedPulse()` use polyBLEP (`ThisIntegratedBlepSample` / `NextBlepSample`) to suppress aliasing at audio rates. Control range uses the aliasing-tolerant `SkewedRamp()` path.
- **SMOOTHNESS as dual-function parameter**: v1 had independent smoothness control mapped to a simple filter. v2 maps the lower half to a frequency-tracking 2-pole lowpass and the upper half to wavefolding, significantly expanding timbral range.
