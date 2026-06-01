# Warps — Source Code Reference

Warps is a Mutable Instruments Eurorack meta-modulator. It accepts two audio signals — a *carrier* and a *modulator* — and combines them using one of several algorithms spanning crossfading, ring modulation, wavefolding, bit crushing, phase modulation, and vocoding. An internal oscillator can replace the external carrier, and a hidden Easter egg firmware replaces the main algorithms with a frequency-shifting / quadrature modulation engine. The module runs on an STM32F4 at a 96 kHz native sample rate.

---

## Code Architecture

### File Structure

```
warps/
  warps.cc                    — Entry point, ISR callback, global singletons
  cv_scaler.cc / .h           — ADC reading, CV scaling, normalization detection
  settings.cc / .h            — Persistent calibration storage
  ui.cc / .h                  — Button handling, LED output, mode state machine
  meter.h                     — LED meter helper
  resources.cc / .h           — Lookup tables (sin, fold, xfade, MIDI pitch)
  dsp/
    parameters.h              — Parameters struct shared between CV layer and DSP
    modulator.cc / .h         — Central DSP class; algorithm dispatch and processing
    oscillator.cc / .h        — Internal carrier oscillator (sine, triangle, saw, pulse, noise)
    quadrature_oscillator.h   — Wavetable I/Q oscillator used by the Easter egg
    quadrature_transform.h    — Hilbert-transform approximation via allpass chain
    filter_bank.cc / .h       — Multi-rate 20-band filter bank for the vocoder
    vocoder.cc / .h           — Vocoder engine built on top of the filter bank
    limiter.h                 — Look-ahead limiter used after vocoder synthesis
    sample_rate_converter.h   — Polyphase FIR up/downsampler (ratio-6 for xmod path)
    sample_rate_conversion_filters.h — Polyphase FIR coefficients
```

### Execution Flow

`main()` calls `Init()`, which boots all subsystems and hands off to the codec DMA callback `FillBuffer()`. That callback, called at a block rate of 96 000 / 60 ≈ 1.6 kHz, executes three operations in order:

1. `cv_scaler.DetectAudioNormalization()` — blanks internal normalization-probe samples in the audio stream.
2. `cv_scaler.Read(modulator.mutable_parameters())` — reads all ADC channels and writes a `Parameters` struct.
3. `modulator.Process()` — runs the selected algorithm and writes the output buffer.

---

### The `Modulator` Class

`Modulator` is the entire DSP engine. It owns:

- Two `SaturatingAmplifier` instances — input VCA/saturation per channel.
- An `Oscillator` (`xmod_oscillator_`) and a second `Oscillator` (`vocoder_oscillator_`) — the internal carrier generators.
- A `QuadratureOscillator` and two `QuadratureTransform` instances — used exclusively by the Easter egg path.
- Two `SampleRateConverter<SRC_UP, 6, 48>` upsamplers and one `SampleRateConverter<SRC_DOWN, 6, 48>` downsampler — antialiasing wrapper for the cross-modulation algorithms.
- A `Vocoder` instance.
- Working buffers: `buffer_[3][96]` (float) and `src_buffer_[2][576]` (oversampled).

`Process()` dispatches to either `ProcessEasterEgg()` or the main algorithm path based on the `easter_egg_` flag.

---

### The `Parameters` Struct

Defined in `dsp/parameters.h`, `Parameters` carries every control value into the DSP layer:

| Field | Source | Range |
|---|---|---|
| `channel_drive[2]` | LEVEL 1/2 pots + CV | 0–1, quadratic; drives `SaturatingAmplifier` |
| `modulation_algorithm` | ALGORITHM pot + CV | 0–1 continuous; selects + blends algorithms |
| `modulation_parameter` | TIMBRE pot + CV | 0–1; per-algorithm character control |
| `carrier_shape` | button cycle | 0 = external, 1–4 = sine/triangle/saw/pulse, 5 = noise |
| `note` | LEVEL 1 CV + pot | MIDI pitch for internal oscillator |
| `frequency_shift_pot/cv`, `phase_shift` | ALGORITHM pot + CV | Easter egg parameters |

`skewed_modulation_parameter()` applies a concave non-linearity to TIMBRE for algorithms 1–4, so movement near 0 and 1 is compressed and the middle of the range is stretched.

---

### Algorithm Selection and Dispatch

`modulation_algorithm` is a 0–1 float. The main cross-modulation zone covers 0–0.75 (six algorithm slots); the vocoder zone covers 0.75–1.0, with a 0.75–0.80 transition crossfade.

**Xmod dispatch** maps `modulation_algorithm * 8` into a 0–5 integer index into `xmod_table_[]`. Each entry is a pointer to a template instantiation of `ProcessXmod<A, B>`, which simultaneously evaluates both adjacent algorithms `A` and `B` and linearly interpolates between them using the fractional part of the index. This gives a continuous morph between any two adjacent algorithms at sample-level resolution with zero zipper noise.

```cpp
static XmodFn xmod_table_[] = {
  &Modulator::ProcessXmod<ALGORITHM_XFADE,             ALGORITHM_FOLD>,
  &Modulator::ProcessXmod<ALGORITHM_FOLD,              ALGORITHM_ANALOG_RING_MODULATION>,
  &Modulator::ProcessXmod<ALGORITHM_ANALOG_RING_MODULATION, ALGORITHM_DIGITAL_RING_MODULATION>,
  &Modulator::ProcessXmod<ALGORITHM_DIGITAL_RING_MODULATION, ALGORITHM_XOR>,
  &Modulator::ProcessXmod<ALGORITHM_XOR,               ALGORITHM_COMPARATOR>,
  &Modulator::ProcessXmod<ALGORITHM_COMPARATOR,        ALGORITHM_NOP>,
};
```

All six xmod algorithms run at 6× oversampling (576 samples per block at 96 kHz) to push nonlinear aliasing products above 48 kHz before downsampling.

---

### Crossfading — `ALGORITHM_XFADE`

```cpp
return x_1 * fade_in + x_2 * fade_out;
```

`fade_in` and `fade_out` are read from a pair of lookup tables (`lut_xfade_in`, `lut_xfade_out`) indexed by TIMBRE. At TIMBRE = 0 only `x_1` (modulator) passes; at TIMBRE = 1 only `x_2` (carrier) passes. The tables implement a constant-power curve so perceived loudness is stable across the blend.

---

### Wavefolding — `ALGORITHM_FOLD`

```cpp
sum = x_1 + x_2 + x_1 * x_2 * 0.25f;
sum *= 0.02f + parameter;
return Interpolate(lut_bipolar_fold + 2048, sum, kScale);
```

The two inputs are summed with a mild quadratic cross-term, then scaled by a gain that grows with TIMBRE. The scaled sum is fed into a bipolar wavefolder lookup table (`lut_bipolar_fold`), a symmetric waveshaper that maps large excursions back toward zero in a fold-back pattern. At low TIMBRE the output is nearly linear; high TIMBRE drives deep folding with dense upper harmonics.

---

### Analog Ring Modulation — `ALGORITHM_ANALOG_RING_MODULATION`

```cpp
carrier *= 2.0f;
float ring = Diode(modulator + carrier) + Diode(modulator - carrier);
ring *= (4.0f + parameter * 24.0f);
return SoftLimit(ring);
```

Based on Julian Parker's "A simple Digital model of the diode-based ring-modulator" (DAFx-11). `Diode()` approximates the transfer characteristic of a physical diode:

```cpp
float dead_zone = fabs(x) - 0.667f;
dead_zone += fabs(dead_zone);
dead_zone *= dead_zone;
return 0.04324765822726063f * dead_zone * sign;
```

The dead-zone threshold (0.667) models diode forward voltage. The output of the two-diode sum produces a signal dominated by difference and sum frequencies, with harmonic distortion characteristic of real diodes. TIMBRE scales the output gain (4× to 28×) — higher gain drives the `SoftLimit` harder, adding saturation.

---

### Digital Ring Modulation — `ALGORITHM_DIGITAL_RING_MODULATION`

```cpp
float ring = 4.0f * x_1 * x_2 * (1.0f + parameter * 8.0f);
return ring / (1.0f + fabs(ring));
```

A direct multiplication followed by soft-limiting via `x / (1 + |x|)`. TIMBRE increases gain before the limiter, pushing the output from pure multiplication toward hard-limited, distorted ring modulation.

---

### XOR — `ALGORITHM_XOR`

```cpp
short x_1_short = Clip16(x_1 * 32768.0f);
short x_2_short = Clip16(x_2 * 32768.0f);
float mod = static_cast<float>(x_1_short ^ x_2_short) / 32768.0f;
float sum = (x_1 + x_2) * 0.7f;
return sum + (mod - sum) * parameter;
```

Signals are quantized to 16-bit integers and bitwise-XORed. TIMBRE crossfades from a plain mix of the two inputs (sum) to the raw XOR output. Low TIMBRE preserves the audio character of both signals; high TIMBRE fully engages bit-crushing artifacts where bit inversions create noise floors and inharmonic spectral content.

---

### Comparator — `ALGORITHM_COMPARATOR`

```cpp
float direct    = modulator < carrier ? modulator : carrier;
float window    = fabs(modulator) > fabs(carrier) ? modulator : carrier;
float window_2  = fabs(modulator) > fabs(carrier) ? fabs(modulator) : -fabs(carrier);
float threshold = carrier > 0.05f ? carrier : modulator;
float sequence[4] = { direct, threshold, window, window_2 };
```

TIMBRE selects and interpolates across four sample-selection functions. At TIMBRE = 0, `direct` is a sample-selector that passes the smaller sample each tick — a half-wave-rectifying comparator. Intermediate positions step through `threshold` (which gates on carrier polarity) and `window` (an amplitude-tracking selector). At TIMBRE = 1, `window_2` produces a full-wave rectified carrier-envelope-locked signal. All produce heavy waveshaping without multiplication.

---

### Vocoder — `MODULATION_ALGORITHM_VOCODER`

The `Vocoder` class uses two `FilterBank` instances — one for the modulator, one for the carrier — with 20 bands spanning the audio range. `FilterBank` is a multi-rate structure: the lowest bands decimate by factor 4, mid bands by factor 3, and high bands run at full rate; per-group `SampleRateConverter` instances handle decimation and interpolation.

For each band, an `EnvelopeFollower` tracks the modulator amplitude with asymmetric attack/release times. The tracked envelope is used to amplitude-modulate the corresponding carrier band. TIMBRE controls formant shift: it slides which modulator band drives each carrier band, effectively transposing vocal formants up or down. The ALGORITHM knob in the vocoder zone controls envelope release time via `set_release_time()`, from fast (bright, consonant) to frozen (infinite hold). A `Limiter` follows synthesis to prevent intersample clipping.

---

### Internal Oscillator

When `carrier_shape` is non-zero, `Oscillator::Render()` generates the carrier internally via a function pointer table (`fn_table_[]`):

- Shape 1 (SINE): table-lookup with phase modulation from input 1 (phase-mod FM).
- Shapes 2–4 (TRIANGLE, SAW, PULSE): polyblep-corrected oscillators with FM modulation applied as a phase-increment scale factor.
- Shape 5 (NOISE): white noise passed through an SVF lowpass tuned to `note`, with a `Duck()` function that attenuates the noise when external audio is detected on that input.

`note` is derived from the LEVEL 1 CV jack (1 V/oct) summed with the LEVEL 1 pot (C2–C7 range). When the LEVEL 1 input is normalled (nothing plugged in), the module detects this and maps the pot to a fixed pitch offset.

For the xmod algorithms, the carrier is generated by `xmod_oscillator_` using shapes sine/triangle/saw. For the vocoder, `vocoder_oscillator_` generates saw/pulse/noise (carrier_shape offset by 2 internally), which are spectrally richer carriers more appropriate for vocoding.

---

### CV Inputs and Normalization Detection

`CvScaler::Read()` samples four ADC channels (LEVEL 1, LEVEL 2, ALGORITHM, PARAMETER), lowpass-filters them (coefficient ≈ 0.08 for CV, 0.026 for pots), applies calibration offsets, and writes the `Parameters` struct.

The module detects whether jacks are normalled (unconnected) using a two-pronged scheme. For CV inputs, a `NormalizationProbe` injects known random noise onto the normalling connection; if the ADC value correlates with the probe pattern, the jack is determined unconnected. For audio inputs, `DetectAudioNormalization()` looks for the known probe signal amplitude range in the audio stream. When a channel is detected as normalled, `channel_drive` is derived from the pot alone (squared, so low levels are accessible) and `note` is offset to C4.

---

### Easter Egg — Frequency Shifter

Activated by a secret button sequence detected in `Ui::DetectSecretHandshake()`, the Easter egg replaces the cross-modulation engine with a frequency shifter based on single-sideband modulation.

`ProcessEasterEgg()` decomposes both inputs into analytic signals (I/Q pairs). When the carrier is internal, `QuadratureOscillator::Render()` generates a quadrature wavetable pair directly. When it is external, `QuadratureTransform::Process()` uses a chain of first-order allpass filters (initialized from `lut_ap_poles`) that approximate a Hilbert transform — their outputs are 90° apart across the audio band.

The modulator is similarly decomposed through a second `QuadratureTransform`. Complex multiplication of the carrier and modulator I/Q pairs yields two sidebands:

```cpp
float up   = carrier_i * modulator_i - carrier_q * modulator_q;  // upper sideband
float down = carrier_i * modulator_i + carrier_q * modulator_q;  // lower sideband
```

TIMBRE crossfades between the upper and lower sidebands using `lut_xfade_in` / `lut_xfade_out`, with the complementary blend sent to the auxiliary output. The ALGORITHM knob controls the shift frequency (with a cubic-to-exponential mapping for fine control near 0 Hz), and feedback from the main output can be mixed back into the modulator input for metallic, recursive timbres.
