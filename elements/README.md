# Elements — Code Architecture

Elements is a Eurorack module by Mutable Instruments implementing modal synthesis: a physical model of a resonant object (plate, bar, membrane, string) being excited by bowing, blowing, or striking. The synthesis engine is sample-accurate at 32 kHz and runs on an STM32F4 microcontroller.

---

## Directory / File Structure

```
elements/
  elements.cc          — main entry point, codec ISR, global objects
  cv_scaler.{h,cc}     — pot/CV reading, calibration, Patch/PerformanceState population
  ui.{h,cc}            — LED metering, button handling, factory test
  meter.h              — exponential peak follower helper
  drivers/             — hardware peripherals (codec, ADCs, gate input, LEDs)
  dsp/
    patch.h            — plain-old-data parameter struct (Patch)
    part.{h,cc}        — top-level DSP coordinator; owns Voice, Reverb, OminousVoice
    voice.{h,cc}       — one synthesis voice: three Exciters + Tube + Resonator/String
    exciter.{h,cc}     — seven excitation models (bow/blow/strike signal generators)
    resonator.{h,cc}   — modal filter bank (up to 64 SVF modes + bowed waveguide modes)
    tube.{h,cc}        — waveguide tube (blow resonator feedback loop)
    string.{h,cc}      — Karplus-Strong / physical string model (alternate resonator)
    multistage_envelope.{h,cc} — multi-segment ADSR with looping and variable shape
    ominous_voice.{h,cc} — easter-egg alternate voice (different synthesis approach)
    dsp.h              — shared constants: kSampleRate (32000), kMaxBlockSize
    fx/
      fx_engine.h      — compile-time delay-line memory allocator + DSP context
      diffuser.h       — 4-allpass granular diffuser (applied to blow buffer)
      reverb.h         — Griesinger plate reverb (Dattorro topology)
```

---

## Signal Chain Overview

```
CV/Gate/Pots
     |
  CvScaler::Read()  →  Patch + PerformanceState
     |
  Part::Process()
     |
  Voice::Process()
     |
  ┌──────────────────────────────────────┐
  │  Exciter: bow  (ProcessFlow)         │  ─→ bow_buffer[]
  │  Exciter: blow (ProcessGranularSP)   │  ─→ blow_buffer[]  →  Tube  →  Diffuser
  │  Exciter: strike (meta-selectable)   │  ─→ strike_buffer[]
  │  External blow_in / strike_in        │
  │                                      │
  │  Summed into raw[]                   │
  └──────────────────┬───────────────────┘
                     │
              Resonator::Process()    (modal SVF bank + bowed waveguide)
              or String::Process()    (Karplus-Strong, alternate model)
                     │
              center[] + sides[]  →  stereo spread
                     │
  Part: SoftLimit  →  Reverb::Process()
                     │
              Codec output (L = aux, R = main)
```

---

## Hardware I/O Pipeline

`elements.cc` registers `FillBuffer` as the codec DMA callback (16-bit, 32 kHz, stereo). On each
block (nominally 16 samples):

1. ADC inputs are split: right channel = blow, left channel = strike.
2. A fast-attack / slow-release noise gate is applied to both inputs using an asymmetric leaky
   integrator (`error > 0 ? 0.1 : 0.0001`). Below a threshold of `0.0001` the signal is linearly
   attenuated, suppressing codec noise floor without a hard cut.
3. `cv_scaler.Read()` scans one pot per call (round-robin via `pots_.Scan()`) and reads all CVs,
   populating `Patch` and `PerformanceState`.
4. `part.Process()` is called; results written back to `Codec::Frame` via `SoftConvert` (soft clip
   to 16-bit).

The reverb buffer (`uint16_t reverb_buffer[32768]`) is placed in CCM RAM via
`__attribute__((section(".ccmdata")))`, freeing the main SRAM bus for the audio buffers.

---

## CV / Knob Mapping (`cv_scaler.cc`)

`CvScaler` reads two ADC banks: `PotsAdc` for 27 front-panel potentiometers and `CvAdc` for 7 CV
inputs. Pot values are mapped through one of four transfer laws:

- `LAW_LINEAR` — direct 0–1 mapping
- `LAW_QUADRATIC_BIPOLAR` — `±(x²·3.3)`: gives fine control near zero for attenuverters
- `LAW_QUARTIC_BIPOLAR` — `±(x⁴·3.3)`: even finer center control (used for FM attenuverter)
- `LAW_QUANTIZED_NOTE` — 5-octave quantised pitch range with hysteresis (coarse tune knob)

The `BIND` macro combines pot and attenuverter-scaled CV into a single destination field with
optional slew limiting. Parameters controlling the resonator geometry and position use very
slow slew (`0.01–0.05`) to prevent zipper noise from propagating through the filter bank.

Pitch from V/oct is calibrated with a two-point `C1`/`C3` scheme stored in flash. MIDI pitch is
converted to normalized frequency via a split lookup table (`lut_midi_to_f_high` × `lut_midi_to_f_low`)
with 8.8 fixed-point indexing.

The `strength` parameter (`PerformanceState::strength`) comes from the Pressure CV input inverted:
`1.0 - cv_.float_value(CV_ADC_PRESSURE)`. It scales exciter amplitude across all three exciters
using a two-stage lookup (`lut_accent_gain_coarse` × `lut_accent_gain_fine`).

---

## The `Patch` Struct — Parameter Space

`dsp/patch.h` defines all synthesis parameters as normalized floats:

| Field | What it controls |
|---|---|
| `exciter_envelope_shape` | AD/ADSR shape morphing (0=percussive AD, 0.5=ADSR, 1=long AD) |
| `exciter_bow_level/timbre` | Bow exciter amplitude and spectral content |
| `exciter_blow_level/meta/timbre` | Blow exciter amplitude, model selection, brightness |
| `exciter_strike_level/meta/timbre` | Strike exciter amplitude, model selection, brightness |
| `exciter_signature` | Per-unit random seed affecting sample selection and plectrum shape |
| `resonator_geometry` | Partial frequency spacing (negative = inharmonic, positive = stiff bar) |
| `resonator_brightness` | Q-loss rate across partials (high = brighter, longer HF decay) |
| `resonator_damping` | Overall modal Q (decay time) |
| `resonator_position` | Pickup/excitation position (comb-filter amplitude shaping via cosine) |
| `resonator_modulation_frequency/offset` | LFO rate and offset for stereo side-channel modulation |
| `reverb_diffusion` | Allpass coefficient in the Griesinger loop |
| `reverb_lp` | One-pole LP coefficient inside reverb loop |
| `space` | Meta-parameter: 0→dry, 0.1–0.8→stereo spread, 0.8–1.75→reverb mix, >1.75→freeze |

Five of these (`resonator_modulation_frequency`, `resonator_modulation_offset`,
`reverb_diffusion`, `reverb_lp`, `exciter_signature`) are seeded from the MCU serial number in
`Part::Seed()`, giving each unit a subtly unique character without user-accessible controls.

---

## Exciter Models (`dsp/exciter.cc`)

`Exciter` is a polymorphic signal generator dispatched through a static function-pointer table
`fn_table_[]`. Each instance (bow, blow, strike in `Voice`) is configured independently.

After the model-specific process function runs, a state-variable filter (`stmlib::Svf`) shapes the
spectrum according to `timbre_`. For `EXCITER_MODEL_NOISE` the filter is set with variable
resonance; for all others it runs as a fixed high-Q low-pass to control brightness.

**Bow** — `ProcessFlow`: Generates bandlimited noise with occasional polarity flips. A random
sample below a power-law threshold flips the sign of a running state, while the remaining output
interpolates toward white noise at a rate controlled by `parameter_`. This approximates the
turbulent, aperiodic quality of rosin-on-string friction. The bow exciter output is not summed
directly into the resonator input; instead it sets the `bow_strength_buffer_[]` which is passed
to `Resonator::Process()` to drive the embedded bowing table (see below).

**Blow** — `ProcessGranularSamplePlayer` (default): Reads a stored noise sample bank with a
random phase-restart probability, producing granular turbulence. `timbre_` controls playback
speed (and thus spectral content); `parameter_` (`exciter_blow_meta`) controls the restart point.
`exciter_signature` selects which 8192-sample window into `smp_noise_sample` to use. The blow
buffer is then passed through `Tube::Process()` for physical tube resonance, then through
`Diffuser::Process()` (4-stage allpass chain) to add spectral smearing. `blow_meta` also selects
among seven possible exciter models via `set_meta()`, spanning the enum from
`EXCITER_MODEL_GRANULAR_SAMPLE_PLAYER` through `EXCITER_MODEL_NOISE`.

**Strike** — meta-selectable from `EXCITER_MODEL_SAMPLE_PLAYER` through
`EXCITER_MODEL_PARTICLES`:
- `ProcessSamplePlayer`: Cross-fades between two adjacent stored percussive samples at speeds
  set by `timbre_`. Plays once on gate rising edge; applies a damping ramp on gate release.
  The `parameter_` value indexes into 9 sample slots (`smp_boundaries[]`).
- `ProcessMallet`: Emits a single amplitude-calibrated impulse (`GetPulseAmplitude(timbre_)`)
  on the rising gate edge, then applies a damping ramp via `parameter_`. The damping feeds back
  into the resonator's `damping` parameter as a palm-mute effect.
- `ProcessPlectrum`: Emits a negative impulse at gate onset, waits `parameter_²·4096 + 64`
  samples, then emits a positive impulse. The two-impulse sequence simulates the pick
  displacement and release of a plectrum.
- `ProcessParticles`: Models a bouncing particle: time between impulses is proportional to a
  random-walk state variable, producing irregular impulse trains that slow down over time
  (controlled by `parameter_` as a decay factor).

---

## Resonator — Modal Filter Bank (`dsp/resonator.cc`)

The modal resonator is the acoustic core of Elements. `ComputeFilters()` sets up up to 64
second-order SVF bandpass filters (`stmlib::Svf f_[kMaxModes]`).

**Partial frequencies**: Starting from the fundamental `frequency_`, each mode adds a
`stretch_factor` that is incremented by `stiffness` each step. `stiffness` is interpolated from
`lut_stiffness` using `geometry_`. Negative stiffness (negative geometry) produces inharmonic
partials converging toward equal spacing (bell/plate-like); positive stiffness produces
stretched partials (stiff bar/marimba-like); zero produces harmonic series.

**Q per mode**: A base Q of `500 × lut_4_decades[damping_ × 0.8]` (roughly four decades of
range) is multiplied per partial by `q_loss`, which itself is set from `brightness_` and allowed
to recover toward 1.0 at a rate `q_loss_damping_rate` determined by geometry. This makes low
partials decay faster than high ones at low brightness (absorptive body), or vice versa.

**Clock division**: To save CPU, only the first 24 modes are recomputed every block; higher modes
alternate on even/odd blocks via `clock_divider_`.

**Stereo synthesis via cosine amplitude shaping**: `Resonator::Process()` uses
`stmlib::CosineOscillator` to generate a cosine amplitude series indexed by `position_`. Each
mode's bandpass output is multiplied by successive cosine terms. The center channel uses
`position_` directly; the side channel uses `modulation_offset_ + lfo` (a triangle LFO at
`modulation_frequency_`). Taking the difference `sum_side - sum_center` gives the sides signal.
This is a frequency-domain comb filter approximation: modes near nulls of the cosine are
attenuated, simulating a pickup at a specific point on the resonator.

**Bowed waveguide modes**: The first 8 modes also have a parallel banded waveguide structure.
Each bowed mode uses a separate high-Q SVF (`f_bow_[]`) and a short delay line
(`d_bow_[]`, length = 1/partial_frequency, clamped to `kMaxDelayLineSize`). The feedback
signal from the delay lines is summed and passed through the `BowTable()` function:

```cpp
inline float BowTable(float x, float velocity) const {
    x = 0.13f * velocity - x;
    float bow = fabs(6.0f * x) + 0.75;
    bow = bow * bow * bow * bow;
    bow = 0.25f / bow;
    // clamp to [0.0025, 0.245]
    return x * bow;
}
```

This is a simplified friction model (related to McIntyre-Schumacher-Woodhouse bowed-string
tables): the output is approximately proportional to `x` near zero but saturates and rolls off,
modeling the stick-slip nonlinearity. The resulting `bow_signal_` is fed back into the bowed
mode input on the next sample, creating a self-sustaining oscillation when bow strength is
nonzero.

---

## Tube Model (`dsp/tube.cc`)

`Tube` is a simplified cylindrical-bore waveguide for wind-instrument modeling. It uses a single
delay line of length `1/frequency`, a one-zero reflection filter (`zero_state_`), and a naive
reed model:

```
pressure_delta = -0.95 * (delayed_signal * envelope + zero_state) - breath
reed = pressure_delta * -0.2 + 0.8
out = pressure_delta * reed + breath
```

The `breath` variable mixes the incoming blow exciter signal with a DC offset (0.8) that biases
the reed open. `lpf_coefficient` controls a one-pole LP (`pole_state_`) that attenuates high
frequencies in the tube output, scaled by `timbre²`. The tube output is mixed additively into
`blow_buffer_[]` at amplitude `tube_level * 0.5 * envelope`.

---

## String Model (`dsp/string.cc`)

`String` implements Karplus-Strong synthesis with dispersion. It uses:
- A 2048-sample delay line (`StringDelayLine`) for the main feedback loop
- A 1024-sample delay line (`StiffnessDelayLine`) for dispersion (allpass-based stiffness)
- `DampingFilter`: a 3-tap FIR with coefficients derived from `brightness_` for loop filtering
- An IIR SVF for damping at very high frequencies
- A DC blocker

`dispersion_` maps from `resonator_geometry` when in string mode; the geometry knob selects
a chord when `RESONATOR_MODEL_STRINGS` is active (5 voices tuned to one of 11 preset chords
defined in `voice.cc`).

---

## Reverb (`dsp/fx/reverb.h`, `dsp/fx/fx_engine.h`)

The reverb is a Griesinger plate topology from the Dattorro (1997) paper, implemented using
`FxEngine<32768, FORMAT_16_BIT>`. `FxEngine` is a compile-time delay-line memory allocator: the
`Reserve<N, Tail>` template chain computes buffer offsets at compile time, eliminating runtime
allocation overhead. Delay reads/writes are expressed as a DSP-context mini-language (`c.Read()`,
`c.WriteAllPass()`, `c.Interpolate()`, `c.Lp()`).

The topology: 4 allpass diffusers on the input → two-loop structure with modulated delays.
LFO modulation (`LFO_1` at 0.5 Hz, `LFO_2` at 0.3 Hz via cosine oscillators) is applied to
`ap1` (inside the input diffuser, for additional smearing) and `del2` (in the feedback loop,
for a slow shimmer/chorus effect).

The `space` metaparameter in `Part::Process()` controls:
- `raw_gain`: blends the pre-resonator signal directly into the output (dry click bleed)
- `spread`: scales the `sides_buffer_[]` stereo width
- `reverb_amount`: wet/dry mix (`reverb_.set_amount()`)
- `reverb_time`: feedback coefficient (0.35 + 1.2 × reverb_amount)
- At `space >= 1.75`: freeze mode — `input_gain = 0`, `time = 1.0`, `lp = 1.0` (infinite hold)

---

## Envelope and Gate Flags

`MultistageEnvelope` is a phase-accumulator envelope with per-segment shape (linear, exponential,
quartic) driven by a 257-entry lookup table. Gate events are communicated as bitmask flags
(`EXCITER_FLAG_RISING_EDGE`, `EXCITER_FLAG_FALLING_EDGE`, `EXCITER_FLAG_GATE`) shared between the
envelope and all three exciter instances, allowing coherent attack/release behavior across the
whole signal chain.

`Voice::Process()` derives ADSR parameters from `exciter_envelope_shape`:
- 0–0.4: percussive AD (short attack + decay, gain boosted up to 5×)
- 0.4–0.6: ADSR with variable sustain level
- 0.6–1.0: long attack/decay with full sustain

The envelope output scales blow and bow exciter amplitudes; strike uses it only for accent
(`strength` → `accent` lookup). Some exciters modulate resonator damping on release to simulate
palm-muting (mallet) or plectrum drag.

---

## Non-Obvious DSP Techniques

- **Asymmetric leaky integrator noise gate**: The input level estimator uses `0.1` for attack and
  `0.0001` for release, giving fast response to signal and very slow dropout, avoiding clicks on
  quiet sources.
- **Partial frequency domain comb via cosine amplitude modulation**: Rather than a time-domain
  delay-line comb filter (which would zipper badly when modulated), the pickup position effect is
  computed analytically per mode using successive cosine terms, providing smooth modulation.
- **Stiffness as a stretch-factor differential**: Each mode adds a changing increment to the
  stretch factor, not a fixed ratio. The stiffness value decays each step
  (`*= 0.93` or `*= 0.98`) ensuring partials do not diverge indefinitely while still producing
  the characteristic inharmonicity of real materials.
- **Per-unit personality via serial number seeding**: `Part::Seed()` derives 5 synthesis
  parameters (LFO frequency/offset, reverb diffusion/LP, exciter signature) from the MCU's
  96-bit unique ID using a linear congruential scrambler, making every unit subtly distinct.
- **`FxEngine` compile-time delay allocation**: All reverb delay-line sizes are embedded as
  template parameters and resolved to buffer offsets at compile time. No heap allocation; the
  32 KiB reverb buffer in CCM RAM is mapped at link time.
- **Panic recovery**: `Part::Process()` tracks a mean-square level of the output. If it exceeds
  200.0 (catastrophic filter blow-up, e.g. from NaN propagation), a `panic_` flag is set and
  all resonator SVF states are zeroed on the next block, preventing the module from locking up.
