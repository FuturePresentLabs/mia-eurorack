# Elements — Deep Technical Reference

Elements is a Eurorack module by Mutable Instruments implementing modal synthesis: a physical model
of a resonant object (plate, bar, membrane, string) being excited by bowing, blowing, or striking.
The synthesis engine is sample-accurate at 32 kHz and runs on an STM32F4 microcontroller.

---

## Directory / File Structure

```
elements/
  elements.cc          — main entry point, codec ISR, global objects
  cv_scaler.{h,cc}     — pot/CV reading, calibration, Patch/PerformanceState population
  ui.{h,cc}            — LED metering, button handling, factory test, resonator model switching
  meter.h              — integer peak follower (unused in final audio path)
  drivers/             — hardware peripherals (codec, ADCs, gate input, LEDs)
  dsp/
    patch.h            — plain-old-data parameter struct (Patch)
    part.{h,cc}        — top-level DSP coordinator; owns Voice, OminousVoice, Reverb
    voice.{h,cc}       — one synthesis voice: three Exciters + Tube + Resonator/String[5]
    exciter.{h,cc}     — seven excitation models dispatched via fn_table_[]
    resonator.{h,cc}   — modal filter bank: up to 64 SVF bandpass modes + 8 bowed waveguide modes
    tube.{h,cc}        — single-delay-line cylindrical waveguide with reed nonlinearity
    string.{h,cc}      — Karplus-Strong with allpass dispersion and 3-tap FIR damping
    multistage_envelope.{h,cc} — multi-segment ADSR with per-segment shape and looping
    ominous_voice.{h,cc} — easter-egg alternate voice: 2-operator FM synth with 8x oversampling
    dsp.h              — shared constants: kSampleRate (32000.0f), kMaxBlockSize (16)
    fx/
      fx_engine.h      — compile-time delay-line memory allocator + DSP accumulator context
      diffuser.h       — 4-allpass blow-buffer diffuser (126/180/269/444 sample taps)
      reverb.h         — Griesinger plate reverb (Dattorro 1997 topology)
```

---

## Signal Chain Overview

```
Codec DMA (16-bit, 32 kHz, stereo)
     |
  FillBuffer()
     |  input[i].r → blow_in[]   (noise-gated)
     |  input[i].l → strike_in[] (noise-gated)
     |
  CvScaler::Read()  →  Patch + PerformanceState
     |
  Part::Process()
     |
  Voice::Process()
     |
  ┌──────────────────────────────────────────────────────────────────┐
  │  bow_    (EXCITER_MODEL_FLOW)  →  bow_buffer_[]                  │
  │  blow_   (EXCITER_MODEL_GRANULAR_SAMPLE_PLAYER) → blow_buffer_[] │
  │     └→ Tube::Process() [additive] → blow_buffer_[]               │
  │     └→ Diffuser::Process() [in-place 4-allpass smear]            │
  │     └→ blow_in[] mixed in                                        │
  │  strike_ (meta-selectable, 4 models) → strike_buffer_[]         │
  │     └→ strike_in[] mixed in                                      │
  │                                                                  │
  │  Summed into raw_buffer[] per-sample:                            │
  │    bow_buffer * bow_strength * 0.125 * accent                    │
  │    + blow_buffer * envelope * accent                             │
  │    + strike_buffer * strike_level                                │
  │    + strike_in                                                   │
  └──────────────────────────────┬───────────────────────────────────┘
                                 │
              Resonator::Process()    ← bow_strength_buffer_[] fed separately
              (64 SVF bandpass modes + 8 bowed waveguide modes)
              → center_buffer[], sides_buffer[]
                                 │
  Part: sides * spread, L = center + side, R = center - side
  Part: SoftLimit(L), SoftLimit(R)
  Part: Reverb::Process(main[], aux[])
                                 │
  Codec output (output[i].r = main, output[i].l = aux)
```

---

## Hardware I/O Pipeline (`elements.cc`)

`FillBuffer` is registered as the codec DMA interrupt callback for 16-bit stereo frames at 32 kHz.
The nominal block size is 16 samples (`kMaxBlockSize`), giving a 0.5 ms interrupt period.

Audio routing: right channel = `blow_in`, left channel = `strike_in`. Both are filtered by an
asymmetric leaky-integrator noise gate:

```cpp
error = sample * sample - level;
level += error * (error > 0.0f ? 0.1f : 0.0001f);
gain = level <= kNoiseGateThreshold
    ? (1.0f / kNoiseGateThreshold) * level : 1.0f;
```

`kNoiseGateThreshold = 0.0001f`. Attack time constant is 0.1, release is 0.0001 (1000x slower).
Below threshold, the gain is linearly ramped to zero — a soft gate, not a hard cut — suppressing the
codec noise floor without causing clicks on quiet but real sources.

The serial number seeding call is `part.Seed((uint32_t*)(0x1fff7a10), 3)`, which reads 12 bytes
(96 bits) from the STM32F4's hardwired unique device identifier memory region. The address
`0x1fff7a10` is the documented base of the STM32F4 UID registers.

The reverb buffer `uint16_t reverb_buffer[32768]` is placed in CCM RAM via
`__attribute__((section(".ccmdata")))`. CCM is a 64 KiB tightly-coupled data memory on the
STM32F4 that is on a dedicated bus with zero wait-states and does not contend with the main SRAM
bus used by audio sample buffers. This is a deliberate choice to keep reverb reads/writes off the
critical DMA path for main audio.

---

## Code Architecture

### 1. Part, Voice, and the Ownership Model

`Part` (in `dsp/part.h`) is the top-level DSP object. It owns:
- `Voice voice_[kNumVoices]` (currently `kNumVoices = 1`, though the code notes polyphony is
  possible if modes are reduced to 16)
- `OminousVoice ominous_voice_[kNumVoices]` (the easter-egg synth)
- `Reverb reverb_`
- Working buffers: `raw_buffer_[kMaxBlockSize]`, `center_buffer_[]`, `sides_buffer_[]`
- Metering state: `resonator_level_`, `scaled_exciter_level_`, `scaled_resonator_level_`

`Voice` (in `dsp/voice.h`) owns all synthesis objects for one voice:
- `MultistageEnvelope envelope_`
- `Exciter bow_`, `blow_`, `strike_`
- `Tube tube_`
- `Diffuser diffuser_`
- `Resonator resonator_`
- `String string_[kNumStrings]` — five strings for chord mode
- `stmlib::DCBlocker dc_blocker_`
- Local work buffers: `bow_buffer_`, `bow_strength_buffer_`, `blow_buffer_`, `strike_buffer_`
- `float diffuser_buffer_[1024]` — static buffer for the Diffuser's FxEngine

`Part::Process()` calls `Voice::Process()`, which writes into three shared Part buffers: `raw`,
`center`, `sides`. Part then applies stereo spread, soft-limiting, and reverb.

The `active_voice_` counter advances on each rising gate edge, cycling through voices round-robin.
With only one voice this is a no-op, but the infrastructure is complete.

---

### 2. CV Scaling and Parameter Mapping (`cv_scaler.cc`)

`CvScaler` manages 27 panel potentiometers (via `PotsAdc`, scanned one per call round-robin through
`pots_.Scan()`) and 7 CV inputs (via `CvAdc`, converted each call). Pot values pass through one
of four transfer laws before entering a global one-pole LP filter (`0.01` coefficient for all pots):

- `LAW_LINEAR`: `value = raw / 65536.0f`
- `LAW_QUADRATIC_BIPOLAR`: `x = raw/32768 - 1; value = sign(x) * x^2 * 3.3`. Attenuverter pots
  use this — the quadratic law concentrates resolution near center (zero gain).
- `LAW_QUARTIC_BIPOLAR`: same but `x^4 * 3.3`. Used for the FM attenuverter for even finer
  near-zero control.
- `LAW_QUANTIZED_NOTE`: `note = 60 * raw/65536` (5-octave range), half-sample LP filtered,
  then integer quantization with `±0.3` semitone hysteresis. Used for `POT_RESONATOR_COARSE`.

The `BIND` macro (the core of `CvScaler::Read()`) combines pot and CV:

```cpp
float pot = pot_lp_[POT_##NAME];
float cv  = offset[CV_ADC_##NAME] - cv_.float_value(CV_ADC_##NAME);
float value = pot + pot_lp_[POT_##NAME##_ATTENUVERTER] * cv;
// slew-limited write to destination
destination += (value - destination) * lp_coefficient;
```

The CV offset is subtracted from the calibrated zero — meaning positive voltage deflects the pot
value upward. The attenuverter pot (using `LAW_QUADRATIC_BIPOLAR`) multiplies the CV before adding.
The `lp_coefficient` parameter controls slew: most parameters use `1.0` (direct) except for
geometry (`0.05`), blow/strike meta (`0.05`), position (`0.01`), and space (`0.01`) — the slowest
slewed parameters. Position gets `0.01` because it modulates the modal amplitude cosine oscillator;
even a small step causes a discontinuity in the bank sum, so it must be ramped over many blocks.

Pitch CV uses a two-point calibration: `pitch = pitch_offset + cv * pitch_scale`. The coarse knob
adds a quantized 0–60 semitone offset (plus a constant `+19.0` MIDI offset). The fine knob adds
`±(2/3.3)` semitones. V/oct jumps larger than 0.4 semitones bypass the 10% LP slew to avoid
pitch lag on hard CV transitions.

Frequency conversion uses a 16.8 fixed-point two-table product:
```cpp
int32_t pitch = static_cast<int32_t>((midi_pitch + 48.0f) * 256.0f);
float freq = lut_midi_to_f_high[pitch >> 8] * lut_midi_to_f_low[pitch & 0xff];
```
`lut_midi_to_f_high` is indexed by the integer MIDI note, `lut_midi_to_f_low` by the fractional
eighth-semitone. Their product is the normalized frequency (cycles/sample at 32 kHz).

`PerformanceState::strength` is `1.0 - cv_.float_value(CV_ADC_PRESSURE)` — the PRESSURE CV input
is inverted so that higher voltage = more strength.

---

### 3. Envelope (`dsp/multistage_envelope.h`)

`MultistageEnvelope` is a phase-accumulator envelope with up to 6 segments. Gate flags are a 3-bit
bitmask: `EXCITER_FLAG_RISING_EDGE`, `EXCITER_FLAG_FALLING_EDGE`, `EXCITER_FLAG_GATE`. On each
call to `Process(flags)`, the current segment phase advances by `Interpolate8(lut_env_increments,
time_[segment_])`. The output value is `start_value + (target - start_value) * t` where `t` is
looked up from per-segment shape tables:

- `ENV_SHAPE_QUARTIC` (attack): `t = phase^4`, very fast initial attack
- `ENV_SHAPE_EXPONENTIAL` (decay/release): pseudo-exponential via a 257-entry table
- `ENV_SHAPE_LINEAR`: used in several alternate modes

`Voice::Process()` derives ADSR from `exciter_envelope_shape`:
- `shape < 0.4`: AD mode, `attack = shape*0.75+0.15`, `decay = attack*1.8`, sustain=0,
  `envelope_gain = 5 - shape*10` (up to 5× gain boost for very short percussive bursts)
- `0.4 <= shape < 0.6`: ADSR with fixed attack=0.45, decay=0.81, sustain `= (shape-0.4)*5`
- `shape >= 0.6`: long AD mode, `attack = (1-shape)*0.75+0.15`, sustain=1.0 (infinite hold)

The envelope output gates blow and bow levels. Strike uses it only for the `accent` lookup.

---

### 4. Exciter Models (`dsp/exciter.cc`)

`Exciter` dispatches through `fn_table_[]` (a static pointer-to-member table, indexed by
`ExciterModel` enum):

```
[0] ProcessGranularSamplePlayer  — blow default
[1] ProcessSamplePlayer          — strike percussive samples
[2] ProcessMallet                — strike impulse
[3] ProcessPlectrum              — strike two-impulse pick model
[4] ProcessParticles             — strike bouncing particle impulse train
[5] ProcessFlow                  — bow noise model
[6] ProcessNoise                 — filtered white noise
```

After the model-specific function runs, `Exciter::Process()` applies a spectral shaping filter
(an `stmlib::Svf`) to the output buffer. For `EXCITER_MODEL_NOISE`, the SVF is configured with
variable resonance from `parameter_`. For all other models (except the two sample players which
skip filtering entirely), it is set as a high-Q low-pass controlled by `timbre_` using
`lut_approx_svf_g[cutoff_index]` and a fixed R of 2.0. The `lut_approx_svf_h` table provides a
high-pass compensation gain that keeps overall level constant as the cutoff sweeps.

**Bow — `ProcessFlow`**: Generates low-bandwidth colored noise with occasional polarity flips.
The power-law threshold `threshold = 0.0001 + (parameter_^4) * 0.125` gates random sign flips of
a running state variable `particle_state_`. Each sample output is:

```cpp
float sample = RandomSample();   // uniform [0, 1)
if (sample < threshold) particle_state_ = -particle_state_;
*out++ = particle_state_ + (sample - 0.5f - particle_state_) * scale;
```

where `scale = parameter_^4`. At low `parameter_`, threshold is tiny (rare flips) and scale is
near zero, producing nearly DC. At high `parameter_`, frequent flips and higher `scale` produce
something closer to white noise. The bow exciter signal goes to `bow_buffer_[]` but is NOT summed
directly into the resonator input `raw[]`. Instead it is used to populate `bow_strength_buffer_[]`
(see below under Resonator).

**Blow — `ProcessGranularSamplePlayer`**: Reads from `smp_noise_sample` (40,963 samples of stored
noise), a window selected by `signature_ * 8192` samples into the table. Playback phase increments
by `131072 * SemitonesToRatio(72*timbre - 60)` per sample — sweeping `timbre` from 0 to 1 varies
speed from -60 to +12 semitones relative to a 4× playback base rate. On each sample, a
`RandomSample() < 0.01` probability (1% per sample at 32 kHz = about 320 resets/second average)
resets the phase to `restart_point = parameter_ * 32767 << 17`, implementing granular re-triggering.
The `parameter_` (`exciter_blow_meta`) also selects among all seven `ExciterModel` values via
`set_meta()`, so at `blow_meta > ~0.14` the blow exciter transitions out of granular playback into
the other models. The blow buffer then passes through Tube (additive) and Diffuser (in-place).

The actual mixing of `blow_in[]` into the blow path happens in `Voice::Process()`:
```cpp
blow_level = patch.exciter_blow_level * 1.5f;
tube_level = blow_level > 1.0f ? (blow_level - 1.0f) * 2.0f : 0.0f;
blow_level = blow_level < 1.0f ? blow_level * 0.4f : 0.4f;
// ...
blow_buffer_[i] = blow_buffer_[i] * blow_level + blow_in[i];
```

So `exciter_blow_level` simultaneously scales synthesized blow and, when > 0.667, activates the
Tube feedback path. The external blow signal is mixed in at unity after both the synthesized blow
and Tube contributions.

**Blow meta (`exciter_blow_meta`) model selection**: The `set_meta()` call maps 0–1 linearly onto
all seven exciter model indices:

```cpp
meta *= static_cast<float>(last - first + 1);  // 7 in the blow case
model_ = static_cast<ExciterModel>(first + meta_integral);
parameter_ = meta_fractional;
```

For blow, `first = EXCITER_MODEL_GRANULAR_SAMPLE_PLAYER`, `last = EXCITER_MODEL_NOISE`. The
`parameter_` passed to the model becomes the within-model fractional value of `meta`.

**Strike — meta-selectable from `EXCITER_MODEL_SAMPLE_PLAYER` through `EXCITER_MODEL_PARTICLES`**:
The meta mapping uses a bent transfer: values `<= 0.4` are scaled by `0.625` (emphasizing the
sample player range), values `> 0.4` are scaled by `1.25 - 0.25` (emphasizing mallets/particles).

- `ProcessSamplePlayer`: Cross-fades between two adjacent stored percussive samples at indexes from
  `smp_boundaries[]` (9 slots, 128,013 total samples). Playback speed is
  `65536 * SemitonesToRatio(72*timbre - 36 + 7)` samples/sample. At rising edge, phase resets to 0.
  On gate release, `damp_state_` ramps toward 1 at rate `1 - 0.95*(1-damp)` per sample, and
  `damping_` is set to `damp * (parameter_ >= 0.8 ? parameter_*5-4 : 0)`. This `damping_` value
  is fed back into the resonator's effective `damping` parameter in `Voice::Process()` to create a
  palm-mute effect on note release.

- `ProcessMallet`: Emits a single impulse of amplitude `GetPulseAmplitude(timbre_)` at the rising
  gate edge, then zeros output. `GetPulseAmplitude` looks up `lut_approx_svf_gain[cutoff_index]`
  — this is the gain of a unit impulse through the post-exciter SVF, so the impulse amplitude is
  pre-compensated to produce consistent energy into the resonator regardless of timbre (brightness).
  On release, `damping_ = damp_state * (1 - parameter_)` where `parameter_` is the
  TIMBRE-derived within-model meta value. Higher `parameter_` = less palm muting.

- `ProcessPlectrum`: On rising edge, emits `impulse = -amplitude * (0.05 + signature*0.2)` (a
  negative pluck) and sets `plectrum_delay_ = 4096 * parameter_^2 + 64` samples. During the
  delay, damping ramps: `damp = 1 - 0.997*(1-damp)` per sample (very slow ramping). When delay
  expires, `impulse = +amplitude` (positive release). After release, `damp *= 0.9` per sample.
  `damping_ = damp * 0.5` bleeds into resonator damping throughout. The `signature_` field
  randomizes the initial negative impulse amplitude per unit, making each module's plectrum
  slightly different.

- `ProcessParticles`: Models a bouncing particle. On gate rising edge, `particle_state_` is set
  to `1 - 0.6*r^2` (r from `RandomSample()`), clamping to a near-1 initial bounce height.
  `particle_range_` tracks the shrinking bounce envelope and decays each bounce by:
  `particle_range_ *= 1 - (1-parameter_)^2 * 0.5`.
  The inter-impulse delay is `particle_state_ * 0.15 * kSampleRate` samples. At each bounce, the
  state undergoes a random walk: 70% probability to multiply by a random `1.05 + 0.5*r^2`
  (bounce higher), 30% probability to divide (bounce lower), clamped to `[0.02, particle_range_+0.25]`.
  Impulse amplitude = `particle_state_ * GetPulseAmplitude(timbre) * (1 - gain^2)` where
  `gain = 1 - particle_range_`. This produces an impulse train that becomes more irregular and
  quieter as `particle_range_` decays.

All four strike models feed `strike_in[]` directly into the summed resonator input, bypassing the
envelope:
```cpp
input_sample += strike_in[i];  // external IN is always present at unity
```

---

### 5. Tube Waveguide (`dsp/tube.cc`)

`Tube` is a simplified cylindrical-bore waveguide model for wind-instrument resonance. It is
applied additively to `blow_buffer_[]` — it does not replace the blow signal but augments it.

The waveguide uses a single delay line `delay_line_[kTubeDelaySize]` where `kTubeDelaySize = 2048`
samples. The delay length is `1/frequency` (samples), halved repeatedly until it fits the buffer.
Interpolated delay reads use linear interpolation between adjacent samples.

Per-sample processing:

```cpp
float breath = *input_output * damping + 0.8f;
// damping maps patch.resonator_damping → [3.6, 1.8]:
// damping_coeff = 3.6 - patch_damping * 1.8

float in = interpolated_delay_read(delay_integral, delay_fractional);
float pressure_delta = -0.95f * (in * envelope + zero_state_) - breath;
zero_state_ = in;

float reed = pressure_delta * -0.2f + 0.8f;
float out = pressure_delta * reed + breath;

CONSTRAIN(out, -5.0f, 5.0f);
delay_line_[d] = out * 0.5f;

pole_state_ += lpf_coefficient * (out - pole_state_);
*input_output += gain * envelope * pole_state_;
```

The `zero_state_` term is a one-zero reflection filter — it creates the closed-end reflection at
the far end of the tube. `pressure_delta` approximates the reed differential pressure. The reed
transfer function `reed = -0.2*pressure_delta + 0.8` gives a simple linear model: at rest bias
(`pressure_delta ≈ 0`), reed value = 0.8 (open); large positive `pressure_delta` closes the reed.
`out = pressure_delta * reed + breath` is the pressure-reed product plus the driving breath, which
is a naive but computationally cheap approximation of reed excitation.

The `lpf_coefficient = frequency * (1 + timbre^2 * 256)` — at high timbre, the one-pole LP inside
the tube has a cutoff approaching Nyquist (pass-through); at low timbre, the tube output is
heavily low-passed.

The Tube output is mixed into `blow_buffer_` only when `tube_level > 0`, which occurs only when
`exciter_blow_level > 0.667`. The mix amplitude is `tube_level * 0.5 * envelope`.

---

### 6. Blow Diffuser (`dsp/fx/diffuser.h`)

After Tube processing, `blow_buffer_` passes through a 4-stage series allpass chain with delay
lengths 126, 180, 269, and 444 samples. All four stages use the same feedback coefficient
`kap = 0.625f`. The `FxEngine<1024, FORMAT_32_BIT>` backing store uses 32-bit float samples
(no quantization). The allpass equation implemented by `c.WriteAllPass(ap, -kap)` is:

```
y[n] = -kap * x[n] + delayed_x + kap * delayed_y
```

(Schroeder allpass). This adds spectral diffusion to the blow signal — the turbulent noise
becomes more colorless and uniform before entering the resonator, which prevents individual
noise partials from exciting resonator modes in a tonally biased way.

---

### 7. Resonator Modal Filter Bank (`dsp/resonator.cc`)

The resonator is the acoustic core. `ComputeFilters()` sets up up to 64 second-order SVF bandpass
filters (`stmlib::Svf f_[kMaxModes]`), called once per block. `Voice::ResetResonator()` calls
`resonator_.set_resolution(52)` — the practical limit before CPU is exhausted.

**Partial frequency computation**: The outer loop accumulates `partial_frequency = harmonic *
stretch_factor`. Starting with `harmonic = frequency_` (fundamental) and `stretch_factor = 1.0`,
each partial adds the current `stiffness` value to `stretch_factor` before multiplying:

```cpp
float stiffness = Interpolate(lut_stiffness, geometry_, 256.0f);
float harmonic = frequency_;
float stretch_factor = 1.0f;
// ...
for (size_t i = 0; i < min(kMaxModes, resolution_); ++i) {
    float partial_frequency = harmonic * stretch_factor;
    // ...
    stretch_factor += stiffness;
    if (stiffness < 0.0f) stiffness *= 0.93f;
    else                   stiffness *= 0.98f;
    harmonic += frequency_;
}
```

The `lut_stiffness` table maps `geometry_` (0 to 1) to stiffness values: negative at low geometry
(bell/plate inharmonicity — modes converge toward equal spacing), zero at center (pure harmonic
series), positive at high geometry (stiff bar — modes stretch outward like a marimba). The
stiffness decays each step (`*0.93` for negative, `*0.98` for positive) to prevent infinite
divergence at high mode counts.

For example: at `geometry = 0` with `stiffness = -0.3`, mode n has frequency approximately
`n * f0 + n*(n-1)/2 * (-0.3) * f0 * (0.93^n)`, converging toward equal frequency spacing
rather than harmonic multiples. This produces a bell-like inharmonic spectrum.

**Q per mode**: The base Q is `500 * Interpolate(lut_4_decades, damping_ * 0.8, 256.0f)`. The
`lut_4_decades` table spans four decades (10^0 to 10^4) logarithmically, giving the Q a range of
roughly 500 to 5,000,000. The `damping_ * 0.8` limiting means the highest damping only reaches
80% of the table, preserving some decay even at maximum. Per-mode Q is then multiplied by
`q_loss` after each partial:

```cpp
float brightness_attenuation = 1.0f - geometry_;
brightness_attenuation = pow8(brightness_attenuation);  // 8th power squashing
float brightness = brightness_ * (1.0f - 0.2f * brightness_attenuation);
float q_loss = brightness * (2.0f - brightness) * 0.85f + 0.15f;
float q_loss_damping_rate = geometry_ * (2.0f - geometry_) * 0.1f;
// per mode:
q *= q_loss;
q_loss += q_loss_damping_rate * (1.0f - q_loss);
```

`q_loss` is a per-mode Q multiplier: `< 1` means upper modes decay faster (absorptive body,
low brightness), `= 1` means all modes decay equally. `q_loss_damping_rate` causes `q_loss`
to recover toward 1.0 at a rate proportional to `geometry_`, meaning high-geometry resonators
(stiff bars) have more uniform decay across modes.

**SVF filter setup**: Each mode uses `f_[i].set_f_q<FREQUENCY_FAST>(partial_frequency, 1 +
partial_frequency * q)`. This is a normalized state-variable filter with the denominator
`1 + f/Q` — the second argument is the actual denominator coefficient, not the standard Q.
The `partial_frequency * q` term means Q scales with frequency, which approximates constant
bandwidth rather than constant Q.

**Clock division for CPU efficiency**: Modes 0–24 are recomputed every block. Modes 25–63
alternate on even/odd blocks via `(i & 1) == (clock_divider_ & 1)`. The filter coefficients
for the skipped modes simply persist from the previous block — acceptable because high modes
change frequency slowly, and their shorter decay means any transient error disappears quickly.

**SVF processing**: Each sample processes all `num_modes` filters in the inner loop:

```cpp
float input = *in++ * 0.125f;   // 12 dB input attenuator
float sum_center = 0.0f;
float sum_side = 0.0f;
amplitudes.Start();
aux_amplitudes.Start();
for (size_t i = 0; i < num_modes; i++) {
    s = f_[i].Process<FILTER_MODE_BAND_PASS>(input);
    sum_center += s * amplitudes.Next();
    sum_side += s * aux_amplitudes.Next();
}
*sides++ = sum_side - sum_center;
```

The `0.125f` scaling prevents accumulation overflow across 64 resonant modes.

---

### 8. Stereo Field and the Cosine Amplitude Comb (`dsp/resonator.cc`)

The stereo effect is a frequency-domain approximation of pickup-position comb filtering.
`CosineOscillator amplitudes` and `aux_amplitudes` are initialized each sample with:

- `amplitudes.Init(previous_position_)` — the POSITION parameter
- `aux_amplitudes.Init(modulation_offset_ + lfo)` — position offset modulated by a triangle LFO

`CosineOscillator` produces the sequence `cos(pi * k * x)` for k = 0, 1, 2, ... via a recurrence
relation (computed with `COSINE_OSCILLATOR_APPROXIMATE`). Mode k is multiplied by `cos(pi * k *
position)` for the center channel and `cos(pi * k * offset)` for the side channel. The output is:

```
sum_center = sum_k (mode_k_output * cos(pi * k * position))
sum_side   = sum_k (mode_k_output * cos(pi * k * offset))
sides = sum_side - sum_center
```

The difference `sum_side - sum_center` is the sides signal. When `offset != position`, different
modes are attenuated differently in the two channels, creating a stereo image without any time-
domain delay or allpass structure — no flanger-like artifacts when modulated. The comment in the
code explicitly notes: "The correct way of simulating the effect of a pickup is to use a comb
filter. But it sounds very flange-y when modulated, even mildly, and incur a slight delay/smearing
of the attacks. Thus, we directly apply the comb filter in the frequency domain."

The LFO is a triangle wave: `lfo = phase > 0.5 ? 1 - phase : phase`, with rate
`modulation_frequency_` (a hardware-unit-specific value seeded from the serial number, in the
range `[0.4, 1.2] / 32000.0` Hz). It is added to `modulation_offset_` when computing
`aux_amplitudes`. The LFO subtly shifts which modes are attenuated in the side channel,
creating slow stereo movement.

`Part::Process()` then reconstructs L/R:
```cpp
float side = sides_buffer_[j] * spread;
float r = center_buffer_[j] - side;
float l = center_buffer_[j] + side;
```

`spread` is derived from `space`: zero at `space <= 0.1`, reaching maximum 0.7 at `space >= 0.8`.

---

### 9. Bowed Waveguide Modes (`dsp/resonator.cc`)

Alongside the 64 bandpass modes, the first 8 modes also drive a parallel banded waveguide
structure. For each bowed mode `i`:

- `d_bow_[i]` is a delay line of length `round(1 / partial_frequency)` samples, clamped to
  `kMaxDelayLineSize = 1024`. If the period exceeds 1024, it is halved until it fits.
- `f_bow_[i]` is a high-Q SVF (`f_bow_[i].set_g_q(f_[i].g(), 1 + partial_frequency * 1500)`)
  — tuned to the same frequency but with much higher Q (by factor ~3).

In the per-sample loop, after the normal mode sum:

```cpp
float bow_signal = 0.0f;
input += bow_signal_;    // feed last cycle's bow nonlinearity back into exciter
amplitudes.Start();
for (size_t i = 0; i < num_banded_wg; ++i) {
    s = 0.99f * d_bow_[i].Read();    // 0.99 = slight energy loss per cycle
    bow_signal += s;
    s = f_bow_[i].Process<FILTER_MODE_BAND_PASS_NORMALIZED>(input + s);
    d_bow_[i].Write(s);
    sum_center += s * amplitudes.Next() * 8.0f;
}
bow_signal_ = BowTable(bow_signal, *bow_strength++);
```

The `BowTable` function implements a simplified friction nonlinearity (derived from the McIntyre-
Schumacher-Woodhouse bowed-string model):

```cpp
inline float BowTable(float x, float velocity) const {
    x = 0.13f * velocity - x;
    float bow = x;
    bow *= 6.0f;
    bow = fabs(bow) + 0.75f;
    bow *= bow;     // bow^2
    bow *= bow;     // bow^4
    bow = 0.25f / bow;
    if (bow < 0.0025f) bow = 0.0025f;
    if (bow > 0.245f) bow = 0.245f;
    return x * bow;
}
```

`velocity` is `bow_strength_buffer_[i]` (from `Voice::Process()`: `e * patch.exciter_bow_level`).
`x` is the summed delay-line feedback, representing the relative velocity at the string/bow contact
point. The expression `0.13 * velocity - x` is the slip velocity. The denominator `(|6x| + 0.75)^4`
is a sharp notch around zero slip — near zero, `bow` is close to `0.25 / 0.75^4 ≈ 0.79`, so the
output ≈ `x * 0.79` (stick regime). At large slip `|x|`, the denominator grows as `|x|^4`, the
output falls toward zero (slip regime). The clamping to `[0.0025, 0.245]` prevents numerical
instability. The `0.99` coefficient on the delay line read is the only per-cycle energy loss;
the bow force sustains the oscillation when `velocity > 0`.

The crucial architectural point: `bow_signal_` from the previous sample is fed back into `input`
before the bowed modes process. This creates a closed feedback loop: delay line → SVF filter →
delay line read → BowTable → back into the combined input — a self-sustaining oscillation when bow
strength is sufficient. The bowed modes contribute to `sum_center` with an extra ×8 gain to bring
them to a level comparable to the regular modes.

`bow_strength_buffer_[i]` in `Voice::Process()` is set to `envelope_value_ * patch.exciter_bow_level`.
A side effect: `damping` is also reduced by `(1 - bow_strength_buffer_[0]) * bow_level * 0.0625`,
modeling the physical effect of the bow pressing on the string/object and providing additional
mechanical damping when the bow barely touches.

---

### 10. String Model (`dsp/string.cc`)

When `resonator_model_ = RESONATOR_MODEL_STRING` or `RESONATOR_MODEL_STRINGS`, `Voice::Process()`
routes audio to `String::ProcessInternal<enable_dispersion>()` instead of the modal resonator.

`String` implements Karplus-Strong with the following features:

**Delay line**: `StringDelayLine string_` — 2048-sample circular buffer read with Hermite
(cubic) interpolation for sub-sample accuracy.

**Dispersion (inharmonicity)**: When `enable_dispersion = true` (set in `Voice::ResetResonator()`),
an additional `StiffnessDelayLine stretch_` (1024 samples) and an allpass section implement
string stiffness. The `set_dispersion()` method maps the raw `dispersion_` value through a
piecewise function that has a dead zone around `[0.24, 0.26]` and negative values for `< 0.24`,
positive for `> 0.26`. Negative dispersion uses a `curved_bridge_` nonlinearity and DC blocking.
Positive dispersion uses an allpass at `ap_delay = delay * stretch_point` samples:

```cpp
float ap_gain = -0.618f * dispersion / (0.15f + fabs(dispersion));
s = string_.ReadHermite(main_delay);
s = stretch_.Allpass(s, ap_delay, ap_gain);
```

The allpass gain asymptotes toward `-0.618f` (≈ `-1/phi`, golden ratio reciprocal), which is a
common choice in digital waveguide dispersion filters for stable spectral stretching.

**Damping**: A 3-tap FIR `DampingFilter` provides loop filtering:

```cpp
float h0 = (1 + brightness) * 0.5f;
float h1 = (1 - brightness) * 0.25f;
y = damping * (h0 * x_ + h1 * (x + x__));
```

This is a 3-tap symmetric FIR with response `H(z) = damping * [h1 + h0*z^-1 + h1*z^-2]`. At
`brightness = 1`: `h0 = 1, h1 = 0` — pure delay (no HF loss). At `brightness = 0`: `h0 = 0.5,
h1 = 0.25` — a low-pass averaging filter. The overall gain is the `damping` coefficient itself.
An additional IIR SVF (`iir_damping_filter_`) low-passes at a frequency computed from damping and
brightness, and `lut_svf_shift` compensates for the group delay introduced by the IIR stage.

**Chord mode**: `RESONATOR_MODEL_STRINGS` instantiates 5 `String` objects. The `resonator_geometry`
knob selects among 11 preset chord tables defined in `voice.cc`:

```cpp
float chords[11][5] = {
    { 0.0f, -12.0f, 0.0f, 0.01f, 12.0f },   // octave/unison
    { 0.0f, -12.0f, 3.0f, 7.0f,  10.0f },   // minor 7th
    { 0.0f, -12.0f, 3.0f, 7.0f,  12.0f },   // minor add octave
    { 0.0f, -12.0f, 3.0f, 7.0f,  14.0f },   // minor 9th
    { 0.0f, -12.0f, 3.0f, 7.0f,  17.0f },   // minor 11th
    { 0.0f, -12.0f, 7.0f, 12.0f, 19.0f },   // power + octave
    { 0.0f, -12.0f, 4.0f, 7.0f,  17.0f },   // major 11th
    { 0.0f, -12.0f, 4.0f, 7.0f,  14.0f },   // major 9th
    { 0.0f, -12.0f, 4.0f, 7.0f,  12.0f },   // major add octave
    { 0.0f, -12.0f, 4.0f, 7.0f,  11.0f },   // major 7th
    { 0.0f, -12.0f, 5.0f, 7.0f,  12.0f },   // sus4 add octave
};
```

Selection uses hysteresis (`±0.1` on a 0–10 float): `float chord = geometry * 10; int chord_index
= round(chord + hysteresis)`. Each string is tuned by `SemitonesToRatio(chords[chord_index][i])`.
Note the `0.01` semitone detune in chord 0 to prevent perfect unison beating.

In chord mode, the `String::Process()` output is stored in `center[]` and `sides[]` with
semantics inverted from the normal resonator: the code swaps `left = center[i]; right = sides[i];
center[i] = left - right; sides[i] = left + right;` before returning. This is because
`String::Process()` accumulates the main output (at full delay position) into `out` and the
comb-position output into `aux`, and Part expects center/sides format.

---

### 11. Reverb (`dsp/fx/reverb.h`, `dsp/fx/fx_engine.h`)

The reverb implements the Griesinger plate topology from Dattorro (1997) using
`FxEngine<32768, FORMAT_16_BIT>`. The 32,768-element `uint16_t` buffer is the `reverb_buffer[]`
allocated in CCM RAM in `elements.cc`.

**`FxEngine` compile-time allocation**: The `Reserve<N, Tail>` template chain computes each delay
line's base address and length at compile time:

```cpp
template<typename Memory, int32_t index>
struct DelayLine {
    enum {
        length = DelayLine<typename Memory::Tail, index - 1>::length,
        base   = DelayLine<Memory, index - 1>::base +
                 DelayLine<Memory, index - 1>::length + 1
    };
};
```

The `+1` per delay line ensures consecutive lines do not alias. The `FORMAT_16_BIT` template
parameter means all delay samples are compressed to `uint16_t` (signed 16-bit via cast), scaling
by `1/32768`. This halves the memory footprint but introduces quantization noise on quiet reverb
tails — acceptable given the CCM size constraint.

**Delay line sizes and topology**:

```
ap1: 150   ap2: 214   ap3: 319   ap4: 527      (input diffuser)
dap1a: 2182  dap1b: 2690  del1: 4501           (loop 1)
dap2a: 2525  dap2b: 2197  del2: 6312           (loop 2)
```

Total: 21,617 samples = ~675 ms of delay at 32 kHz.

**Topology** (Griesinger/Dattorro plate):

```
stereo input (L+R)
    ↓
4 series allpass diffusers (ap1→ap2→ap3→ap4), coefficient = diffusion_
    ↓ apout
    ┌──────────────────────────────────────────┐
    │  Load apout                              │
    │  + del2 read (LFO_2 modulated, ×krt)    │ ← loop 1 starts here
    │  Lp(lp_1, klp)                          │
    │  WriteAllPass dap1a (-kap/kap)          │
    │  WriteAllPass dap1b (kap/-kap)          │
    │  Write del1 (×2.0)                      │
    │  Tap → left output                      │
    └──────────────────────────────────────────┘
    ┌──────────────────────────────────────────┐
    │  Load apout                              │
    │  + del1 read (×krt)                     │ ← loop 2
    │  Lp(lp_2, klp)                          │
    │  WriteAllPass dap2a (kap/-kap)          │
    │  WriteAllPass dap2b (-kap/kap)          │
    │  Write del2 (×2.0)                      │
    │  Tap → right output                     │
    └──────────────────────────────────────────┘
```

The `×2.0` write gain compensates for the `×krt` read gain in the cross-coupled feedback, keeping
unity loop gain at `krt = 0.5`. Two LFOs modulate delay reads: `LFO_1` at 0.5 Hz applies ±80
samples of modulation to the first input diffuser allpass (ap1), and `LFO_2` at 0.3 Hz applies
±100 samples to the `del2` read in loop 1. Both use `stmlib::CosineOscillator` with `APPROXIMATE`
mode. LFO values are refreshed every 32 samples (`(write_ptr_ & 31) == 0`) and held constant
between updates — a 32-sample LFO update period at 32 kHz means LFO values update at ~1000 Hz,
well above any LFO rate.

The `c.Lp(state, klp)` operation is `state += klp * (accum - state); accum = state` — a per-loop
one-pole LP inside the feedback path. `klp` is `patch_.reverb_lp` (seeded from serial number,
range `[0.7, 0.9]`), attenuating high frequencies on each reverb loop traversal.

**Space metaparameter** controls reverb behavior:
- `space <= 0.05`: `raw_gain = 1.0` — the raw (pre-resonator) signal bleeds through at full level,
  bypassing synthesis entirely for a dry passthrough feel
- `0.05 < space <= 0.1`: `raw_gain = 2 - space*20` — linear fade-out of raw bleed
- `space > 0.1`: `spread = min(space - 0.1, 0.7)` — stereo width grows
- `space > 0.6`: `reverb_amount = space - 0.6` — reverb wet mix grows
- `reverb_time = 0.35 + 1.2 * reverb_amount`
- `space >= 1.75`: freeze — `input_gain = 0, time = 1.0, lp = 1.0` (infinite hold, no new input)

---

### 12. Serial Number Seeding (`dsp/part.cc`)

```cpp
void Part::Seed(uint32_t* seed, size_t size) {
    uint32_t signature = 0xf0cacc1a;
    for (size_t i = 0; i < size; ++i) {
        signature ^= seed[i];
        signature = signature * 1664525L + 1013904223L;
    }
    // Extract 3-bit fields from the scrambled value:
    x = static_cast<float>(signature & 7) / 8.0f;
    signature >>= 3;
    patch_.resonator_modulation_frequency = (0.4f + 0.8f * x) / kSampleRate;

    x = static_cast<float>(signature & 7) / 8.0f;
    signature >>= 3;
    patch_.resonator_modulation_offset = 0.05f + 0.1f * x;

    x = static_cast<float>(signature & 7) / 8.0f;
    signature >>= 3;
    patch_.reverb_diffusion = 0.55f + 0.15f * x;

    x = static_cast<float>(signature & 7) / 8.0f;
    signature >>= 3;
    patch_.reverb_lp = 0.7f + 0.2f * x;

    x = static_cast<float>(signature & 7) / 8.0f;
    signature >>= 3;
    patch_.exciter_signature = x;
}
```

The scrambler is a standard LCG (`a=1664525, c=1013904223`, Numerical Recipes constants) run once
per UID word after XOR. Three bits are extracted per parameter: 8 discrete values (0/8, 1/8, ...
7/8). The resulting ranges:

| Parameter | Range |
|---|---|
| `resonator_modulation_frequency` | [0.4, 1.2] / 32000 Hz |
| `resonator_modulation_offset` | [0.05, 0.15] |
| `reverb_diffusion` | [0.55, 0.70] (allpass coefficient) |
| `reverb_lp` | [0.70, 0.90] (reverb LP coefficient) |
| `exciter_signature` | [0, 0.875] — selects noise sample window and plectrum amplitude |

These five parameters are never exposed to the user. They are overwritten each boot from the MCU
UID, so they cannot be saved or changed. The call site in `elements.cc` passes address `0x1fff7a10`
with `size=3` (reading 96-bit UID words 0, 1, 2).

---

### 13. Panic Recovery (`dsp/part.cc`)

`Part::Process()` tracks `resonator_level_` as a mean-square level estimate of the main output:

```cpp
for (size_t i = 0; i < size; ++i) {
    float error = main[i] * main[i] - resonator_level;
    resonator_level += error * (error > 0.0f ? 0.05f : 0.0005f);
}
```

This is an asymmetric leaky integrator (same topology as the noise gate): attack = 0.05, release
= 0.0005. The 100:1 asymmetry means the level estimate rises quickly on loud signals but falls
very slowly. If `resonator_level >= 200.0f`, `panic_ = true`.

On the next `Process()` call with `panic_` set, `voice_[i].Panic()` is called, which calls
`ResetResonator()`, zeroing all SVF states and re-initializing the delay lines. Then
`resonator_level_ = 0.0f` and `panic_ = false`. This recovers from NaN propagation through a
blown-up filter bank (which would otherwise lock the module with all-NaN output indefinitely).

---

### 14. OminousVoice — Easter Egg FM Synth (`dsp/ominous_voice.cc`)

Activated by holding the button for 3+ seconds with all resonator attenuverters at maximum and
all damping/position/space pots at maximum. Toggle stored in flash (`CalibrationSettings::
boot_in_easter_egg_mode`). The LED pattern cycles to display that it's active.

`OminousVoice` is a 2-operator FM synthesizer processed at 8× oversampling (`kOversamplingUp = 8`,
256 kHz internal rate). Each oscillator is a `FmOscillator` with carrier and modulator. The
modulator phase is `frequency + lut_fm_frequency_quantizer[ratio * 128]` (quantized to harmonic
ratios), and the carrier uses self-feedback from the previous sample (`feedback_amount` from
`exciter_bow_timbre`). To prevent aliasing at high FM amounts, an attenuation factor `1 - brightness
* brightness * 0.0015` is applied, clamped to zero.

Downsampling uses a two-stage chain: an IIR SVF LP at `1/8 * 0.8 = 0.1` normalized frequency
(4th-order Butterworth), followed by a 101-tap Parks-McClellan FIR designed with scipy
(`remez(101, [0, 0.3/8, 0.495/8, 0.5], [1, 0])`). The IIR pre-filters to -48 dB stopband; the
FIR achieves the final transition band shaping. The FIR is processed by picking every 8th
sample after updating the ring buffer for all 8 oversampled inputs.

Each oscillator feeds a `Spatializer` that rotates at rates `[f, f*1.123456]` (slightly detuned
rotation speeds create an inward/outward stereo motion). Distance is set from `resonator_position`.
The `Spatializer` applies a low-pass `behind_filter_` (SVF at 0.05 normalized frequency) to model
sound-behind-head tonal attenuation, then computes L/R gains via sine table lookup at the current
rotation angle.

The `exciter_blow_meta` and `exciter_strike_meta` knobs select FM ratio for oscillator 0 and 1
respectively. `exciter_bow_level` selects the inter-oscillator detune via `lut_detune_quantizer`.

---

### 15. Memory Layout

| Region | Contents |
|---|---|
| CCM RAM (`0x10000000`, 64 KiB) | `reverb_buffer[32768]` (64 KiB, FORMAT_16_BIT) |
| Main SRAM | All other objects: `Part`, `Voice` (including `Resonator` with 64 SVFs + 8 bowed SVFs + 8 delay lines), `String[5]`, `Tube`, `Diffuser`, audio I/O buffers |
| Flash | Firmware, sample tables (`smp_sample_data`: 128,013 samples, `smp_noise_sample`: 40,963 samples), all lookup tables |
| Flash (calibration sector) | `CalibrationSettings` — pitch offset/scale, CV offsets, easter egg flag, resonator model |

`Resonator` memory footprint (on stack/SRAM as part of `Voice`):
- `stmlib::Svf f_[64]`: 64 SVFs — each SVF has ~4 floats of state (16 bytes) ≈ 1024 bytes
- `stmlib::Svf f_bow_[8]` + `stmlib::DelayLine<float, 1024> d_bow_[8]`: 8 × 4096 bytes = 32 KiB
- Work buffers in `Voice`: `bow_buffer_`, `bow_strength_buffer_`, `blow_buffer_`, `strike_buffer_`
  (each 16 floats = 64 bytes) + `diffuser_buffer_[1024]` = 4096 bytes

The bowed delay lines (`d_bow_`) at 1024 floats each × 8 = 32 KiB is the single largest object
in SRAM. This fits on the STM32F407 which has 128 KiB main SRAM + 64 KiB CCM.

---

### 16. Non-Obvious Implementation Choices

**Bow exciter does not directly excite the resonator**: `ProcessFlow` output goes to `bow_buffer_[]`,
then multiplied by `e * bow_level * 0.125 * accent` before being summed into `raw[]` (the resonator
input). However the *primary* role of the bow exciter signal is as a strength envelope: it
populates `bow_strength_buffer_[i] = e * patch.exciter_bow_level`. This buffer is passed to
`Resonator::Process()` as the `bow_strength` argument and drives the `BowTable` friction model in
the bowed waveguide modes. The small direct contribution (`* 0.125`) is incidental.

**Strike bleed is a second nonlinear loudness stage**: When `exciter_strike_level > 0.8`:
```cpp
strike_bleed = (strike_level - 1.0f) * 2.0f;
```
The raw strike buffer is summed into `center[]` after the resonator. This is not filtered by the
resonator at all — it is a direct click bleed that bypasses modal synthesis. It is used for the
case where a player wants the initial transient "thwack" of the beater to be audible separately
from the resonated tone.

**Position slew rate of 0.01 is extremely slow**: At 32 kHz / 16 samples/block = 2000 blocks/sec,
`lp_coefficient = 0.01` means a time constant of ~50 blocks = ~25 ms. This prevents any
position-change from causing audible steps in the cosine amplitude series even at the per-block
rate. Position changes from CV can still happen smoothly within this slew window.

**`lut_midi_to_f_high` × `lut_midi_to_f_low`**: The MIDI-to-frequency table is split into coarse
(integer note, 256-entry) and fine (fractional sub-semitone, 256-entry) tables, indexed by 8.8
fixed-point. This avoids a 65536-entry single table while still giving 1/256 semitone resolution.

**Envelope attack shape is quartic**: The `set_adsr()` call sets `shape_[0] = ENV_SHAPE_QUARTIC`.
The quartic curve (`t^4`) is very fast at first and slows dramatically — giving attacks a very
punchy initial transient that transitions smoothly. Decay and release use `ENV_SHAPE_EXPONENTIAL`.

**SVF filter frequency accuracy**: The resonator uses `FREQUENCY_FAST` mode for filter coefficient
computation, which sacrifices some accuracy for speed. The bowed-mode SVFs use `set_g_q` directly
(bypassing frequency computation entirely) by reusing `f_[i].g()` from the already-computed
normal mode. The string model's IIR damping filter uses `FREQUENCY_ACCURATE` mode because at low
pitches the damping cutoff frequency needs to track closely.

**The `space` range extends to 2.0, not 1.0**: The `BIND` macro in `cv_scaler.cc` clamps `space`
to `[0.0, 2.0]`. Values `1.0–1.75` encode higher reverb times (up to `krt = 0.35 + 1.2*0.65 =
1.13`, well above 1.0 but clipped in the reverb feedback path), and values `>= 1.75` trigger
freeze. The `krt > 1.0` region is a deliberately intentional instability zone that creates a slowly
building wash.
