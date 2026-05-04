Braids
==

Braids is a voltage-controlled monophonic digital sound source from Mutable
Instruments. It implements 47 oscillator algorithms spanning analog emulation,
FM synthesis, physical modeling, wavetable scanning, drum synthesis, and more.

For each algorithm, TIMBRE and COLOR CV inputs serve different purposes
(described below). The FM input can modulate pitch, or in some modes take on
alternative roles.

All audio processing runs at 96 kHz in blocks of 24 samples. After the
oscillator stage, the signal passes through sample rate reduction, bit
crushing, VCA, and the signature waveshaper (a per-device unique transfer
function seeded from the MCU serial number, adding subtle unit-to-unit
variation).

Architecture
--

The firmware is organized in three layers:

- **`AnalogOscillator`** renders 9 primitive analog-style waveforms using
  phase accumulation with BLAMP (band-limited step) anti-aliasing. Used as
  building blocks by the macro layer.

- **`DigitalOscillator`** implements 33 digital synthesis algorithms: FM,
  physical modeling, wavetables, drum synthesis, noise-based, formant/vocal
  synthesis, and granular synthesis. State is held in a `DigitalOscillatorState`
  union so only one algorithm's state exists at a time.

- **`MacroOscillator`** presents the 47 user-facing models via a function
  pointer dispatch table (`fn_table_`). These either wrap a single
  `AnalogOscillator`/`DigitalOscillator` shape, or combine multiple
  oscillators (triple stacks, sync pairs, saw + comb filter, etc.).

On shape change, the oscillator state is reset (`Init()` called) to prevent
zapping artifacts. On strike, `digital_oscillator_.Strike()` is called, which
triggers percussive/excitation events in the drum and physical models.

### Parameter Interpolation

All parameter changes (TIMBRE, COLOR, pitch) are linearly interpolated across
each 24-sample block using macros defined in `parameter_interpolation.h`. This
prevents zipper noise when a 96 kHz audio rate modulates a parameter from a
4 kHz control rate (one sample per 24-sample block = 96/24 = 4 kHz).

The `BEGIN_INTERPOLATE_PARAMETER_0` / `INTERPOLATE_PARAMETER_0` /
`END_INTERPOLATE_PARAMETER_0` macros compute a start value, delta, and
increment per sample, yielding smooth transitions.

Phase increments are also interpolated (`BEGIN_INTERPOLATE_PHASE_INCREMENT`
etc.) for the same reason, preventing audio-rate clicks when pitch CV changes.

### Hardware Tuning: ComputePhaseIncrement

Pitch is expressed as a 16-bit MIDI-like value (`60 << 7` = middle C). The
`ComputePhaseIncrement()` function converts this to a phase increment using a
lookup table (`lut_oscillator_increments`) indexed by the pitch value, with
linear interpolation between table entries. Octave transposition is handled by
bit-shifting the increment, computed by looping the pitch down by octave
until it falls within the valid table range, then shifting back.

The digital oscillator has the same function plus `ComputeDelay()`, which uses
a separate lookup table (`lut_oscillator_delays`) to convert pitch to a delay
line length for physical modeling and comb filter modes.

---

Macro Oscillator Algorithms
--

Below is the complete list of 47 user-facing algorithms with detailed
explanations of how each works internally, grouped by synthesis family.

## Analog Waveforms

These use the `AnalogOscillator` with BLAMP anti-aliasing and support hard
sync with fractional reset times.

### 0. CSAw

**Implementation:** Delegates to `AnalogOscillator::RenderCSaw`.

A single `AnalogOscillator` with shape `OSC_SHAPE_CSAW` renders the waveform.
TIMBRE maps to the pulse width (`pw = parameter * 49152`) which controls the
duty cycle of the pulse component. COLOR maps to the aux parameter, setting
`discontinuity_depth_` (the amplitude of the pulse step) to `-2048 + (COLOR >> 2)`,
creating a wavefolder-like saturation effect via lookup table `ws_moderate_overdrive`
applied in the output stage.

The CSaw algorithm oscillates between two states: a rising sawtooth ramp
(phase < pw) and a downward pulse step (phase >= pw), each generating BLAMP
correction at the transitions. A DC shift of 8192 is subtracted and the output
is gained up by roughly 13/8.

### 1. MORPH

**Implementation:** `MacroOscillator::RenderMorph`.

Two `AnalogOscillator` instances render different waveforms that are crossfaded
in a multi-segment TIMBRE sweep:

- **Segment 1** (TIMBRE 0-10922): Triangle crossfading to Saw.
- **Segment 2** (TIMBRE 10923-21845): Square crossfading to Saw.
- **Segment 3** (TIMBRE 21846-65535): Square with increasing PWM, crossfading
  to Sine.

COLOR adds a low-pass filter (one-pole, `lp_state += (sample - lp_state) * f >> 15`)
on the mixed signal, with cutoff frequency tracking inverse of COLOR. The
filtered signal is then sent through `ws_violent_overdrive` waveshaper with
the wet/dry mix controlled by COLOR. Drive is attenuated at high pitches to
prevent excessive aliasing.

### 2. SAW/SQUARE

**Implementation:** `MacroOscillator::RenderSawSquare`.

Two `AnalogOscillator` instances: one `OSC_SHAPE_VARIABLE_SAW` and one
`OSC_SHAPE_SQUARE`, both receiving the same TIMBRE for pulse width control.
COLOR crossfades between the two via `Mix()`. The square wave is attenuated
by 148/256 before mixing.

### 3. SINE/TRIANGLE

**Implementation:** `MacroOscillator::RenderSineTriangle`.

Two analog oscillators: `OSC_SHAPE_SINE_FOLD` and `OSC_SHAPE_TRIANGLE_FOLD`.
TIMBRE controls fold intensity on both, but fold attenuation varies with pitch
to prevent aliasing:

- Sine fold attenuation = `32767 - 6 * (pitch - 92*128)` (rolls off above ~C8)
- Triangle fold attenuation = `32767 - 7 * (pitch - 80*128)` (rolls off above ~A7)

COLOR crossfades between the two folded waveforms.

The wavefolding in `RenderTriangleFold`/`RenderSineFold` works by multiplying
the raw waveform by a gain factor (`2048 + parameter * 30720 / 32767`) then
indexing into a waveshaper lookup table (`ws_tri_fold` or `ws_sine_fold`),
creating characteristic folded harmonics. Both are 2x oversampled.

### 4. BUZZ

**Implementation:** `MacroOscillator::RenderBuzz`.

Two `AnalogOscillator` instances with shape `OSC_SHAPE_BUZZ` are summed
together. TIMBRE controls the buzz spectrum shape via `parameter` offset
subtracted from pitch: `shifted_pitch = pitch + (32767 - TIMBRE) / 2`. This
shifts the zone selection for the bandlimited comb wavetables. COLOR detunes
the second oscillator by `COLOR >> 8` semitones.

The buzz oscillator renders by selecting two adjacent wavetables from
`WAV_BANDLIMITED_COMB_0` through `WAV_BANDLIMITED_COMB_14` based on
`shifted_pitch >> 10`, then crossfading between them. There are 15 pitch
zones, each with a precomputed bandlimited comb waveform.

## Stacking and Sub-Octave

### 5. SQUARE SUB

**Implementation:** `MacroOscillator::RenderSub`.

One `AnalogOscillator` with shape `OSC_SHAPE_SQUARE` renders the main voice.
A second `AnalogOscillator` with shape `OSC_SHAPE_SQUARE` renders a sub-octave
voice, pitched either 1 or 2 octaves below depending on COLOR (< 16384 =
2 octaves, >= 16384 = 1 octave). TIMBRE controls PWM on the main square.
COLOR also controls the sub mix level: at center (16384) the sub is silent,
at extremes the sub is at full volume.

### 6. SAW SUB

Same as SQUARE SUB but the main oscillator uses `OSC_SHAPE_VARIABLE_SAW`
instead.

### 7. SQUARE SYNC

**Implementation:** `MacroOscillator::RenderDualSync`.

Two `AnalogOscillator` instances with `OSC_SHAPE_SQUARE` are chained in a
master-slave hard sync configuration. The master oscillator renders to
`buffer` and writes fractional sync timing to `sync_buffer_` (a uint8_t array
where the value encodes the fractional reset position: `phase / (increment >> 7) + 1`).
The slave oscillator reads `sync_buffer_` as its sync input, so it resets its
phase at the master's reset point, achieving hard sync.

TIMBRE controls the slave pitch offset (`pitch + TIMBRE / 4`). COLOR
crossfades between master and slave outputs.

### 8. SAW SYNC

Same as SQUARE SYNC but both oscillators use `OSC_SHAPE_SAW`.

### 9-12. TRIPLE SAW / SQUARE / TRIANGLE / SINE

**Implementation:** `MacroOscillator::RenderTriple`.

Three `AnalogOscillator` instances are summed after attenuation by 21/64 each.
TIMBRE detunes oscillator 1, COLOR detunes oscillator 2. The detuning values
are looked up from a 65-entry `intervals[]` table containing musical intervals
ranging from -24 semitones to +24 semitones, with microtonal divisions at 4
cent resolution. The index into the table is `parameter >> 9` with crossfade
at `parameter << 8` for smooth interpolation between interval steps.

The base shape selects among SAW, SQUARE, TRIANGLE, or SINE based on the
macro shape enum.

## Ring Modulation / Mixing

### 13. TRIPLE RING MOD

**Implementation:** `DigitalOscillator::RenderTripleRingMod`.

Three sine oscillators (carrier + 2 modulators) multiplied together:
`out = sin(carrier_phase) * sin(mod1_phase) * sin(mod2_phase)`. Both modulator
phases use separate phase increments computed from pitch + a detune controlled
by TIMBRE (modulator 1) and COLOR (modulator 2). The resulting product goes
through `ws_moderate_overdrive` waveshaper.

State is persisted across blocks in `state_.vow.formant_phase[]` (reusing the
vowel state union member). Phase is stored with a 1<<30 offset to center the
carrier sine.

### 14. SAW SWARM

**Implementation:** `DigitalOscillator::RenderSawSwarm`.

Seven sawtooth oscillators summed together. The master oscillator uses the
base `phase_increment_`, while 6 additional oscillators are detuned using
increments precomputed at the start of each block. Detune amount is
quadratic in TIMBRE: `detune = (TIMBRE + 1024)^2 >> 9`, giving greater
spread at higher settings. Each slave increment is interpolated from
`ComputePhaseIncrement()` for fractional semitone detuning.

On strike, all 6 slave phases are randomized to prevent phase locking.

The summed signal passes through a state-variable high-pass filter (chamberlin
topology): `notch = in - bp * damp`, `lp += f * bp`, `hp = notch - lp`,
`bp += f * hp`. The cutoff tracks pitch with an offset controlled by COLOR,
which is remapped differently above/below 10922 (1/6 of range). The filter
output (`hp`) is taken.

### 15. SAW COMB

**Implementation:** `MacroOscillator::RenderSawComb` + `RenderComb`.

A sawtooth from `AnalogOscillator` is fed into a comb filter
(`DigitalOscillator::RenderComb`). TIMBRE controls the comb feedback
(resonance), COLOR controls delay time.

The comb filter uses an interpolated delay line of length `kCombDelayLength`
(8192 samples). Pitch is converted to delay length via `ComputeDelay()`.
To avoid clicks from sudden delay changes, the delay is filtered:
`filtered_pitch = (15 * filtered_pitch + pitch) / 16`. Resonance is
computed by warping TIMBRE through `ws_moderate_overdrive`.

The delayed sample is read with linear interpolation: `a + (b - a) * fraction`.
The feedback is `delayed * resonance + input / 2`. Output is `(input + delayed * 2) / 2`.

### 16. TOY

**Implementation:** `DigitalOscillator::RenderToy`.

Generates lo-fi 8-bit sounds at 4x oversampling. The base waveform is
generated by an XOR operation on the phase bits:

```
held_sample = (((phase >> 24) ^ (x << 1)) & (~x)) + (x >> 1)
```

where `x = COLOR >> 8`. TIMBRE controls decimation factor (sample-and-hold
rate): at maximum TIMBRE, every sample is fresh; at minimum, the value is
held for many samples. The 4x oversampled signal is filtered through a 4-tap
FIR filter (`kFIR4Coefficients = { 10530, 14751, 16384, 14751 }`) with DC
offset removal of 28208.

## Digital Filters

### 17-20. DIGI FILTER LP / PK / BP / HP

**Implementation:** `DigitalOscillator::RenderDigitalFilter`.

Self-oscillating resonant filters used as sound sources. The algorithm uses
a sine carrier amplitude-modulated by a window (saw or triangle, selected by
COLOR). The architecture differs by filter type (controlled by `filter_type`):

- **LP** (0): `saw_tri = window * carrier >> 16`, `square = integrator`
- **PK** (1): same but `square = (pulse + integrator) >> 1`
- **BP/HP** (2,3): `saw_tri = carrier * window >> 16`, `square = pulse`

The integrator accumulates a rectified double-saw pulse:
`square_integrator += pulse * integrator_gain >> 16`, where
`integrator_gain = modulator_phase_increment >> 14`. Polarity flips on each
phase zero crossing, creating a resonant peaking effect.

TIMBRE shifts the modulator frequency (`pitch + TIMBRE / 2`). The modulator
phase increment is smoothly interpolated to the target over the block.
COLOR crossfades between saw/triangle and square/integrator outputs.

On sync, polarity, phase, modulator phase, and integrator are all reset.

## Formant / Vocal

### 21. VOSIM

**Implementation:** `DigitalOscillator::RenderVosim`.

VOSIM (VOcoid SIMulation) synthesis. Two formant sine oscillators are
amplitude-modulated by a bell-shaped envelope (`lut_bell`). TIMBRE and COLOR
directly set the formant frequencies:

```
formant_increment[0] = ComputePhaseIncrement(TIMBRE / 2)
formant_increment[1] = ComputePhaseIncrement(COLOR / 2)
```

The carrier `phase_` indexes `lut_bell` to get the amplitude modulation
envelope. The sample output is:

```
sample = (16384 + 8192) + sine(formant_1)/2 + sine(formant_2)/4
sample *= lut_bell(phase) / 2
```

Formant phases are reset at the beginning of each carrier cycle (when
`phase < phase_increment`).

### 22. VOWEL

**Implementation:** `DigitalOscillator::RenderVowel`.

A bank of 3 formant oscillators produces vowel sounds. 9 vowel phonemes are
defined in `vowels_data[]` with 3 formant frequencies and 3 amplitudes each.
8 consonant definitions are in `consonant_data[]`.

On strike, a consonant is randomly selected and its formant frequencies and
amplitudes are loaded for 160 frames (~40ms at 4kHz block rate). After the
consonant period, the formant target transitions to the vowel selected by
TIMBRE (high 4 bits select vowel, low 12 bits provide crossfade between
adjacent vowels). COLOR controls formant shift (a global pitch scaling of
formant frequencies: `formant_shift = 200 + COLOR / 64`).

The three formants use `wav_formant_sine` for formants 1-2 and
`wav_formant_square` for formant 3, indexed by `(phase >> 24) & 0xf0 | amplitude`.
The output is multiplied by `255 - (carrier_phase >> 24)` to create an
amplitude envelope that decays over each cycle, with formant phase resets at
cycle boundaries. Random noise is optionally mixed in for certain consonants
(those with index >= 6).

### 23. VOWEL FOF

**Implementation:** `DigitalOscillator::RenderVowelFof`.

FOF (Forme d'Onde Formantique) synthesis implemented with a bank of 5
state-variable filters driven by a bandlimited sawtooth wave. The sawtooth
uses BLAMP anti-aliasing (same technique as the analog oscillator).

A 5x5x5 lookup table (`formant_f_data` / `formant_a_data`) stores formant
frequencies and amplitudes for 5 voice types (bass, tenor, countertenor,
alto, soprano) x 5 vowels (a, e, i, o, u). TIMBRE and COLOR select the
position in this 2D phoneme space with bilinear interpolation via
`InterpolateFormantParameter()`.

Each of the 5 formant channels runs a chamberlin SVF (frequency `f`,
amplitude-scaled output), summed together. The output is 2x upsampled
(rendered at 48 kHz and doubled).

## Harmonics

### 24. HARMONICS

**Implementation:** `DigitalOscillator::RenderHarmonics`.

Additive synthesis with 12 harmonics. The spectrum is shaped by two resonant
peaks:

- **Peak 1**: center frequency controlled by TIMBRE:
  `peak = 12 * TIMBRE / 128` (in harmonic index * 256 units)
- **Peak 2**: placed at `peak/2 + 12*128` with amplitude controlled by
  COLOR squared: `second_peak_amount = COLOR^2 / 32768`

Each harmonic's amplitude is computed from a Cauchy/Lorentzian distribution:
`g = 32768 * 128 / (128 + (x - peak)^2 / width)`, where bandwidth `width` is
derived from COLOR via a double square root for fine control at low settings.

Total gain is normalized so the sum of all 12 harmonics equals a fixed
reference. Harmonics whose frequency exceeds 0.25 * sample rate are zeroed
(preventing aliasing: `(phase_increment >> 16) * (i+1) > 0x4000`). Amplitudes
are smoothed over time: `amplitude += (target - amplitude) >> 8`.

Output is 2x upsampled.

## Frequency Modulation

### 25. FM

**Implementation:** `DigitalOscillator::RenderFm`.

Traditional 2-operator FM with sine carriers. The modulator phase increment is
`ComputePhaseIncrement(12*128 + pitch + (COLOR - 16384)/2) / 2` (one octave
above the carrier by default, with COLOR offset). The modulation index is
controlled by TIMBRE with linear interpolation across the block:

```
pm = sin(modulator_phase) * TIMBRE << 2
out = sin(carrier_phase + pm)
```

The `<< 2` scales the modulation for a wide index range.

### 26. FEEDBACK FM

**Implementation:** `DigitalOscillator::RenderFeedbackFm`.

FM where the output sample feeds back into the modulator phase:

```
pm = previous_sample << 14
pm = sin(modulator_phase + pm) * (TIMBRE * attenuation) << 1
out = sin(carrier_phase + pm)
```

COLOR controls the modulator frequency (same as regular FM). Attenuation
reduces the modulation index at high pitches:
`attenuation = 32767 - 4 * (pitch - 72*128 + (COLOR - 16384)/2)`, clamped to
[0, 32767]. This prevents aliasing from overly wide sidebands at high
frequencies. The feedback creates richer spectra than standard FM.

### 27. CHAOTIC FEEDBACK FM

**Implementation:** `DigitalOscillator::RenderChaoticFeedbackFm`.

FM where the modulator's *phase increment* itself is modulated by the audio
output, creating chaotic/nonlinear behavior:

```
pm = sin(modulator_phase) * TIMBRE << 1
out = sin(carrier_phase + pm)
modulator_phase += (modulator_increment / 256) * (129 + out / 512)
```

The modulator phase increment is multiplied by `(129 + output / 512)`, so the
modulator frequency varies with the audio signal. This positive feedback
creates unstable, noisy textures at high modulation indices.

TIMBRE controls modulation index, COLOR controls modulator base frequency.
Parameter quantization is applied (via `lut_fm_frequency_quantizer`) for
musical intervals.

## Physical Modeling - Strings

### 28. PLUCKED

**Implementation:** `DigitalOscillator::RenderPlucked`.

Karplus-Strong plucked string synthesis with 3-voice polyphony.

**On strike:** The active voice is round-robined (`active_voice_++ % 3`). The
delay line size is determined by the pitch, with dynamic oversampling (`shift`)
so the delay length fits in the allocated buffer (1025 samples per voice).
`initialization_ptr` is set based on COLOR (pick position): wider pick = more
high frequencies. The delay line is filled with white noise during
initialization (the "pluck"), after which the Karplus-Strong loop takes over.

**During sustain:** The delay line is read at the pitch-dependent rate
(`p.phase >> (22 + p.shift)`) with `Interpolate1022` interpolation. The
write pointer chases the read pointer. At each write, a two-point average
`(a + b) / 2` is applied with `update_probability`:

- When TIMBRE < 16384 (damped mode): always update, loss = frequency-dependent
  damping factor.
- When TIMBRE > 16384 (drone mode): update probability decreases, allowing
  the string to ring freely. Loss goes to 0 at max TIMBRE.

The loss factor is `4096 - (phase_increment >> 14)`, clamped to >= 256, and
further reduced by TIMBRE when in damped mode.

Output is 2x upsampled.

### 29. BOWED

**Implementation:** `DigitalOscillator::RenderBowed`.

Waveguide bowed string model with two delay lines (bridge and neck) and a
nonlinear bow friction function.

**Setup:** Two delay lines store bridge (int8_t, 1024 samples) and neck
(int8_t, 4096 samples) reflections. On strike, both are zeroed.

**At each sample:**
1. Read from bridge and neck delay lines with fractional interpolation.
2. Bridge signal passes through a one-pole low-pass filter
   (`kBridgeLPGain`, `kBridgeLPPole1`) to model string stiffness.
3. Reflections are computed as `-lp_state` (bridge) and `-nut_value` (neck).
4. String velocity = `bridge_reflection + nut_reflection`.
5. Bow velocity from `lut_bowing_envelope` (a decay envelope indexed by
   `excitation_ptr`).
6. Friction = nonlinear function of velocity difference. The friction is
   computed as `velocity_delta * TIMBRE_pressure / 32`, clamped, then used
   to index `lut_bowing_friction`.
7. New velocity = `friction * velocity_delta / 32768`.
8. Delay lines are updated: neck gets `bridge_reflection + new_velocity`,
   bridge gets `nut_reflection + new_velocity`.

**Body resonance:** A biquad IIR filter (`kBiquadGain`, `kBiquadPole1`,
`kBiquadPole2`) models the instrument body.

COLOR controls bow position (affects the bridge/neck delay split ratio).
TIMBRE controls bow pressure (affects the friction nonlinearity steepness).
Output is 2x upsampled.

## Physical Modeling - Wind

### 30. BLOWN

**Implementation:** `DigitalOscillator::RenderBlown`.

Reed wind instrument model (clarinet-like). A delay line (bore) of length
`kWGBoreLength` (2048 samples) with a nonlinear reed table and reflection
coefficient.

**At each sample:**
1. Breath pressure = `kBreathPressure` (26214) + random fluctuations
   (`Random::GetSample() * TIMBRE_pressure / 32768 * kBreathPressure / 32768`).
2. Read from bore delay line with fractional interpolation.
3. Pressure delta = reflection coefficient * (`dl_value / 2` + lp_state).
4. Reed nonlinearity: `reed = (pressure_delta * kReedSlope / 4096) + kReedOffset`,
   clamped to [0, 32767].
5. Output = `pressure_delta * reed / 32768 + breath_pressure`.
6. Write to bore delay line.
7. Body filter: one-pole low-pass with coefficient from `lut_flute_body_filter`
   indexed by pitch.

The reflection coefficient `kReflectionCoefficient = -3891` and reed parameters
`kReedSlope = -1229`, `kReedOffset = 22938` model the reed's nonlinear
pressure-flow relationship. TIMBRE controls breath pressure, COLOR controls
`normalized_pitch` used to index the body filter coefficient.

### 31. FLUTED

**Implementation:** `DigitalOscillator::RenderFluted`.

Flute physical model with dual delay lines (jet and bore) and a blowing jet
driving function.

**Setup:** Two delay lines: bore (`int8_t`, 4096 samples) and jet (`int8_t`,
1024 samples). On strike, both are zeroed and `excitation_ptr` = 0.

**Delay computation:**
- `bore_delay = 2 * pitch_delay - 2*65536`
- `jet_delay = bore_delay * (48 + COLOR/1024) / 256`
- `bore_delay -= jet_delay`
- If delays exceed buffer size, both are halved (transpose up an octave).

**At each sample:**
1. Read from bore and jet delay lines with fractional interpolation.
2. Breath pressure from `lut_blowing_envelope[excitation_ptr]` + random
   fluctuations scaled by breath intensity (TIMBRE).
3. Bore signal through body filter (one-pole)
   `lp_state = (-coeff * bore_value + (4096 - coeff) * lp_state) / 4096`.
4. DC blocking filter: `dc = 0.99 * dc + reflection - prev_input`,
   `prev_input = reflection`.
5. Pressure delta = `breath_pressure - reflection/2` written to jet delay.
6. Jet driving function: jet value is used to index `lut_blowing_jet`,
   added to `reflection/2`, written to bore delay.

`excitation_ptr` increments every 4 samples (every other output sample at 2x
upsampling), advancing the blowing envelope.

## Percussive - Modal

### 32. STRUCK BELL

**Implementation:** `DigitalOscillator::RenderStruckBell`.

Modal synthesis with 11 partials, each with its own frequency, amplitude, and
decay rate.

**Partials:** The 11 partial frequencies are defined as offsets from the base
pitch in `kBellPartials[]`: `{-1284, -1283, -184, -183, 385, 1175, 1536, 2233,
2434, 2934, 3110}` cents. These are inharmonic, giving bell-like spectra.
Amplitudes are in `kBellPartialAmplitudes[]`. Two sets of decay rates:
`kBellPartialDecayLong[]` and `kBellPartialDecayShort[]`, blended by TIMBRE.

**On strike:** All partial phases reset and amplitudes set to max.

**During sustain:** Every block, 3 partials have their frequencies updated
(round-robin to save CPU). COLOR detunes partials: even-indexed partials get
`+COLOR/128`, odd-indexed get `-COLOR/128`. Amplitudes decay multiplicatively:
`amplitude *= decay/65536`. TIMBRE blends long/short decay rates:
`balance = (32767 - TIMBRE)^2 / 128`, `decay = decay_long - (decay_long - decay_short) * balance / 128`.
At max TIMBRE (32000+), no decay occurs = droning bell.

**Per sample:** All 11 partials read from sine table, multiplied by amplitude,
summed. Output is 2x upsampled.

### 33. STRUCK DRUM

**Implementation:** `DigitalOscillator::RenderStruckDrum**.

Modal synthesis with 6 partials mixed with filtered noise.

**Partials:** 6 partials from `kDrumPartials[] = {0, 0, 1041, 1747, 1846,
3072}` cents (first two are at the fundamental). Amplitudes and decay rates
in corresponding arrays. Decay works the same as struck bell (long/short
blend by TIMBRE).

**Noise mode:** COLOR morphs between harmonic (partials only) and noise-
dominated modes. When COLOR > 16384, the noise gain increases and specific
partials are modulated by filtered noise:
- `noise_mode_1 = partials[1] * filtered_noise / 256`
- `noise_mode_2 = partials[3] * filtered_noise / 512`
- These are crossfaded with the main harmonic output.

The noise passes through a 3-pole low-pass filter (cascaded one-pole),
with cutoff tracking pitch.

On strike, partial amplitudes are reset. Target amplitudes are smoothed via
a fade increment to avoid clicks.

## Percussive - TR-808 Emulations

These use the `Excitation` class (exponential decay pulses) and `Svf` class
(state-variable filter) from Peaks.

### 34. KICK

**Implementation:** `DigitalOscillator::RenderKick`.

808-style kick drum with 3 excitation pulses feeding a resonant band-pass SVF.

**Setup:** Three `Excitation` pulses:
- `pulse[0]`: click, no delay, decay 3340
- `pulse[1]`: attack, 1ms delay, decay 3072
- `pulse[2]`: body, 4ms delay, decay 4093

One SVF in BP mode with punch.

**On strike:** `pulse[0]` triggered at amplitude 12*32768*0.7, `pulse[1]` at
-19662*0.7, `pulse[2]` at 18000. SVF punch set to 24000.

**During sustain:**
- Excitation = `pulse[0].Process() + (pulse[1].done() ? 0 : 16384) + pulse[1].Process()`
- SVF frequency tracks pitch with a pitch envelope from `pulse[2]`:
  while pulse[2] is active, frequency is `pitch + 17*128` (rising).
- SVF resonance controlled by TIMBRE: scaled and squared for a nonlinear
  response curve.
- Output passes through a one-pole LP filter with coefficient controlled by
  COLOR (quadruple-squared for nonlinear mapping). The LP state is smoothed:
  `lp += (resonator_output - lp) * lp_coefficient / 32768`.

Output is 2x upsampled.

### 35. SNARE

**Implementation:** `DigitalOscillator::RenderSnare`.

808-style snare drum with two tuned resonant SVFs plus a band-pass SVF for
noise (snare rattle).

**Setup:** 4 excitation pulses, 3 SVFs:
- `svf[0]`: drum body mode 1, tuned to `pitch + 12 semitones`
- `svf[1]`: drum body mode 2, tuned to `pitch + 24 semitones`
- `svf[2]`: noise BP filter, tuned to `pitch + 60 semitones`

**On strike:** All 4 pulses triggered. SVF resonance set from pitch-dependent
decay. COLOR controls the snappy/noise level: `pulse[3]` trigger amplitude
is `512 + min(COLOR, 14336) * 2`.

**Per sample:**
- Excitation 1 = `pulse[0] + pulse[1] + (pulse[1].done() ? 0 : 2621)` into
  `svf[0]`
- Excitation 2 = `pulse[2] + (pulse[2].done() ? 0 : 13107)` into `svf[1]`
- Noise = `Random::GetSample() * pulse[3].Process()` into `svf[2]`
- Output = `svf[0].Process(ex1) * g1 + svf[1].Process(ex2) * g2 + svf[2].Process(noise)`

TIMBRE crossfades gains `g1` and `g2` (one goes down as the other goes up,
centered at 22000).

### 36. CYMBAL

**Implementation:** `DigitalOscillator::RenderCymbal`.

808-style cymbal with 6 square-wave oscillators at pseudo-random harmonic
ratios, mixed through BP and HP SVFs.

**Oscillators:** 6 phase accumulators with ratios:
- Root = `ComputePhaseIncrement(40*128 + pitch/2)`
- increments = root/1024 * `{24273, 12561, 18417, 22452, 31858}` / 16
- Plus one additional oscillator at 24x root rate

The 6 MSBs of each phase accumulator are summed (each contributes either 0 or
1 from the sign bit) to create a pseudo-square waveform per oscillator:
`hat_noise = phase[0]>>31 + ... + phase[5]>>31 - 3`, then scaled by 5461.

The 7th oscillator at 24x rate clocks a pseudo-random generator
(`rng_state = rng_state * 1664525 + 1013904223`) which provides white noise.

**Filtering:**
- `svf[0]` in BP mode, frequency set by TIMBRE (COLOR in the original
  binding), resonance 12000
- `svf[1]` in HP mode, same frequency, resonance 2000

The square-wave oscillator sum goes through `svf[0]` (BP), then is mixed with
the noise through `svf[1]` (HP) via a COLOR-driven crossfade.

## Wavetable

### 37. WAVETABLES

**Implementation:** `DigitalOscillator::RenderWavetables`.

20 wavetables, each containing 4-16 individual waveforms of 129 samples.
Defined in `wavetable_definitions[]` with categories like "male", "female",
"choir", "space_voice", "tampura", "cello", "vibes", "piano", "organ",
"metallic", "bell", etc.

TIMBRE selects the position within the current wavetable (scaled by
`wt.num_steps`). Two adjacent waves are crossfaded with `Crossfade()`. COLOR
selects the wavetable (with 64-LSB hysteresis to prevent glitchy transitions
from single-bit DAC errors).

Output is 2x oversampled.

### 38. WAVE MAP

**Implementation:** `DigitalOscillator::RenderWaveMap`.

2D wavetable interpolation over a 16x16 grid (256 wave slots). The grid maps
to physical waves in `wt_waves` via `wt_map[]` (indirection).

TIMBRE selects the X coordinate, COLOR the Y coordinate. For each axis, the
integer part selects two adjacent grid rows/columns, and the fractional part
crossfades between them. Final output is a bilinear interpolation of the 4
surrounding waves:

```
sample = Mix(
    Crossfade(wave[0][0], wave[0][1], phase, y_xfade),
    Crossfade(wave[1][0], wave[1][1], phase, y_xfade),
    x_xfade)
```

2x oversampled.

### 39. WAVE LINE

**Implementation:** `DigitalOscillator::RenderWaveLine`.

Wavetable scanning along a 64-element path (`wave_line[]`). TIMBRE scans
through position with smoothing: `smoothed_param = (3 * prev + 2 * TIMBRE) / 4`.

COLOR morphs between smooth interpolation and stepped (quantized) scanning:
- **COLOR < 8192**: Uses TIMBRE for fine position, renders both "rough"
  (quantized to wave boundaries) and "smooth" (fractionally interpolated)
  versions, crossfades between them by COLOR.
- **8192 < COLOR < 16384**: Crossfade between current wave and next wave.
- **16384 < COLOR < 24576**: Both "rough" and "smooth" are crossfaded from
  next wave.
- **COLOR > 24576**: Both "rough" and "smooth" from next wave but with
  different quantization bit depths (smooth = 7-bit, rough = 5-bit).

This creates a continuum from glitch-free morphing to deliberately lo-fi
zipper-noise effects.

### 40. WAVE PARAPHONIC

**Implementation:** `DigitalOscillator::RenderWaveParaphonic**.

Paraphonic wavetable synthesis with 4 voices scanning a 32-wave mini-wavetable
(`mini_wave_line[]`). COLOR selects one of 17 chord types from `chords[][]`,
each defining 3 intervals (in cents) above the base pitch. The 4 voices are:

- Voice 1: base pitch (no detune)
- Voices 2-4: base pitch + chord intervals 0, 1, 2

Chord selection includes hysteresis: the fractional part of COLOR must be in
the ranges [0, 30720) for chord N, [30720, 34816) for crossfade, [34816, 65535]
for chord N+1.

TIMBRE selects the wave position in the 32-wave table (with crossfade between
adjacent waves). All 4 voices share the same wave but run at different pitches,
producing chordal harmony. On strike, all phases are randomized.

## Noise-Based

### 41. FILTERED NOISE

**Implementation:** `DigitalOscillator::RenderFilteredNoise`.

White noise (`Random::GetSample()`) through a single chamberlin state-variable
filter. Pitch controls cutoff. TIMBRE controls resonance via `lut_svf_damp`
and `lut_svf_scale`. COLOR morphs between LP, BP, and HP: first half of range
crossfades LP to BP, second half crossfades BP to HP.

Gain correction is applied to normalize filter gain (which peaks at resonance):
`gain_correction = f > scale ? scale * 32767 / f : 32767`. Output goes through
`ws_moderate_overdrive`.

### 42. TWIN PEAKS NOISE

**Implementation:** `DigitalOscillator::RenderTwinPeaksNoise`.

White noise through two parallel 2-pole resonators producing two spectral
peaks. Each resonator uses the difference equation:

```
y0 = input * scale
y0 += y1 * coefficient - y2 * Q_squared
y2 = y1, y1 = y0
```

where `coefficient` and `scale` come from `lut_resonator_coefficient` and
`lut_resonator_scale` indexed by pitch. Q (resonance) is controlled by TIMBRE:
`q = 65240 + TIMBRE / 128`, `Q_squared = q^2 / 131072`.

The first resonator's frequency tracks pitch. The second resonator's frequency
is pitch + COLOR offset (both are constrained to [0, 16383]). Both resonator
outputs are summed and gain-compensated: `makeup_gain = 8191 - TIMBRE / 4`.

The resonators only process positive input samples (negative samples are
mirrored), acting as a half-wave rectifier to create energy at the resonant
frequencies.

Output is 2x upsampled (each sample duplicated).

### 43. CLOCKED NOISE

**Implementation:** `DigitalOscillator::RenderClockedNoise`.

A clocked/stepped random voltage generator. The internal phase runs at 8x the
base pitch (3 left shifts of phase increment, with the condition that
`increment < 2^31` to prevent overflow). Each phase wrap (clock tick)
generates a new random value using the LCG: `rng = rng * 1664525 + 1013904223`.

TIMBRE controls a `cycle_phase` that runs independently. When `cycle_phase`
wraps, the RNG is reset to a seed value (randomized on strike). This creates
repeatable looping patterns at low TIMBRE settings.

COLOR controls the number of quantization steps (1-8, encoded as `1 + COLOR >> 10`).
Each random value is quantized: `sample -= sample % quantizer_divider`.

Output values are held between clock ticks (sample-and-hold behavior). The
clock rate is phase-locked to the sample rate by resetting `phase = phase_increment`
after each clock tick, ensuring exact division.

### 44. GRANULAR CLOUD

**Implementation:** `DigitalOscillator::RenderGranularCloud`.

Granular synthesis with 4 overlapping sine-wave grains, each with its own
phase, phase increment, and envelope phase.

**Grain scheduling:** Each grain runs a separate envelope. When a grain's
envelope completes (`envelope_phase > 2^24`), it may be rescheduled with
probability 16384/65536 (~25%). TIMBRE controls the grain rate via
`lut_granular_envelope_rate[TIMBRE >> 7] << 3`. COLOR controls pitch
randomization: each new grain gets a pitch offset of
`Random::GetSample() * COLOR / 65536`, applied with different scaling factors
for positive vs. negative offsets.

**Per sample:** Each grain's phase and envelope phase advance. The grain's
sine output is multiplied by its envelope amplitude from
`lut_granular_envelope[envelope >> 16]`. All 4 grains are summed with 1/4
scaling (not explicitly -- the sum can clip).

### 45. PARTICLE NOISE

**Implementation:** `DigitalOscillator::RenderParticleNoise`.

Random impulses exciting three parallel 2-pole resonators.

**Impulse generation:** Every sample, a random number is generated. If
`noise & 0x7fffff < density` (where `density = 1024 + TIMBRE`), a particle
is triggered with full amplitude (65535). The impulse value decays
exponentially: `amplitude = amplitude * kParticleNoiseDecay / 65536`.

**Resonators:** When a particle fires, three resonator frequencies are
computed from pitch with random offsets controlled by COLOR:

```
p1 = pitch + 3 * noise_a * COLOR / 131072 + 0x600
p2 = pitch + noise_a * COLOR / 32768 + 0x980
p3 = pitch + noise_b * COLOR / 65536 + 0x790
```

Each resonator uses the same 2-pole structure as Twin Peaks Noise, but with
fixed Q (`kResonanceFactor = 32768 * 0.996`). The three resonator outputs
are summed. Output is 2x upsampled (each sample duplicated).

## Digital / Data

### 46. DIGITAL MODULATION

**Implementation:** `DigitalOscillator::RenderDigitalModulation`.

QAM (Quadrature Amplitude Modulation) data stream emulation. Generates a
modem-like sound by modulating QAM symbols onto a sine carrier.

**Symbol stream:** A secondary phase accumulator runs at a rate determined by
`pitch - 1536 + (TIMBRE - 32767) / 3`. Every 4 symbol clock ticks, a new data
byte is shifted. The 128-symbol frame structure is:

- First 32 symbols: preamble (0x00)
- Next 16 symbols: sync word (0x99)
- Next 16 symbols: sync word (0xCC)
- Remaining symbols (1024): data bytes from COLOR, smoothed through a one-pole
  filter: `filtered = (3 * filtered + COLOR) / 4`.

**QAM modulation:** Each 2-bit symbol maps to a constellation point:
`{I, Q} = {23100, -23100}` in four combinations. The output is
`I * sin(phase) + Q * cos(phase)`.

On strike, the symbol count resets.

## Easter Egg

### 47. ?

**Implementation:** `DigitalOscillator::RenderQuestionMark`.

Morse code generator. A lookup table `wt_code[]` contains the Morse code
pattern for "BRAIDS": each pair of bits encodes the length of a Morse element
(2 bits = 0-3 dits). The pattern cycles through elements:

1. If `rng_state` is true: generate sine tone (`sin(phase) * 3/4`)
2. If `rng_state` is false: silence

Between elements (every `dit_duration` samples), the next element length is
decoded. After the full sequence, a long pause (100 dits) occurs before
repeating.

TIMBRE controls `dit_duration` (slower/faster). COLOR controls noise intensity:
random noise is added to the sine tone and passed through a waveshaper
(`sample^2 / 16384`) for distortion. The noise threshold adjusts the point
at which noise becomes audible: higher COLOR = more noise.

---

Common DSP Components
--

### State Variable Filter (`svf.h`)

Used by the drum and vocal algorithms. A chamberlin topology:

```
notch = input - (bp * damp) >> 15
lp += f * bp >> 15
hp = notch - lp
bp += f * hp >> 15
```

Supports LP, BP, HP modes. The `punch` feature boosts the filter frequency
and reduces damping when the filter's `lp` state is high, giving an
envelope-dependent resonance sweep.

### Excitation Generator (`excitation.h`)

An exponential decay generator: `state = (state * decay) >> 12`. On trigger,
the level is set and after an optional delay, the state jumps to `abs(level)`.
The output polarity tracks the sign of the trigger level.

### BLAMP Anti-Aliasing

The analog oscillator uses band-limited step (BLEP) correction for waveforms
with discontinuities. Two polynomial corrections are applied:

```
ThisBlepSample(t) = t^2 >> 18   (where t in [0, 65535])
NextBlepSample(t) = -(65535-t)^2 >> 18
```

These correction values are applied at each discontinuity (phase reset, pulse
width transition) to the current and next sample, producing bandlimited
waveforms without aliasing.

The correction amplitude is scaled by the magnitude of the discontinuity
(`discontinuity = after - before`), so larger voltage jumps get more
correction.

### Hard Sync with Fractional Timing

Hard sync is implemented by passing a `uint8_t* sync` array between oscillator
instances. The sync signal encodes the fractional reset time within a sample:
`*sync = phase / (increment >> 7) + 1`. A value of 0 means no sync, and
values 1-128 encode the fractional position.

When a slave oscillator receives a sync signal, it computes the phase at the
sync point and applies BLAMP corrections for any discontinuity occurring
during the reset. This allows for perfectly bandlimited hard sync even with
fractional timing (rather than sample-quantized resets).

---

Project Structure
--

**braids.cc** -- Top-level entry point. Hardware initialization, audio
rendering loop at 96 kHz. Reads CV inputs, runs envelope, pitch quantization,
oscillator rendering, then effects (sample rate reduction, bit crushing, VCA,
signature waveshaper).

**analog_oscillator.cc/.h** -- Analog-style oscillator primitives with BLAMP.

**digital_oscillator.cc/.h** -- All digital/complex algorithms. 33 engines.

**macro_oscillator.cc/.h** -- Dispatches to analog/digital/composite
algorithms. Contains composite algorithms: Morph, SawSquare, SineTriangle,
Buzz (dual), Sub, DualSync, Triple*.

**envelope.h** -- AD envelope generator. Two-segment exponential envelope.

**excitation.h** -- Exponential decay generator (from Peaks), used for 808
drum excitation pulses.

**signature_waveshaper.h** -- Per-device unique waveshaper using MCU serial
number. Applies sigmoid distortion with small "bumplets" that vary per unit.

**svf.h** -- State Variable Filter (LP/BP/HP).

**vco_jitter_source.h** -- Analog-style pitch drift generator using a
pseudo-random temperature fluctuation model.

**quantizer.cc/.h** -- Note quantizer for snapping pitch CV to musical scales.

**quantizer_scales.h** -- 45 quantizer scale definitions.

**settings.cc/.h** -- Persistent settings storage (flash).

**parameter_interpolation.h** -- Macros for linear interpolation across blocks.

**ui.cc/.h** -- Display, encoder, menu navigation, calibration wizard.

**resources.cc/.h** -- Auto-generated lookup tables.

**drivers/** -- Hardware abstraction: ADC, DAC, gate input, encoder, OLED
display, system clock, temperature sensor.

**resources/** -- Python scripts that precompute lookup tables during build.

**bootloader/** -- Firmware update bootloader.

**hardware_design/** -- Eagle CAD schematic/board files, BOM, gerbers, panel
drawings.

**test/** -- Desktop test program for debugging without hardware.

**data/** -- Raw binary wavetable data and layout map.

Build Process
--

1. Python scripts in `resources/` generate `resources.cc` and `resources.h`.
2. Bootloader `.cc` files are compiled to a `.hex` file.
3. Main firmware `.cc` files are compiled to a `.hex` file.
4. The two hex files are merged into the final binary written to the MCU.
