# Plaits — Deep DSP Reference

Plaits is a Mutable Instruments macro-oscillator Eurorack module and the successor to Braids. It provides 24 synthesis engines selectable by a single knob, each controlled by a common four-parameter interface: HARMONICS, TIMBRE, MORPH, and V/oct pitch. Two simultaneous audio outputs are always produced: the primary OUT and a complementary AUX. This document is a technical reference aimed at DSP developers; it covers every engine in detail, the Voice render loop, the LPG model, parameter pre-processing, and the shared RAM system.

---

## Global DSP Constants

Defined in `dsp/dsp.h`:

- `kSampleRate = 48000.0f` — nominal rate
- `kCorrectedSampleRate = 47872.34f` — actual I2S clock (72 MHz system clock / 47 divider ratio); used for `a0` and the 6-op FM engine
- `a0 = (440.0f / 8.0f) / kCorrectedSampleRate` — normalized frequency of A-9 (one A four octaves below concert A); all `NoteToFrequency()` results are expressed as normalized frequency (cycles per sample, range 0–0.5)
- `kMaxBlockSize = 24`, `kBlockSize = 12` — render block sizes in samples (~0.25 ms per block)

The 0.27% difference between `kSampleRate` and `kCorrectedSampleRate` would put the module ~5 cents flat if ignored. The correction applies everywhere except `kSampleRate`-derived timing constants (envelope coefficients), where a few cents of error is acceptable.

---

## Architecture Overview

### The `Voice` Class

`Voice` (`dsp/voice.h`, `dsp/voice.cc`) is the single top-level synthesis object. It owns all 24 engine instances as concrete member variables (no heap allocation), an `EngineRegistry<kMaxEngines>` pointer array, two `ChannelPostProcessor` objects, a `DecayEnvelope`, an `LPGEnvelope`, and two float render buffers (`out_buffer_[kMaxBlockSize]`, `aux_buffer_[kMaxBlockSize]`).

`Voice::Render()` is called once per 12-sample block. Its pipeline:

1. Write the trigger CV to `trigger_delay_` (a 5-sample ring buffer) and read the delayed value. This absorbs the typical 1 ms CV-gate skew from sequencers and MIDI-to-CV converters.
2. Detect rising edge (threshold 0.3 V upward, 0.1 V downward hysteresis). On rising edge: latch `engine_cv_` from `modulations.engine`, call `lpg_envelope_.Trigger()` (if LEVEL is unpatched), call `decay_envelope_.Trigger()`.
3. Compute `engine_index` via `engine_quantizer_.Process(patch.engine, engine_cv_)`. `HysteresisQuantizer2` applies a 5% deadband around bin boundaries to prevent jitter at the boundary when the ENGINE CV hovers near a transition.
4. On engine change or `reload_user_data_` flag: call `LoadUserData()` and `Reset()` on the incoming engine, then reset `out_post_processor_`.
5. Build `EngineParameters p`: compute `p.harmonics`, `p.timbre`, `p.morph`, `p.note`, `p.accent`, `p.trigger` via `ApplyModulations()`.
6. Call `e->Render(p, out_buffer_, aux_buffer_, size, &already_enveloped)`.
7. Determine LPG bypass: `lpg_bypass = already_enveloped || (!level_patched && !trigger_patched)`.
8. If not bypassed, update `lpg_envelope_` via `ProcessPing()` (trigger mode) or `ProcessLP()` (level mode).
9. Call `out_post_processor_.Process()` and `aux_post_processor_.Process()` to apply gain and LPG and write to `Frame*` (interleaved `short` pairs).

### The `EngineRegistry` and Shared RAM

All engines share one contiguous RAM region managed by `stmlib::BufferAllocator`. During `Voice::Init()`, each engine's `Init()` is called in sequence, but the allocator is rewound before each call:

```cpp
for (int i = 0; i < engines_.size(); ++i) {
    allocator->Free();
    engines_.get(i)->Init(allocator);
}
```

Only one engine is active at a time, so this is safe. Engines allocate scratch buffers (delay lines, grain state arrays, FM accumulators, wavetable index maps) from this arena. A string voice's Karplus-Strong delay line and the 6-op FM accumulator buffer occupy the same physical RAM.

Registration order in `Voice::Init()` sets engine indices 0–23:

| Index | Class | `already_enveloped` | `out_gain` | `aux_gain` |
|-------|-------|--------------------|-----------|-----------:|
| 0 | `VirtualAnalogVCFEngine` | false | 1.0 | 1.0 |
| 1 | `PhaseDistortionEngine` | false | 0.7 | 0.7 |
| 2 | `SixOpEngine` (bank A) | true | 1.0 | 1.0 |
| 3 | `SixOpEngine` (bank B) | true | 1.0 | 1.0 |
| 4 | `SixOpEngine` (bank C) | true | 1.0 | 1.0 |
| 5 | `WaveTerrainEngine` | false | 0.7 | 0.7 |
| 6 | `StringMachineEngine` | false | 0.8 | 0.8 |
| 7 | `ChiptuneEngine` | false* | 0.5 | 0.5 |
| 8 | `VirtualAnalogEngine` | false | 0.8 | 0.8 |
| 9 | `WaveshapingEngine` | false | 0.7 | 0.6 |
| 10 | `FMEngine` | false | 0.6 | 0.6 |
| 11 | `GrainEngine` | false | 0.7 | 0.6 |
| 12 | `AdditiveEngine` | false | 0.8 | 0.8 |
| 13 | `WavetableEngine` | false | 0.6 | 0.6 |
| 14 | `ChordEngine` | false | 0.8 | 0.8 |
| 15 | `SpeechEngine` | false* | −0.7 | 0.8 |
| 16 | `SwarmEngine` | false | −3.0 | 1.0 |
| 17 | `NoiseEngine` | false | −1.0 | −1.0 |
| 18 | `ParticleEngine` | false | −2.0 | 1.0 |
| 19 | `StringEngine` | true | −1.0 | 0.8 |
| 20 | `ModalEngine` | true | −1.0 | 0.8 |
| 21 | `BassDrumEngine` | true | 0.8 | 0.8 |
| 22 | `SnareDrumEngine` | true | 0.8 | 0.8 |
| 23 | `HiHatEngine` | true | 0.8 | 0.8 |

*`ChiptuneEngine` and `SpeechEngine` modify `already_enveloped` dynamically at render time.

A negative `out_gain` or `aux_gain` causes `ChannelPostProcessor` to run `stmlib::Limiter` before scaling, protecting against the unpredictable peak amplitudes of noise-source engines.

---

## Parameter Pre-Processing

### The `Patch` and `Modulations` Structs

`Patch` holds normalized [0, 1] knob values: `note` (MIDI offset), `harmonics`, `timbre`, `morph`, `engine`, `decay`, `lpg_colour`, and three attenuverter amounts (`frequency_modulation_amount`, `timbre_modulation_amount`, `morph_modulation_amount`).

`Modulations` holds live CV values and presence flags. The presence flags (`frequency_patched`, `timbre_patched`, `morph_patched`, `trigger_patched`, `level_patched`) change which modulation path is taken.

### `ApplyModulations()`

```cpp
float value = base_value;
modulation_amount *= max(fabsf(modulation_amount) - 0.05f, 0.05f);
modulation_amount *= 1.05f;

float modulation = use_external_modulation
    ? external_modulation
    : (use_internal_envelope ? envelope : default_internal_modulation);
value += modulation_amount * modulation;
CONSTRAIN(value, minimum_value, maximum_value);
```

The dead-zone logic (`max(|amount| - 0.05, 0.05) * 1.05`) creates a 5% center dead zone around zero on the attenuverter, then re-scales the remaining range to restore full travel. This prevents bleed-through from un-zeroed attenuverters when no CV is connected.

### How Each Parameter Is Computed

**HARMONICS** (`p.harmonics`): additive sum of `patch.harmonics + modulations.harmonics`, clamped to [0, 1]. No envelope routing; always a direct sum.

**NOTE** (`p.note`): base is `patch.note + (modulations.note + previous_note_) * 0.5f`. The note CV is averaged over two consecutive blocks to smooth step changes from sequencers. Then `ApplyModulations()` adds frequency modulation. When a trigger is patched and no freq CV is connected, the internal envelope modulates note: `decay_envelope_.value()^2 * 48.0f` semitones (up to 4 octaves of exponential pitch sweep on attack). For `SpeechEngine` (index 15), this envelope amplitude is further attenuated by a function of `p.harmonics`.

**TIMBRE** (`p.timbre`): `ApplyModulations()` using `patch.timbre_modulation_amount`. Internal envelope source is `decay_envelope_.value()` (linear, not squared). For `ChiptuneEngine` (index 7) with trigger patched and timbre unpatched, the internal envelope amount is zeroed so the engine's built-in arpeggiator envelope handles amplitude.

**MORPH** (`p.morph`): identical routing to TIMBRE. Internal envelope source is `decay_envelope_.value()` (linear).

### The `DecayEnvelope`

```cpp
void DecayEnvelope::Process(float decay) {
    value_ *= (1.0f - decay);
}
```

A pure single-pole exponential decay. Decay coefficient `short_decay` is:

```cpp
(200.0f * kBlockSize) / kSampleRate * SemitonesToRatio(-96.0f * patch.decay)
```

At `patch.decay = 0`: `short_decay ≈ 0.05 * SemitonesToRatio(0) = 0.05` — about 20 blocks (~80 ms) time constant.  
At `patch.decay = 1`: `short_decay ≈ 0.05 * SemitonesToRatio(-96) = 0.05 * 2^-8 ≈ 0.0002` — very slow (~8 s time constant).

The squared value used for note modulation (`value^2`) creates a faster initial transient and longer tail than a pure exponential.

---

## LPG and Envelope System

### `LPGEnvelope` — Vactrol Model

The vactrol is modeled by a single state variable `vactrol_state_` with asymmetric dynamics:

**Attack** (when `vactrol_error > 0`): coefficient = 0.6 per block (very fast, ~2 blocks to near-unity).

**Decay** (when `vactrol_error <= 0`):
```cpp
vactrol_coefficient = short_decay + (1.0f - vactrol_state_^4) * decay_tail;
```
`decay_tail` is derived from the DECAY knob and LPG COLOUR (hf):

```cpp
decay_tail = (20.0f * kBlockSize) / kSampleRate
    * SemitonesToRatio(-72.0f * patch.decay + 12.0f * hf) - short_decay;
```

The `(1 - state^4)` term means the decay is faster when `vactrol_state_` is near 1 (just after attack) and slower as it approaches zero — the same asymptotic behavior seen in physical vactrols (opto-resistors are faster to charge than discharge, and the discharge tail follows an RC curve that slows as resistance increases).

From `vactrol_state_`, three outputs are derived:

- `gain_ = vactrol_state_` (linear amplitude)
- `frequency_ = 0.003 + 0.3 * vactrol_state_^4 + hf * 0.04` — highly nonlinear; cutoff drops off sharply as the vactrol closes because the state enters the `^4` term
- `hf_bleed_ = (tail^2 + (1 - tail^2) * hf) * hf^2` where `tail = 1 - vactrol_state_` — the amount of pre-filter signal blended in to simulate the LPG's characteristic HF bleed at rest

**Trigger mode** (`ProcessPing()`): `vactrol_state_` is ramped upward at a rate proportional to the note frequency (`attack = NoteToFrequency(p.note) * kBlockSize * 2`), then released. Higher notes cause faster attack, mimicking the behavior where short-period signals charge the vactrol more quickly.

**Level mode** (`ProcessLP()`): `level = compressed_level` drives the vactrol directly. `compressed_level = 1.3 * level / (0.3 + |level|)` is a soft sigmoid compression applied first.

### `LowPassGate`

`LowPassGate` implements the audio effect:

```cpp
const float s = *in * gain_modulation.Next();
const float lp = filter_.Process<FILTER_MODE_LOW_PASS>(s);
*out = lp + (s - lp) * hf_bleed;
```

`filter_` is an `stmlib::Svf` (state-variable filter) set to `q = 0.4`. The output is a mixture of the low-passed signal and the unfiltered (but gain-scaled) signal weighted by `hf_bleed`. At low `hf_bleed`, the gate sounds fully filtered (dark, muffled close). At high `hf_bleed` (controlled by LPG COLOUR), the gate passes transients through with frequency-dependent coloring.

The LPG applies to both OUT and AUX identically via the shared `lpg_envelope_` state — both outputs see the same gain, frequency, and hf_bleed envelope.

---

## Engine Descriptions

### Engine 0 — `VirtualAnalogVCFEngine` (VA + VCF)

**Family**: Virtual analog subtractive synthesis.

A single `VariableShapeOscillator` plus a suboscillator at f0/2, fed into a two-stage SVF chain.

**TIMBRE**: Filter cutoff. Mapped as `f0 * SemitonesToRatio((timbre - 0.2) * 120)`. At TIMBRE=0.2, cutoff equals f0. The range spans roughly ±5 octaves from that center.

**MORPH**: Oscillator waveform. Below 0.25: pure saw. 0.25–0.5: morphs saw toward square (via `shape` parameter of `VariableShapeOscillator`). 0.5–0.75: square with increasing pulse-width narrowing. At extremes (|morph − 0.5| > 0.3): sub-oscillator sine fades in at 5× slope.

**HARMONICS**: Filter character. Center range (0.4–0.6) produces no resonance and full two-stage filter. As HARMONICS moves away from 0.5 in either direction, resonance `q` rises as `(2.667 * max(|harmonics - 0.5| - 0.125, 0))^4 * 48`. Below 0.4 HARMONICS, a second SVF stage at the same cutoff is blended in, deepening the rolloff. The gain compensation also adjusts to prevent blowup at high resonance.

**AUX**: High-pass output from the first SVF stage (soft-clipped).

**`already_enveloped`**: false. LPG applies normally.

**Implementation detail**: Two cascaded `stmlib::Svf` instances, the first processed for both LP and HP simultaneously using `Process<FILTER_MODE_LOW_PASS, FILTER_MODE_HIGH_PASS>()`. The LP output is soft-clipped before entering the second stage. The second stage adds warm saturation at high resonance settings.

---

### Engine 1 — `PhaseDistortionEngine` (Phase Distortion / Phase Modulation)

**Family**: Phase distortion, inspired by Casio CZ synthesizers.

The engine runs two instances of a `PhaseDistortionOscillator`-like shaper at 2x oversampling, then naive 2:1 downsampling by averaging pairs.

**TIMBRE**: Phase distortion amount. `amount = 8 * timbre^2 * (1 - modulator_f * 3.8)`. The factor `(1 - modulator_f * 3.8)` reduces the index at high modulator frequencies to prevent aliasing.

**MORPH**: Pulse width of the modulator's asymmetric triangle wave (`pw = 0.5 + morph * 0.49`).

**HARMONICS**: Modulator ratio. Uses the same `lut_fm_frequency_quantizer` table as `FMEngine` to quantize the modulator-to-carrier ratio. The modulator frequency is also constrained to a maximum of 0.25 (Nyquist/2).

**OUT**: Carrier phase distorted by a synced modulator (`shaper_.Render<true, true>()`). The modulator resets in sync with the carrier.

**AUX**: Free-running version — the modulator is not synced to the carrier (`modulator_.Render<false, true>()`). This produces more complex, time-varying spectra since the modulator phase relationship drifts.

**`already_enveloped`**: false.

**Implementation detail**: The engine operates at half the nominal carrier frequency (`f0 = 0.5 * NoteToFrequency(note)`) to compensate for the effective doubling of the waveform period by the 2x oversampling plus halving. The Sine lookup is applied in the downsampler loop: `out[i] = 0.5 * Sine(phase[2i] + 0.25) + 0.5 * Sine(phase[2i+1] + 0.25)`.

---

### Engines 2, 3, 4 — `SixOpEngine` (6-Operator FM, Three Banks)

**Family**: 6-operator DX7-style FM synthesis.

Each of the three engine slots registers the same `SixOpEngine` instance but loads a different user data bank. The `SixOpEngine` holds 32 patches per bank (each packed in the standard Sysex format parsed by `fm::Patch::Unpack()`).

**HARMONICS**: Patch selector. `patch_index_quantizer_.Process(harmonics * 1.02f)` maps [0, 1] to one of 32 patches with hysteresis. The 1.02x scaling ensures the last patch is reachable even with slight ADC undertravel.

**TIMBRE**: `p->brightness` — modulates high-frequency operator levels (operator envelopes respond to brightness in the fm::Voice implementation, rolling off upper partials).

**MORPH**: `p->envelope_control` — compresses all operator envelopes, from full ADSR (morph=0) toward a sustained tone (morph=1). In trigger-unpatched (free-running) mode, MORPH also scrubs the LFO: `lfo_.Scrub(2 * kCorrectedSampleRate * morph)`, letting MORPH manually position within the LFO waveform cycle.

**AUX**: Identical to OUT (`out[i] = aux[i] = SoftClip(temp[i] * 0.25f)`).

**`already_enveloped`**: true. The fm::Envelope inside each operator handles amplitude. The external LPG must not re-envelope an already-enveloped DX patch.

**Implementation detail**: Two `FMVoice` instances round-robin new notes. Each FMVoice wraps an `fm::Voice<6>` (the full 6-operator engine from `dsp/fm/voice.h`) and an `fm::Lfo`. Staggered rendering is used: on each block, one voice renders a block of `size * kNumSixOpVoices` samples into `temp_buffer_` at a time; the results from the previous blocks of the other voice are accumulated in `acc_buffer_`. This spreads the computational load of expensive 6-op FM across multiple blocks, allowing the two voices to run concurrently in the render timeline despite the 12-sample block limitation.

---

### Engine 5 — `WaveTerrainEngine` (Wave Terrain Synthesis)

**Family**: Wave terrain / 2D table lookup.

A circular path oscillates through an (x, y) terrain. The terrain is a 2D surface; the path is a Lissajous circle. The output sample at each point is `Terrain(path_x[t], path_y[t])`.

**TIMBRE**: Radius of the circular path. `radius = 0.1 + 0.9 * timbre * attenuation * (2 - attenuation)` where `attenuation = max(1 - 8 * f0, 0)` keeps amplitude well-behaved at high pitches where a large-radius path would cause steep spectral content.

**MORPH**: X-axis offset. `x_offset = 1.9 * morph - 1.0` shifts the circular orbit horizontally across the terrain, changing which region of the terrain the path traverses. At morph=0 (offset = −1.0) the path is at the left edge; at morph=1 (offset = +1.0) at the right edge.

**HARMONICS**: Terrain selector. Interpolates between 8 (or 9 with user data) terrain surfaces. The terrains are:
- 0–4: Mathematical surfaces defined in `Terrain()` using combinations of Sine and Squash functions
- 5, 6, 7: Wavetable banks re-interpreted as 2D terrains (`TerrainLookupWT()`)
- 8: User-defined terrain loaded via user data (`TerrainLookup()` from a 64×64 int8 grid)

**OUT**: `Terrain(x, y)` sampled along the circular path.

**AUX**: `Sine(1 + 0.5 * (path_y + terrain_value))` — a nonlinear mix of the y-coordinate and the terrain value passed through a sine function, producing FM-like timbres.

**`already_enveloped`**: false.

**Implementation detail**: The path is generated by `QuadratureOscillator::RenderQuadrature()` using the IIR "magic sine" algorithm (2x oversampled). The oversampled path is decimated in the terrain lookup loop. Terrain interpolation between adjacent surfaces uses linear fractional blending of two consecutive `Terrain()` calls.

---

### Engine 6 — `StringMachineEngine` (String Machine Emulation)

**Family**: Divide-down organ/string machine with VCF and ensemble.

A polyphonic chord generator using divide-down oscillators (integer frequency division from a master oscillator), a SVF lowpass filter, and a bucket-brigade-style ensemble chorus.

**HARMONICS**: Chord type (passed to `ChordBank::set_chord()`). Selects from a table of up to-6-note chord ratios.

**TIMBRE**: Filter cutoff and ensemble character. Cutoff = `2.2 * f0 * SemitonesToRatio(120 * timbre)`. Also controls ensemble depth (`depth = 0.35 + 0.65 * timbre`) and ensemble amount (`amount = |timbre - 0.5| * 2`). At timbre=0.5 the ensemble amount is 0 (no chorus). Moving away in either direction increases ensemble effect; moving toward 0 gives dark filtered tones, moving toward 1 gives bright filtered tones.

**MORPH**: Registration (harmonic blend). Maps [0, 1] through an 11-entry registration table, morphing from pure saw through increasingly square waveforms using divide-down oscillators tuned to sub-octaves and overtones.

**OUT / AUX**: Even-indexed chord notes go to OUT, odd-indexed go to AUX. Both are summed and filtered independently by two SVFs (`svf_[0]` at `cutoff`, `svf_[1]` at `cutoff * 1.5`). The two filtered channels are then crossmixed: `out = 0.66 * l + 0.33 * r` and `aux = 0.66 * r + 0.33 * l`, creating a subtle stereo spread before the ensemble processes them.

**`already_enveloped`**: false.

---

### Engine 7 — `ChiptuneEngine` (Chiptune / Arpeggiator)

**Family**: Chiptune/retro game synthesis with arpeggiator.

Uses `SuperSquareOscillator` (a polyBLEP square with selectable overtone structure) for chord voices and `NESTriangleOscillator` for a bass note.

**HARMONICS**: Chord type for the arpeggiator chord bank.

**TIMBRE**: In clocked mode (trigger patched): arpeggiator pattern selector. `arpeggiator_pattern_selector_.Process(timbre)` maps to 12 patterns (4 modes × 3 octave ranges). In free-running mode: controls chord inversion/voicing spread.

**MORPH**: Waveform (`shape = morph * 0.995`). Morphs the `SuperSquareOscillator` through its internal overtone mix.

**OUT**: In clocked mode, a single arpeggiated note. In free-running mode, all chord voices summed with alternating phase signs (`amplitudes[odd] = -amplitudes[odd]`).

**AUX**: Bass oscillator at f0/2 (`bass_.Render(f0 * 0.5 * root_transposition, ...)`).

**`already_enveloped`**: dynamically set to `clocked` (true when trigger is patched, false otherwise). In clocked mode, the engine has an internal amplitude envelope (`envelope_state_ *= decay`) applied to OUT only, with a separately-scaled version for AUX (scaled by `aux_envelope_amount_`). The Voice layer's LPG is bypassed when clocked because the engine is self-enveloping.

**Implementation detail**: The envelope shape is set externally by `Voice::Render()` via `chiptune_engine_.set_envelope_shape(patch.timbre_modulation_amount)` when trigger is patched and timbre CV is unpatched. The modulation amount knob (normally an attenuverter for TIMBRE CV) is repurposed as an envelope decay control. `NO_ENVELOPE = 2` is a sentinel value that disables the internal envelope.

---

### Engine 8 — `VirtualAnalogEngine` (Dual VA Oscillator)

**Family**: Virtual analog with polyBLEP oscillators.

The active firmware variant is `VA_VARIANT == 2`, which pairs a pulse-width square oscillator with a variable-saw oscillator.

**TIMBRE**: Controls the PW of the square/sync oscillator. `square_pw = 1.3 * timbre - 0.15` (clamped to [0.005, 0.5]). When timbre > 0.5, a self-sync ratio appears: `square_sync_ratio = (timbre - 0.5)^2 * 4 * 48` semitones, driving the sync frequency up to 2 octaves above the carrier.

**MORPH**: Controls the variable-saw oscillator. `saw_pw` and `saw_shape` both derive from morph: at morph=0, the saw is a pure sawtooth; at morph=1, it approaches a sine. The gain of each oscillator is also morph-dependent: `saw_gain = 8 * (1 - morph)`, `square_gain = min(timbre * 8, 1)`.

**HARMONICS**: Detune between the two oscillators. `ComputeDetuning()` maps [0, 1] to one of five intervals (unison, fifth, octave, 19th, two octaves) using the `intervals[]` table `{0, 7.01, 12.01, 19.01, 24.01}` semitones, with smooth interpolation via double-smoothstep `Squash(Squash(frac))`.

**OUT**: Normalized sum of the square + saw oscillators.

**AUX**: The difference of two synced instances of `VariableShapeOscillator` at primary and auxiliary frequencies with morph-controlled waveform, producing a spectrally rich alternative.

**`already_enveloped`**: false.

---

### Engine 9 — `WaveshapingEngine` (Waveshaper / Wavefolder)

**Family**: Subtractive-with-waveshaping; bandlimited source into lookup-table waveshaper and analog wavefolder.

Two `VariableShapeOscillator` instances in `OSCILLATOR_SHAPE_SLOPE` mode provide the source signals: a slope (asymmetric triangle/ramp) for OUT and a symmetric triangle for AUX.

**MORPH**: Pulse-width of the slope (`pw = morph * 0.45 + 0.5`). Ranges from symmetric triangle (pw=0.5) to near-sawtooth (pw=0.95).

**HARMONICS**: Waveshaper curve selection. Maps [0, 1] to a position in `lookup_table_i16_table[]`, which contains 4 distinct waveshaping curves (sine, half-rectified, full-rectified, peaked). The `shape_amount_attenuation = Tame(f0, slope, 16)` function limits how far HARMONICS deviates from 0.5 (neutral) at high frequencies to prevent aliasing from the shaper's high-order harmonics.

**TIMBRE**: Wavefolder depth. `wavefolder_gain = timbre`. Applied to `lut_fold` and `lut_fold_2` — two different sigmoid-like folding curves stored in ROM. The `Tame()` function is applied here too with a higher-order spectral estimate to prevent aliasing at high pitches and complex waveforms.

**OUT**: The slope signal after waveshaping then wavefolding, read from `lut_fold` at index `mix * wf_gain + 0.5`.

**AUX**: A triangle-driven sine (`Sine(tri * 0.25 + 0.5)`) crossfaded with `lut_fold_2` as TIMBRE increases (`overtone_gain = timbre * (2 - timbre)`, double-squared for extra nonlinearity).

**`already_enveloped`**: false.

---

### Engine 10 — `FMEngine` (2-Operator FM)

**Family**: Classic 2-operator sine FM with feedback, 4× oversampled.

Integer 32-bit phase accumulators for carrier and modulator. A 4× FIR downsampler (`Downsampler` wrapping `4x_downsampler.h`) handles decimation.

**HARMONICS**: Modulator-to-carrier ratio. Looked up from `lut_fm_frequency_quantizer` (128 entries, [0, 1] mapped to quantized harmonic ratios). This is the same table used by Braids and Rings.

**TIMBRE**: FM index. `amount = 2 * timbre^2 * hf_taming`. Quadratic response gives intuitive feel. `hf_taming = clamp(1 - (modulator_note - 72) * 0.025, 0, 1)^2` reduces index to zero as the modulator approaches MIDI note 112 to prevent aliasing.

**MORPH**: Feedback type. `feedback = 2 * morph - 1` (bipolar). For negative feedback (`feedback < 0`): modulates the modulator's frequency (`phase_feedback = 0.5 * feedback^2`). For positive feedback (`feedback > 0`): self-feedback on the modulator waveform (`modulator_fb = 0.25 * feedback^2`). These are mechanistically different: frequency feedback produces a softer, less chaotic sound than waveform feedback.

**OUT**: Carrier sinusoid modulated by `amount * modulator`.

**AUX**: Sub-oscillator at carrier_phase/2, modulated by `amount * carrier * 0.25`. This is a sub-octave FM tone where the carrier drives sub-oscillator FM, producing a characteristically different harmonic stack.

**`already_enveloped`**: false.

**Implementation detail**: The carrier runs at `note - 24` (two octaves down from nominal) to keep the FM bandwidth reasonable. The FIR filter in `Downsampler` effectively provides 4x oversampling anti-aliasing for both the carrier and sub paths independently.

---

### Engine 11 — `GrainEngine` (Grainlet / VOSIM)

**Family**: Synchronous granular / formant synthesis.

Two `GrainletOscillator` instances plus one `ZOscillator`. `GrainletOscillator` produces single-cycle windows: a carrier oscillator (at f0) resets periodically, and within each cycle the amplitude envelope is a windowed formant sine burst.

**TIMBRE**: Formant frequency. `f1 = NoteToFrequency(24 + 84 * timbre)`. Maps to roughly 30 Hz – 22 kHz formant frequency range.

**HARMONICS**: Two simultaneous effects. (1) Detuning ratio between the two grainlets: `ratio = SemitonesToRatio(-24 + 48 * harmonics)`. (2) Carrier bleed amount: `carrier_bleed = harmonics < 0.5 ? 1 - 2*harmonics : 0`. At harmonics=0, full carrier sine is present and the formant window is minimized. At harmonics=0.5 and above, pure grain texture with no carrier bleed.

**MORPH**: Carrier shape. `carrier_shape = 0.33 + (morph - 0.33) * max(1 - f0*24, 0)`. This blends a carrier from triangular (at morph=0.33) toward sinusoidal at high morph values. The high-frequency rolloff `max(1 - f0*24, 0)` pins the shape to 0.33 at high pitches where nonlinear carrier shapes would alias badly.

**OUT**: Sum of the two GrainletOscillator outputs (summed within the engine), DC-blocked via a high-pass filter at `0.3 * f0`.

**AUX**: `ZOscillator` output, a more complex VOSIM-like synthesis that modulates the formant parameter across a Z-plane. DC-blocked separately.

**`already_enveloped`**: false.

---

### Engine 12 — `AdditiveEngine` (Additive Synthesis, 32 Partials)

**Family**: Additive synthesis with spectral envelope control.

Three `HarmonicOscillator` instances, each a bank of cosine oscillators using the CORDIC-like fast cosine algorithm from `stmlib::CosineOscillator`. 24 harmonics on OUT (handled by two HarmonicOscillator instances, partials 1–12 and 13–24), 8 organ-stop partials on AUX.

The spectral envelope `UpdateAmplitudes()` distributes energy using a tent function centered at `centroid` with slope `slope` and periodic bumps `bumps`:

```
gain(i) = relu(1 - |i - center| * slope)^4 * (1 + Sine(0.25 + |i-center| * bumps))^4
```

The result is normalized to unit sum. A per-partial ONE_POLE filter (`ONE_POLE(amplitudes[j], gain, 0.001f)`) low-pass filters each amplitude to prevent clicks from parameter jumps.

**TIMBRE**: Spectral centroid (`centroid = timbre`). Sweeps the energy peak from the fundamental (timbre=0) through all 24 harmonics (timbre=1).

**MORPH**: Spectral slope sharpness. `raw_slope = (1 - 0.6 * harmonics) * morph`. Cubed: `slope = 0.01 + 1.99 * raw_slope^3`. At morph=0, slope~=0.01 and all harmonics have nearly equal amplitude (buzz). At morph=1, slope~=2.0 and only one or two harmonics near the centroid are audible (sine-like).

**HARMONICS**: Spectral formant bumps. `bumps = 16 * harmonics^2`. Adds periodic resonant peaks to the spectral envelope, spaced every `1/bumps` harmonics. Also slightly reduces slope effectiveness: `raw_slope *= (1 - 0.6 * harmonics)`.

**OUT**: 24-harmonic additive synth.

**AUX**: 8-partial organ-stop synthesis using `organ_harmonics[] = {0,1,2,3,5,7,9,11}` (pipes at 1', 2', 4', 8', 2⅔', 1⅓', ⅔', ¼').

**`already_enveloped`**: false.

---

### Engine 13 — `WavetableEngine` (8×8×3 Wavetable Cube)

**Family**: Trilinear-interpolated wavetable synthesis.

A 3D wavetable space (8 waves × 8 waves × 3 banks = 192 wavetables, each 128 samples stored in integrated form in `wav_integrated_waves[]`). The fourth bank (indices 192–255) is either user-supplied or a pseudo-random selection.

The three axes are independently low-pass filtered before indexing: `x_lp_`, `y_lp_`, `z_lp_` track `x_pre_lp_`, `y_pre_lp_`, `z_pre_lp_` via one-pole filters. The cutoff of these inner filters is `lp_coefficient = min(max(2 * f0 * (4 - 3 * quantization), 0.01), 0.1)`, which ties wavetable interpolation smoothness to pitch.

**TIMBRE**: X axis of the wavetable cube (first dimension, within-bank horizontal). Low-pass filtered with coefficient ~0.2.

**MORPH**: Y axis (second dimension, within-bank vertical). Low-pass filtered with coefficient ~0.2.

**HARMONICS**: Z axis (bank selector, cross-bank interpolation). Low-pass filtered with coefficient ~0.05 (much slower, for smooth bank morphing). Also controls quantization: `quantization = clamp(harmonics*7 - 3, 0, 1)`. When harmonics > 3/7, the fractional parts of x, y are pulled toward a hard grid (`Clamp(frac, 16)` pulls toward {0, 0.0625, 0.125, ...}), creating a discontinuous "digital" texture.

**OUT**: Trilinear interpolation across 8 surrounding wavetable corners, processed through `diff_out_` (a differentiator that recovers the original waveform from the integrated storage) scaled by `gain = (1 / (f0 * 131072)) * (0.95 - f0)`.

**AUX**: `static_cast<float>(static_cast<int>(out * 32)) / 32.0f` — a 5-bit truncation of the OUT signal, creating bit-crush quantization noise for a lo-fi contrast.

**`already_enveloped`**: false.

The Z axis folding logic: `if z >= 4: z = 7 - z` mirrors the upper half of the bank index back, so the interpolation path goes 0→1→2→3→2→1→0 as HARMONICS sweeps from 0 to 1, creating a palindromic wavetable journey.

---

### Engine 14 — `ChordEngine` (Chord Generator)

**Family**: 5-voice divide-down organ / wavetable chord synthesizer.

Five simultaneous voices at chord-defined intervals from f0. Each voice crossfades between a `DivideDownVoice` (at low MORPH) and a `WavetableVoice` (at high MORPH), with independent fade points staggered as `{0.55, 0.47, 0.49, 0.51, 0.53}` to prevent all voices from clicking simultaneously.

**HARMONICS**: Chord type (via `ChordBank::set_chord()`). Accesses a table of up to-6-voice chord ratios (major, minor, suspended, etc.).

**TIMBRE**: Chord inversion and voicing spread. Passed to `ComputeChordInversion()`, which reorganizes and spreads the chord notes across octaves. Also assigns `aux_note_mask` — a bitmask determining which voices route to AUX vs. OUT.

**MORPH**: Timbre character (low MORPH = organ registration, high MORPH = wavetable). `registration = max(1 - morph * 2.15, 0)` selects from an 8-entry registration table. Above morph=0.535, wavetable voices fade in with `waveform = max((morph - 0.535) * 2.15, 0)` selecting across 15 hardcoded wavetable entries from `wav_integrated_waves`.

**OUT**: All chord voices summed, with AUX voices added back in (for complete chord audibility on OUT too).

**AUX**: The AUX-assigned chord notes × 3 — louder to compensate for fewer voices.

**`already_enveloped`**: false.

**Implementation detail**: `DivideDownVoice` uses integer frequency division from a reference phase accumulator, with gain shaped by multiple harmonic amplitudes from the registration table. This produces the characteristic additive timbre of 1960s string/organ machines (Solina, Eminent) without independent oscillators per voice.

---

### Engine 15 — `SpeechEngine` (Formant / LPC Speech)

**Family**: Speech synthesis via three interpolated algorithms.

HARMONICS selects and blends between synthesis models along a 0–1 axis divided into regions by `group = harmonics * 6`:

- `group` 0–1: Naive formant synthesis (`NaiveSpeechSynth`) → SAM (`SamSpeechSynth`)
- `group` 1–2: SAM → LPC continuous vowel (`LpcSpeechSynthController`, word_bank = −1)
- `group` 2–6: LPC word-bank playback (4 word banks, selected by `word_bank_quantizer_`)

**TIMBRE**: Formant character / spectral brightness (passed as `parameters.timbre` to each synth).

**MORPH**: Phoneme/vowel position or formant crossfade character (passed as `parameters.morph`).

**OUT**: Main voice signal (glottal buzz + formants).

**AUX**: Noise excitation or secondary formant signal (model-dependent).

**`already_enveloped`**: Dynamically set. In the naive/SAM/LPC-vowel region: `*already_enveloped = false` — the LPG envelopes the continuous vowel. In the word-bank LPC region with trigger patched: `*already_enveloped = replay_prosody = true` — the word playback carries its own amplitude profile.

**Voice-layer special handling**: When engine_index == 15, the internal envelope amplitude for note modulation is reduced: `internal_envelope_amplitude = clamp(2 - harmonics * 6, 0, 1)`. This fades out pitch modulation by the internal envelope as HARMONICS moves into the word-bank region. The FM modulation amount knob is repurposed as prosody intensity (`speech_engine_.set_prosody_amount()`), and the morph modulation amount knob as playback speed (`speech_engine_.set_speed()`).

---

### Engine 16 — `SwarmEngine` (Swarm of Oscillators)

**Family**: Stochastic/granular — 8 detuned grain voices.

`kNumSwarmVoices = 8` `SwarmVoice` instances, each containing a `GrainEnvelope`, an `AdditiveSawOscillator`, and a `FastSineOscillator`. Each voice has a `rank_` from −1 to +1.

Each `GrainEnvelope` generates a grain phase that advances at `density * fm_` per block, where `fm_` is a randomized grain rate (0.5–2.0 in free-running mode, decaying in burst mode). On each grain boundary, a random new target frequency within spread around f0 is chosen.

**TIMBRE**: Grain density. `density = NoteToFrequency(timbre * 120) * 0.025 * block_size`. Exponential mapping ensures extreme low densities (long grains / slow modulation) and extreme high densities (textural buzz).

**HARMONICS**: Pitch spread. `spread = harmonics^3`. Each voice's frequency deviates from f0 by: `f0 * SemitonesToRatio(48 * expo_amount * spread * rank)` (exponential) plus `rank * (rank + 0.01) * spread * 0.25 * f0` (linear).

**MORPH**: Grain size. `size_ratio = 0.25 * SemitonesToRatio((1 - morph) * 84)`. Decreases exponentially as morph increases. Each voice applies a slightly smaller size_ratio (`size_ratio *= 0.97^i`). When size_ratio < 1: overlapping grains, continuous texture. When size_ratio >= 1: discrete windowed grains with sine-envelope amplitude shaping.

**OUT**: Summed `AdditiveSawOscillator` outputs (saw texture).

**AUX**: Summed `FastSineOscillator` outputs (sine texture).

**`already_enveloped`**: false. Both `out_gain` and `aux_gain` are negative (−3.0, 1.0) so the limiter path is used for OUT.

**Burst mode**: When trigger is patched, voices switch to burst mode. On rising edge, all grains restart immediately (`phase_ = 0.5, fm_ = 16`) and the grain rate decays by 0.8× per grain, producing a dense-to-sparse sweep.

---

### Engine 17 — `NoiseEngine` (Clocked Noise + Multimode Filter)

**Family**: Clocked noise / pitched noise.

Two `ClockedNoise` instances (noise re-seeded at a programmable clock frequency) drive a multimode SVF and two bandpass filters.

**TIMBRE**: Clock frequency. `clock_f = NoteToFrequency(timbre * (128 - clock_lowest_note) + clock_lowest_note)`. In unpatched-trigger mode, the clock can go sub-audio (clock_lowest_note = 0). With trigger patched, clock_lowest_note = −24 (goes lower, allowing very low-rate sample-and-hold behavior). The gain applied to the filtered output is `1 / sqrt((0.5 + q) * 40 * f0)` to normalize amplitude across frequency.

**HARMONICS**: Filter cutoff ratio between the two noise paths, and LP/HP blend for OUT. `f1 = NoteToFrequency(note + harmonics * 48 - 24)` — a secondary clock derived from f0 offset by ±24 semitones. The mode parameter `mode_modulation` drives `ProcessMultimodeLPtoHP()` which sweeps the SVF from LP to HP as harmonics sweeps 0→1.

**MORPH**: Filter Q. `q = 0.5 * SemitonesToRatio(morph * 120)`. Exponential mapping; at morph=0.5 this is 0.5 (gentle), at morph=1 this is large (resonant to self-oscillating).

**OUT**: `lp_hp_filter_` output in multimode mode (LP at harmonics=0, HP at harmonics=1).

**AUX**: Sum of two `bp_filter_` bandpass outputs: one centered at f0 (driven by first noise), one at f1 (driven by second noise). This creates a pitched spectral mixture for harmonic noise textures.

**`already_enveloped`**: false. Both gains are negative (limiter path).

**Sync**: On trigger rising edge, both `ClockedNoise` instances reseed simultaneously, creating a rhythmic noise burst.

---

### Engine 18 — `ParticleEngine` (Pitched Noise Particles)

**Family**: Stochastic resonator / pitched noise via `kNumParticles` bandpass impulse particles.

Each `Particle` fires impulses at a stochastic rate and filters them with a high-Q resonator tuned near f0.

**TIMBRE**: Particle density. `density_sqrt = NoteToFrequency(60 + timbre^2 * 72)`, then `density = density_sqrt^2 / kNumParticles`. Quadratic response mapped through `NoteToFrequency` gives perceptual linearity.

**HARMONICS**: Spectral spread. `spread = 48 * harmonics^2` — maximum spread of ±24 semitones for each particle's resonant frequency.

**MORPH**: Dual control with a split at 0.5:
- Below 0.5: diffusion amount. `diffusion = raw_diffusion = (2 * |morph - 0.5|)^2`. Applies `Diffuser::Process()` (an allpass diffuser from `dsp/fx/`) with feedback `0.8 * diffusion^2` and delay `0.5 * diffusion + 0.25` — creates reverberant washing of the particle cloud.
- Above 0.5: filter Q. `q = 0.5 + SemitonesToRatio((morph - 0.5) * 120)^2` — resonance rises exponentially. Above morph=0.5 the diffusion is zero (dry).

**OUT**: Sum of all particle outputs, post-processed by a lowpass filter at f0 (post_filter_). With diffusion on AUX: same as OUT but with diffuser applied.

**AUX**: Sum of all particle aux outputs (secondary resonator outputs per particle).

**`already_enveloped`**: false. `out_gain = −2.0` (limiter path).

---

### Engine 19 — `StringEngine` (Karplus-Strong Strings, 3 Voices)

**Family**: Extended Karplus-Strong physical model (from Mutable Instruments Rings).

Three `StringVoice` instances from `dsp/physical_modelling/`. Each StringVoice contains a delay line for the feedback loop and per-voice parameters for brightness, damping, and nonlinearity.

**HARMONICS**: Structure / brightness. Passed directly to `voice_[i].Render(..., harmonics, ...)`. Controls the amount of high-frequency content in the excitation and in the loop filter.

**TIMBRE**: Brightness (squared: `timbre * timbre`). Controls the loop filter cutoff — higher timbre gives brighter, more sustained tone.

**MORPH**: Damping / nonlinearity. Controls string damping and the saturation character of the loop filter.

**OUT / AUX**: Both produced by the StringVoice render (OUT gets the direct string signal, AUX a body resonance simulation).

**`already_enveloped`**: true. Each StringVoice applies its own amplitude envelope (the decay of the delay-line feedback loop).

**Voice stealing**: On trigger rising edge, the current note's frequency is read from a 14-sample delay line (`f0_delay_.Read(14)` — delayed to avoid clicking on rapid retriggering) and assigned to the next round-robin voice, while `active_string_` advances. This creates Rings-style polyphony where previous notes continue to ring out.

**`out_gain = −1.0`**: Limiter path active.

---

### Engine 20 — `ModalEngine` (Modal Resonator)

**Family**: Modal synthesis / resonator bank from Mutable Instruments Rings.

A single `ModalVoice` containing a `Resonator` — a bank of SVF-based resonators, each tuned to a partial frequency defined by the `harmonics` parameter.

**HARMONICS**: Partial ratios / inharmonicity. Controls how the resonant frequencies deviate from harmonic series. Low HARMONICS: near-harmonic (bell-like). High HARMONICS: inharmonic (metallic, xylophone-like). A one-pole filter (`harmonics_lp_`) smooths rapid changes at rate 0.01 per block.

**TIMBRE**: Excitation brightness / attack sharpness.

**MORPH**: Damping / decay character.

**OUT / AUX**: The modal resonator output (OUT), plus a second signal from the ModalVoice's internal excitation path (AUX).

**`already_enveloped`**: true. The resonator bank is excited by a click/impulse and decays naturally. No external envelope is needed.

**`out_gain = −1.0`**: Limiter path active.

---

### Engine 21 — `BassDrumEngine` (808 + Synthetic Bass Drum)

**Family**: Analog circuit model + digital synthesis model.

Blends `AnalogBassDrum` (from `dsp/drums/`) — an 808-derived model using a sine oscillator with exponential pitch sweep and attack FM — with `SyntheticBassDrum` — a triangle oscillator with a saturation circuit.

**TIMBRE**: Attack character for both models.

**MORPH**: Decay and pitch envelope shape.

**HARMONICS**: Controls the crossfade between models. Below 0.25: pure analog model (attack FM only). 0.25–0.5: analog model + self-FM adds in. 0.5–1.0: overdrive via `Overdrive::Process()` on the analog path, and the synthetic model mixes in on AUX.

Specific mappings:
- `attack_fm_amount = clamp(harmonics * 4, 0, 1)` — how much initial pitch jump at attack
- `self_fm_amount = clamp(harmonics * 4 - 1, 0, 1)` — self-FM for more aggressive click
- `drive = clamp(harmonics * 2 - 1, 0, 1) * clamp(1 - 16 * f0, 0, 1)` — overdrive amount (pitch-dependent: disabled at high pitches)

**OUT**: Analog bass drum path with optional overdrive.

**AUX**: Synthetic bass drum model.

**`already_enveloped`**: true. Both models are self-enveloping (decay driven by exponential coefficient).

---

### Engine 22 — `SnareDrumEngine` (808 + Synthetic Snare)

**Family**: Dual snare drum model.

`AnalogSnareDrum` (from `dsp/drums/`) models an 808 snare: a tuned resonator body with a bridged noise source (snare rattle). `SyntheticSnareDrum` uses a more digitally-natured approach.

All four parameters (TIMBRE, MORPH, HARMONICS, NOTE) are passed directly to both models identically. The models interpret them in their own way:

- NOTE: Snare fundamental frequency
- TIMBRE: Tonal/noise balance, attack brightness
- MORPH: Body decay
- HARMONICS: Snare rattle / overtone balance

**OUT**: `AnalogSnareDrum` output.

**AUX**: `SyntheticSnareDrum` output.

**`already_enveloped`**: true.

---

### Engine 23 — `HiHatEngine` (808 + Metallic Hi-Hat)

**Family**: Dual hi-hat model.

Two hi-hat implementations (`hi_hat_1_`, `hi_hat_2_`) from `dsp/drums/`. The 808-style hi-hat uses six detuned square oscillators whose sum passes through a bandpass filter and VCA; the metallic hi-hat uses a different noise-and-resonator approach.

All parameters are passed identically to both instances:

- NOTE: Fundamental frequency of the hat
- TIMBRE: Cutoff / brightness
- MORPH: Decay length
- HARMONICS: Metallic resonance / overtone content

**OUT**: First hi-hat model (808 style).

**AUX**: Second hi-hat model (metallic).

**`already_enveloped`**: true.

**Implementation detail**: Both hi-hat renderers write into the same `temp_buffer_` (split into two halves: `temp_buffer_` and `temp_buffer_ + size`) for intermediate processing, with final output written to separate `out` and `aux` buffers.

---

## `ChannelPostProcessor` and Output Scaling

`ChannelPostProcessor` sits between the engine float output and the DAC `short` output. For each channel:

1. If `gain < 0`: run `limiter_.Process(-gain, in, size)` — `stmlib::Limiter` applies a look-ahead peak limiter with the absolute value of `out_gain` as the drive.
2. `post_gain = (gain < 0 ? 1.0 : gain) * -32767.0` — the negation inverts the signal polarity (DAC convention).
3. If LPG bypass: `*out = Clip16(1 + (int32_t)(*in * post_gain))` — straight scale and clip.
4. If not bypass: `lpg_.Process(post_gain * lpg_gain, lpg_frequency, hf_bleed, in, out, size, stride)` — `LowPassGate` applies envelope-controlled low-pass and HF bleed simultaneously.

The stride of 2 writes interleaved stereo shorts (`Frame* frames` where `Frame = { short out; short aux; }`).

---

## Oscillator Primitives

The `dsp/oscillator/` directory provides the fundamental building blocks:

- **`VariableShapeOscillator`**: polyBLEP bandlimited oscillator. The `waveshape` parameter controls triangle/saw/square blend continuously. Supports master/slave hard sync (separate slave resets on master zero-crossing). Used by `VirtualAnalogEngine`, `WaveshapingEngine`, `VirtualAnalogVCFEngine`.

- **`VariableSawOscillator`**: variable-slope sawtooth with polyBLEP. Used by `VirtualAnalogEngine`.

- **`HarmonicOscillator`**: bank of cosine oscillators using fast IIR cosine generation (2 multiplies per partial per sample). Template parameter `first_harmonic_index` allows partials 1–12 and 13–24 to split across instances. Used by `AdditiveEngine`.

- **`GrainletOscillator`**: produces grain windows as described under engine 11. Used by `GrainEngine`.

- **`ZOscillator`**: Z-plane oscillator producing VOSIM-like formant peaks. Used by `GrainEngine` for AUX.

- **`SuperSquareOscillator`**: chiptune-style square with selectable overtone series. Used by `ChiptuneEngine`.

- **`NESTriangleOscillator`**: NES-accurate triangle wave (4-bit quantized). Used by `ChiptuneEngine` bass.

- **`SineOscillator`** / **`FastSineOscillator`**: IIR sine approximation. Used pervasively.

- **`StringSynthOscillator`**: divide-down oscillator for `StringMachineEngine` and `ChordEngine`.

---

## Pitch Conversion

`NoteToFrequency()` (`dsp/engine/engine.h`):

```cpp
inline float NoteToFrequency(float midi_note) {
    midi_note -= 9.0f;
    CONSTRAIN(midi_note, -128.0f, 127.0f);
    return a0 * 0.25f * stmlib::SemitonesToRatio(midi_note);
}
```

The −9 semitone offset moves the MIDI convention so that MIDI note 0 corresponds to A-9 (below the range of a standard piano). `a0 = (440/8) / 47872.34 ≈ 1.149e-3` normalized. The 0.25 factor shifts the reference down two octaves. All oscillators accept normalized frequencies (phase increment per sample in [0, 0.5]).

---

## Memory Layout

With `kMaxBlockSize = 24`:

- `out_buffer_[24]` and `aux_buffer_[24]` are flat float arrays in `Voice`. They are passed directly to engine `Render()` and then to `ChannelPostProcessor`.
- The `BufferAllocator` arena is external (provided by the firmware's RAM pool). Its size must accommodate the largest single-engine allocation — in practice, `StringEngine` with 3 Karplus-Strong delay lines dominates.
- `trigger_delay_line_[kMaxTriggerDelay]` is 8 floats of static delay for the trigger pipeline.
- All other engine state (oscillator phases, filter states, LFO state) lives in the engine member variables inside `Voice`, which are concrete objects — the compiler lays them out sequentially in static RAM.

---

## Modulation Routing Summary

| CV Input | Patched behavior | Unpatched behavior |
|----------|-----------------|-------------------|
| FREQ (V/oct) | Directly modulates note pitch | No external freq mod |
| TIMBRE CV | Replaces internal envelope for TIMBRE | Internal `DecayEnvelope` (if trigger patched) or no modulation |
| MORPH CV | Replaces internal envelope for MORPH | Internal `DecayEnvelope` (if trigger patched) or no modulation |
| TRIGGER | Rising edge fires envelopes; bitmask in `p.trigger` | `TRIGGER_UNPATCHED` bit set; engines use sustain/continuous mode |
| LEVEL | `ProcessLP(compressed_level, ...)` — continuous vactrol CV | If trigger patched: `ProcessPing()` — trigger fires vactrol |
| ENGINE CV | Latched on trigger rising edge; otherwise tracked | Continuously tracked |

The `ApplyModulations()` dead-zone ensures that a CV attenuverter set near zero but with a hot CV input does not bleed through. The squaring of `modulation_amount` (`max(|amt| - 0.05, 0.05) * 1.05`) provides the attenuverter's characteristic non-linear taper: sensitive near zero and stronger at the extremes.
