# Plaits

Plaits is a Mutable Instruments macro-oscillator Eurorack module — the successor to Braids. Rather than exposing individual synthesis parameters, it provides a curated set of 24 synthesis "engines," each accessible by turning a single knob. A common four-parameter interface (HARMONICS, TIMBRE, MORPH, plus V/oct pitch) controls every engine, though what each parameter does is engine-specific. The module always produces two simultaneous audio outputs: the primary OUT and an auxiliary AUX, which carries a related but spectrally complementary signal.

---

## Code Architecture

### Directory and File Structure

```
plaits/
  plaits.cc               # Firmware entry point: hardware init, main loop
  ui.cc / ui.h            # UI state machine (knobs, buttons, LEDs)
  pot_controller.h        # Smoothing and scaling for pot readings
  settings.cc / settings.h # Persistent user settings (EEPROM)
  resources.cc / resources.h # ROM lookup tables (wavetables, LUTs)

  dsp/
    dsp.h                 # kSampleRate, kBlockSize, a0 (global constants)
    voice.cc / voice.h    # The Voice class — top-level synthesis orchestrator
    envelope.h            # LPGEnvelope and DecayEnvelope

    engine/               # Original 16 engines (Braids successor era)
    engine2/              # 8 additional engines added in firmware v2

    oscillator/           # Bandlimited primitive oscillators
    physical_modelling/   # Karplus-Strong string, modal resonator
    speech/               # LPC, SAM, naive formant speech synthesizers
    drums/                # 808/synthetic bass drum, snare, hi-hat primitives
    noise/                # Clocked noise, dust, fractal and smooth random
    chords/               # Chord ratio bank
    fm/                   # Full 6-operator DX-style FM engine (fm::Voice<6>)
    fx/                   # LowPassGate, Diffuser, Ensemble, Overdrive
    downsampler/          # 4x FIR downsampler used by the FM engine
```

### The `Voice` Class

`Voice` (in `dsp/voice.h` and `dsp/voice.cc`) is the single top-level object that owns every engine instance and all post-processing state. It is allocated statically in the firmware and driven from the main render loop.

All 24 engine instances are stored as concrete member variables inside `Voice` — no heap allocation, no vtable lookup array. The `EngineRegistry<kMaxEngines>` is a flat array of `Engine*` pointers populated once in `Voice::Init()`. Engines share a single scratch RAM region by each calling `allocator->Free()` before their own `Init()`:

```cpp
for (int i = 0; i < engines_.size(); ++i) {
    allocator->Free();         // rewind the arena
    engines_.get(i)->Init(allocator);
}
```

This means engines cannot run simultaneously; they share whatever dynamic RAM (delay lines, grain buffers, wavetable maps) they need.

`Voice::Render()` is called once per audio block (12 samples at 48 kHz, i.e. ~4 ms). Its responsibilities are:

1. Delay the TRIG input by 5 samples (`trigger_delay_`) to absorb CV-gate skew from sequencers.
2. Detect rising edges and fire the internal `DecayEnvelope` and `LPGEnvelope`.
3. Select an engine via `engine_quantizer_` (a hysteresis quantizer on the PATCH.engine + engine CV sum).
4. Call `LoadUserData()` and `Reset()` on engine change.
5. Compute the final `EngineParameters` struct, mixing pot values, CV, and internal envelope.
6. Call `engine->Render(p, out_buffer_, aux_buffer_, size, &already_enveloped)`.
7. Post-process both OUT and AUX through `ChannelPostProcessor`, which applies either the LPG or a straight gain/clip path.

### The Engine Interface

All engines inherit from the abstract `Engine` base class (`dsp/engine/engine.h`):

```cpp
class Engine {
  virtual void Init(stmlib::BufferAllocator* allocator) = 0;
  virtual void Reset() = 0;
  virtual void LoadUserData(const uint8_t* user_data) = 0;
  virtual void Render(const EngineParameters& parameters,
                      float* out, float* aux, size_t size,
                      bool* already_enveloped) = 0;
  PostProcessingSettings post_processing_settings;
};
```

`EngineParameters` carries the resolved, modulated synthesis parameters: `note` (MIDI), `timbre`, `morph`, `harmonics`, `accent`, and `trigger` (a bitmask of `TRIGGER_LOW`, `TRIGGER_RISING_EDGE`, `TRIGGER_HIGH`, and `TRIGGER_UNPATCHED`).

`PostProcessingSettings` tells the voice layer two things: the output gain for OUT and AUX (a negative value signals that a limiter rather than a multiplier should be used), and `already_enveloped` — a flag that, when true, causes the LPG to be bypassed entirely. Engines that contain their own amplitude envelope (string, modal, drums, 6-op FM, chiptune in clocked mode) set this flag so the external LPG does not double-envelope them.

### Engine Selection and Quantization

The 24 engines are registered in `Voice::Init()` in a specific order. Engines 0–7 are the `engine2/` additions; engines 8–23 are the original 16. The engine index is computed by `engine_quantizer_.Process(patch.engine, engine_cv_)`, which implements hysteresis to prevent jitter at boundaries when modulating the engine CV. The engine CV is latched on the rising edge of the trigger if the trigger input is patched; otherwise it tracks continuously.

### The Patch and Modulations Structs

Two structs decouple the hardware layer from the synthesis layer:

- `Patch` holds knob values: `note`, `harmonics`, `timbre`, `morph`, `engine`, `decay`, `lpg_colour`, and three modulation amount knobs (`frequency_modulation_amount`, `timbre_modulation_amount`, `morph_modulation_amount`).
- `Modulations` holds CV input values and patch presence flags: `engine`, `note`, `frequency`, `harmonics`, `timbre`, `morph`, `trigger`, `level`, and booleans `frequency_patched`, `timbre_patched`, `morph_patched`, `trigger_patched`, `level_patched`.

The `ApplyModulations()` inline function in `Voice` selects between three modulation sources: if the relevant CV input is patched it uses the raw CV; if a trigger is patched it uses the `DecayEnvelope` value; otherwise a default internal value is used. A dead zone and gain squaring on `modulation_amount` produce the typical attenuverter feel.

---

### Key Synthesis Models

#### Virtual Analog Oscillators

Two engines: `VirtualAnalogEngine` (engine 8) and `VirtualAnalogVCFEngine` (engine 0).

`VirtualAnalogEngine` uses `VariableShapeOscillator` — a polyBLEP bandlimited oscillator that continuously morphs between triangle, saw, and square via a `waveshape` parameter, with variable pulse-width and optional hard sync to a master oscillator. In the active variant (`VA_VARIANT == 2`), TIMBRE controls a PW-square oscillator with self-sync, MORPH controls a `VariableSawOscillator`, and HARMONICS detunes the two oscillators against each other using a quantized interval table (unison, fifth, octave, 19th, two octaves). AUX outputs a synced pair.

`VirtualAnalogVCFEngine` adds a suboscillator and a two-stage SVF (state-variable filter). TIMBRE sweeps filter cutoff (tracking the pitch), HARMONICS controls resonance and a second filter stage gain, and MORPH morphs the oscillator waveform from saw through square with a sub-octave sine on the extremes. OUT carries the low-pass output; AUX carries the high-pass.

#### Waveshaping Engine

`WaveshapingEngine` (engine 9) starts from a bandlimited slope/asymmetric triangle generated by `VariableShapeOscillator` in slope mode. MORPH sets pulse-width asymmetry of the slope. HARMONICS selects among four waveshaper curves (stored in `lookup_table_i16_table[]`), interpolated to create a continuous morph. TIMBRE controls a wavefolder applied after the shaper, with an aliasing-aware gain taper (`Tame()`) that reduces fold depth at high pitches. AUX is a different wavefold function applied to a pure sine, crossfading toward the folded output as TIMBRE increases.

#### FM Engines

`FMEngine` (engine 10) is a classic 2-operator sine FM with 4x oversampling and a 4-tap FIR downsampler. The carrier and modulator run as 32-bit integer phase accumulators. HARMONICS selects the modulator-to-carrier ratio from a quantized lookup table (`lut_fm_frequency_quantizer`). TIMBRE controls FM index (quadratic response, anti-alias attenuated at high pitches). MORPH controls feedback: negative feedback goes to the modulator's phase, positive feedback is applied to the modulator waveform itself. OUT is the carrier; AUX is a sub-oscillator at half the carrier frequency modulated by the carrier.

`SixOpEngine` (engines 2–4) is a full 6-operator DX-style FM synth built from the `dsp/fm/` subsystem. `fm::Voice<6>` renders up to 32 DX7-compatible operator algorithms defined in `fm::Algorithms<6>`. Two voices (`FMVoice`) are round-robined for polyphony. HARMONICS selects among up to 32 patches loaded per bank from ROM or user data. TIMBRE and MORPH modulate operator levels and parameters in a patch-defined way. The engine sets `already_enveloped = true` because the FM operators carry their own per-operator envelopes (`fm::Envelope`).

#### Formant / Speech Engines

`SpeechEngine` (engine 15) crossfades among three speech synthesis paradigms controlled by HARMONICS:

- **Naive formant synth** (`NaiveSpeechSynth`): a simple formant oscillator approach.
- **SAM** (`SamSpeechSynth`): a port of the Software Automatic Mouth algorithm.
- **LPC** (`LpcSpeechSynthController`): linear predictive coding with stored phoneme and word banks.

TIMBRE controls the formant character / "mouth shape" and MORPH controls pitch inflection or phoneme crossfade. When HARMONICS exceeds 0.33 (entering the LPC word-replay region), the engine sets `already_enveloped = true` and gates via the trigger input to replay complete words with embedded prosody. The FM modulation amount and morph modulation amount knobs are repurposed as prosody intensity and playback speed controls.

#### Granular / Grain Models

`GrainEngine` (engine 11) uses `GrainletOscillator` — a sync-based oscillator that multiplies a phase-distorted carrier sine envelope by a freely-running formant sine. The carrier resets synchronously at the pitch frequency while the formant runs at an independently set frequency. This creates a single-cycle window of the formant sine, producing VOSIM-like spectral peaks. Two grainlets are mixed on OUT; the `ZOscillator` (which further modulates the formant using a Z-plane approach) runs on AUX.

`SwarmEngine` (engine 16) runs `kNumSwarmVoices` grain voices, each an independent low-frequency oscillator that fires either continuously (free density determined by TIMBRE) or in a burst (one burst per trigger). HARMONICS controls pitch spread, MORPH controls grain size, TIMBRE controls grain density.

`ParticleEngine` (engine 18) generates pitched noise using `Particle` objects — each particle is a bandpass-filtered impulse train at a stochastic frequency near the root note. HARMONICS controls spectral spread, TIMBRE controls density, MORPH below 0.5 adds allpass diffusion (`Diffuser`), above 0.5 raises filter Q into resonant territory.

#### Physical Modelling

`StringEngine` (engine 19) maintains `kNumStrings` voices, each backed by a `StringVoice` — an extended Karplus-Strong model inherited from the Rings module. The delay line length sets the fundamental frequency; a one-pole loop filter controls brightness; damping and structure parameters shape the string's character. On a trigger, a burst of spectrally shaped noise excites the string; in unpatched (sustain) mode, continuous noise re-excites it. Three string voices are voice-stolen in round-robin fashion, so chords sustain independently.

`ModalEngine` (engine 20) uses `ModalVoice`, a modal resonator with `Resonator` — a bank of SVF biquad resonators tuned to inharmonic partial ratios (set by HARMONICS). A click exciter is low-pass filtered and injected into the resonator bank. The engine is self-enveloping (sets `already_enveloped = true`).

#### Percussion / Noise Models

`BassDrumEngine` (engine 21) blends an analog-style 808 kick model (`AnalogBassDrum` — a sine oscillator with exponential pitch and amplitude decay, plus attack FM) with a synthetic model (`SyntheticBassDrum` — a triangle oscillator with saturation). HARMONICS morphs between them and introduces overdrive (`Overdrive`). OUT is the analog path, AUX is the synthetic path.

`SnareDrumEngine` (engine 22) and `HiHatEngine` (engine 23) follow the same pattern: a physical model for OUT and a synthetic equivalent for AUX.

`NoiseEngine` (engine 17) generates clocked noise (`ClockedNoise`) — a noise source re-seeded at a clock frequency set by TIMBRE — then passes it through an SVF in multimode configuration (HARMONICS morphs LP to HP). A second clocked noise instance at a harmonically related clock feeds a pair of bandpass filters to AUX.

---

### The LPG and Internal Envelope

When the trigger input is patched, `Voice::Render()` fires both `DecayEnvelope` (a single-pole exponential decay, used for parameter modulation) and `LPGEnvelope` (used for audio amplitude and filter). When the LEVEL input is patched instead of TRIGGER, `LPGEnvelope` runs in LP (continuous level-follower) mode.

`LPGEnvelope` models a vactrol-style LPG. It maintains `vactrol_state_` with an asymmetric response: fast attack coefficient (0.6) and a variable decay that depends on `patch.decay` and `patch.lpg_colour`. The state drives three outputs:

- `gain()` — amplitude (directly equals `vactrol_state_`)
- `frequency()` — SVF cutoff, computed as `0.003 + 0.3 * vactrol_state_^4 + hf * 0.04`
- `hf_bleed()` — a dry/wet amount for a high-frequency bypass path

`LowPassGate` (in `dsp/fx/low_pass_gate.h`) combines these into a gain-scaled SVF low-pass plus an `hf_bleed` fraction of the pre-filter signal, producing the characteristic partial-filtering LPG timbre.

`ChannelPostProcessor` applies the LPG to both OUT and AUX identically. Engines that report `already_enveloped = true` bypass the LPG entirely. A negative `out_gain` in `PostProcessingSettings` routes the signal through `stmlib::Limiter` instead of a fixed multiplier — used by engines with inherently unpredictable amplitudes (swarm, noise, particle, string).

---

### TIMBRE and MORPH

TIMBRE and MORPH are unitless [0, 1] parameters with engine-specific meanings. They are never hardwired to a particular synthesis dimension globally. As examples:

- In `WaveshapingEngine`: TIMBRE = wavefold depth, MORPH = slope asymmetry.
- In `FMEngine`: TIMBRE = FM index, MORPH = feedback character.
- In `AdditiveEngine`: TIMBRE = spectral centroid, MORPH = spectral slope, HARMONICS = spectral bumps/formants.
- In `WavetableEngine`: TIMBRE, MORPH, HARMONICS are the three axes of a trilinear interpolation through an 8x8x3 wavetable cube.
- In `SpeechEngine`: TIMBRE = formant character, MORPH = phoneme/vowel position.

The modulation matrix allows each of TIMBRE, MORPH, and FREQUENCY to be modulated by CV. When a CV input is patched, its value replaces the internal default modulation source; when unpatched and a trigger is present, the `DecayEnvelope` is used instead.

---

### The AUX Output

Every engine writes two float buffers simultaneously: `out` and `aux`. The AUX output is not a simple copy or fixed offset. Typical AUX contents:

- A different oscillator configuration (VirtualAnalogEngine: synced pair vs. free-running pair)
- A sub-oscillator (FMEngine: sub-octave FM)
- A spectrally complementary signal (WavetableEngine: quantized/bit-crushed version of OUT)
- An alternate synthesis path (BassDrumEngine: synthetic model vs. analog model)
- The same signal with different chord voicing notes (ChordEngine: AUX contains notes routed there by `aux_note_mask`)

AUX shares the same `ChannelPostProcessor` and LPG path as OUT, with its own registered gain.

---

### Pitch and V/Oct Handling

Pitch arrives as a MIDI note number (float) through `EngineParameters::note`. The key conversion is `NoteToFrequency()` in `engine.h`:

```cpp
inline float NoteToFrequency(float midi_note) {
  midi_note -= 9.0f;
  CONSTRAIN(midi_note, -128.0f, 127.0f);
  return a0 * 0.25f * stmlib::SemitonesToRatio(midi_note);
}
```

`a0` is defined in `dsp/dsp.h` as `(440.0f / 8.0f) / kCorrectedSampleRate`, where `kCorrectedSampleRate = 47872.34 Hz` — the actual hardware I2S clock rate from a 72 MHz system clock divided by 47. This correction of ~0.26% prevents the module from being slightly flat. Frequencies are always expressed as normalized phase increments per sample (0 to 0.5), not in Hz, so no further sample-rate scaling is needed in the oscillators.

The note value passed to engines includes both the V/oct CV and the PITCH knob, averaged over two consecutive blocks to reduce zipper noise from step changes in sequencer pitch.

---

### Key DSP Techniques

- **PolyBLEP anti-aliasing**: `VariableShapeOscillator` uses `stmlib::ThisBlepSample` and `NextBlepSample` at all waveform discontinuities (saw reset, square edge, triangle kink), avoiding the need for oversampling in the VA path.
- **ParameterInterpolator**: All slowly-varying parameters (frequency, waveshape, gain) are linearly interpolated sample-by-sample within a block via `stmlib::ParameterInterpolator`, eliminating block-rate zipper noise with no extra state.
- **Shared RAM via arena allocator**: `stmlib::BufferAllocator` is freed and re-used per engine during `Init`, so Karplus-Strong delay lines, grain buffers, and FM accumulators all fit in the same RAM pool.
- **4x oversampling + FIR downsampling**: The 2-op FM engine oversamples by 4x and decimates with a short FIR (`4x_downsampler.h`) to suppress intermodulation aliases that would fold below Nyquist.
- **Wavetable differentiation**: `WavetableEngine` stores tables in integrated (cumulative sum) form to reduce storage; it differentiates on read using Hermite interpolation of adjacent integrated samples, which also provides anti-aliasing proportional to pitch.
- **Vactrol simulation**: `LPGEnvelope` uses a squared state variable for the cutoff frequency (`vactrol_state_^4`) and a nonlinear tail for the decay, producing the log-like response that characterizes Buchla-style vactrols without any lookup table.
