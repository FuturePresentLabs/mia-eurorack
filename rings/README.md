# Rings — Resonator Module: Code Architecture

Rings is a Eurorack module by Mutable Instruments that applies physical modeling resonators to an
input signal or to its own internal excitation. It models the vibration of struck, bowed, or plucked
resonant structures — bells, strings, plates, and instrument bodies — using three distinct DSP
approaches selectable at runtime.

---

## Directory and File Structure

```
rings/
  rings.cc                  — Main entry point, ISR, audio callback (FillBuffer)
  cv_scaler.cc/.h           — ADC scaling, V/Oct calibration, normalization detection
  ui.cc/.h                  — LED/button UI, polyphony/model selection
  settings.cc/.h            — Persistent settings (calibration, easter egg flag)
  meter.h                   — Signal metering utility
  resources.cc/.h           — Lookup tables (stiffness, FM quantizer, sine, etc.)
  drivers/                  — Hardware abstraction (codec, ADC, triggers, LEDs)
  dsp/
    dsp.h                   — Global constants: kSampleRate=48000, kMaxBlockSize=24
    patch.h                 — Patch struct: structure, brightness, damping, position
    performance_state.h     — PerformanceState: strum flag, note, tonic, fm, chord
    part.h/.cc              — Part class: top-level voice orchestrator
    resonator.h/.cc         — Modal resonator (bank of SVF bandpass filters)
    string.h/.cc            — Waveguide string (Karplus-Strong + dispersion)
    fm_voice.h/.cc          — FM voice (carrier + modulator with envelope follower)
    plucker.h               — Noise burst generator for internal string excitation
    strummer.h              — Strum/onset detection logic
    onset_detector.h        — Three-band energy onset detector
    note_filter.h           — Adaptive lag + median filter for V/Oct pitch
    follower.h              — Three-band envelope/centroid follower (used by FMVoice)
    limiter.h               — Stereo soft-limiter with peak follower
    string_synth_part.cc/.h — Easter egg: polyphonic string synth (additive + FX)
    string_synth_voice.h    — Individual string synth voice (additive harmonics)
    string_synth_envelope.h — AR envelope for string synth
    string_synth_oscillator.h — Bandlimited oscillator for string synth
    fx/
      reverb.h              — Griesinger plate reverb (allpass diffuser loop)
      chorus.h              — Chorus effect (string synth)
      ensemble.h            — Ensemble chorus (string synth)
      fx_engine.h           — Low-level delay-line DSP engine backing all FX
```

---

## Audio Callback and Signal Flow

`FillBuffer` in `rings.cc` runs per codec DMA block (24 samples at 48 kHz, giving ~2 ms blocks).
The pipeline per block is:

1. **Noise gate** — a fast-attack/slow-decay level follower gates the audio input. Below
   `kNoiseGateThreshold` (0.00003) the gain ramps linearly to zero, suppressing hiss when
   nothing is plugged in.
2. **`Strummer::Process`** — decides whether a strum event occurs this block.
3. **`Part::Process`** (or `StringSynthPart::Process` in easter egg mode) — runs all voices and
   writes stereo output to `out[]` and `aux[]`.
4. **Codec output** — 16-bit clip, written to left/right codec channels.

The 32 KiB reverb buffer is placed in CCM RAM (`__attribute__ ((section (".ccmdata")))`) to keep
it off the main SRAM bus.

---

## The Part Class

`Part` (`dsp/part.h/.cc`) is the central voice manager. It owns up to `kMaxPolyphony` (4) voices
and dispatches to one of six resonator models defined by `ResonatorModel`:

```
RESONATOR_MODEL_MODAL                     — bank of resonant filters
RESONATOR_MODEL_SYMPATHETIC_STRING        — waveguide string + sympathetic strings
RESONATOR_MODEL_STRING                    — waveguide string with dispersion
RESONATOR_MODEL_FM_VOICE                  — FM synthesis (hidden/bonus mode)
RESONATOR_MODEL_SYMPATHETIC_STRING_QUANTIZED — sympathetic strings, chord-quantized
RESONATOR_MODEL_STRING_AND_REVERB         — waveguide string fed into plate reverb
```

`Part` holds:
- `Resonator resonator_[4]` — modal voices
- `String string_[8]` — waveguide voices (up to 8: 4 polyphony × 2 strings each)
- `FMVoice fm_voice_[4]`
- `Plucker plucker_[4]` — internal pluck exciter per voice
- `stmlib::Svf excitation_filter_[4]` — pre-resonator bandpass/lowpass
- `stmlib::DCBlocker dc_blocker_[4]`
- `NoteFilter note_filter_`
- `Reverb reverb_`, `Limiter limiter_`

`ConfigureResonators` runs lazily when `dirty_` is set (on polyphony or model changes). For modal
mode, resolution is split across voices: `resolution = 64 / polyphony - 4`. For string modes all
8 strings are initialized with LFOs at slightly different frequencies (0.5 Hz down to 0.171 Hz)
used to modulate sympathetic string positions.

### Polyphony and Voice Allocation

`Part::Process` implements round-robin voice stealing. On each strum, the previously active voice
is latched (its `note_` is frozen from `note_filter_.stable_note()`), then `active_voice_` is
advanced. For 3-voice polyphony a ping-pong pattern (`kPingPattern`) distributes voices unevenly
to produce a more musical rotation.

In polyphony=1, `out_buffer_` and `aux_buffer_` from the resonator are sent directly to the stereo
outputs as two independent pickup positions. In polyphony>1, odd-indexed voices go to `aux` and
even-indexed voices go to `out`, giving spatial separation of simultaneous voices.

---

## Modal Resonator (`Resonator`)

**File:** `dsp/resonator.h/.cc`

The modal resonator models a struck or bowed body as a sum of up to 64 resonant modes. Each mode
is a second-order SVF bandpass filter (`stmlib::Svf`). `ComputeFilters` sets frequencies and Q for
all active modes:

- **Partial frequencies** start at `frequency_` (the fundamental) and accumulate a `stiffness`
  increment computed from a lookup table (`lut_stiffness`) indexed by `structure_`. Stiffness < 0
  compresses partials (inharmonic bar/plate modes); stiffness > 0 spreads them (bell-like). The
  stretch factor decays each iteration (×0.93 or ×0.98) to keep high partials from running away.
- **Q** starts at `500 × lut_4_decades[damping_]` and decays per partial by `q_loss`, which is
  derived from `brightness_`. Higher brightness means less Q attenuation per partial, preserving
  high-frequency modes longer.
- Partials that would alias (frequency ≥ 0.49 Nyquist) are clamped and do not increment
  `num_modes`.

`Resonator::Process` runs `num_modes` bandpass filters on each sample. Modes are paired into odd
and even groups. A `CosineOscillator` running at `position_` computes per-sample amplitude weights
for each mode pair: this is the pickup position simulation. Position 0 or 1 corresponds to a node
of many harmonics (few harmonics are audible); position 0.5 gives maximum even-harmonic suppression
(like a bridge pickup). Output is split across `out` (odd modes) and `aux` (even modes).

---

## String / Waveguide Resonator (`String`)

**File:** `dsp/string.h/.cc`

Implements Karplus-Strong waveguide synthesis. The core is a delay line (`StringDelayLine`, 2048
floats) with a damping filter in the feedback loop. Per-sample processing:

1. Read from the delay line at a fractional delay (Hermite cubic interpolation) to get the string
   output.
2. Mix in the external input sample.
3. Apply `DampingFilter` — a 3-tap FIR: `y = damping * (h0*x_1 + h1*(x + x_2))` where `h0` and
   `h1` are derived from `brightness_`. Higher brightness weights the center tap more, making the
   filter more transparent. Lower brightness weights outer taps, increasing rolloff.
4. Optionally apply an SVF lowpass (`iir_damping_filter_`) for additional damping.
5. Write back to the delay line.

The `aux` output is a second read from the delay line at `comb_delay = delay × position`, providing
the second pickup position.

**Damping mapping** uses an exponential RT60 formula: `rt60 = 0.07 × 2^(damping×96) × sampleRate`.
The per-sample decay coefficient is derived from `delay/rt60`. When `damping >= 0.95`, the code
crossfades to an infinite-decay coefficient, giving the fully sustained mode.

**Dispersion** (enabled only for `RESONATOR_MODEL_STRING` and `RESONATOR_MODEL_STRING_AND_REVERB`)
adds a second delay line (`StiffnessDelayLine`, 1024 floats) used as an allpass filter. The allpass
gain and delay fraction are controlled by `structure_` — a `dispersion` value derived as a
piecewise function around `structure = 0.25`. Positive dispersion stretches partials upward (like a
stiff piano string); negative values implement AC-blocking and asymmetric bridge clipping for a
different inharmonic character.

---

## Sympathetic Strings

Sympathetic string mode allocates `num_strings = 2 × kMaxPolyphony / polyphony` `String` objects
per voice. The first string is driven by the main exciter signal; additional strings are driven by a
scaled copy of the first string's differential output (`out - aux`). Each sympathetic string has:
- Brightness rolled off quadratically: `b' = b^4`
- Damping clamped to the range [0.7, 0.97]
- Position modulated by a per-string `CosineOscillator` LFO

`ComputeSympatheticStringsNotes` maps the `structure` parameter through a list of just-intonation
intervals (unison, perfect 4th/5th, octaves, and compounds) using a nonlinear `Squash` function to
cluster intervals at the extremes and spread them in the middle. In the quantized variant,
`structure` selects from 11 chord tables (octave, 5th, sus4, minor, m7, etc.) stored in `chords[]`.

---

## FM Voice (`FMVoice`)

**File:** `dsp/fm_voice.h/.cc`

A two-operator FM synth with an internal envelope and an audio-driven follower. The modulator
frequency is quantized to musical ratios via `lut_fm_frequency_quantizer` indexed by `ratio_`
(mapped from `structure`).

The `Follower` class splits input audio into three bands and computes a band-weighted centroid
(spectral center of gravity). This centroid drives FM amount, creating FM index that tracks the
brightness of an external audio input. When no audio is present, the internal `amplitude_envelope_`
and `brightness_envelope_` (triggered by strum) decay at rates set by `damping_`.

`position_` (via `feedback_amount_`) controls modulator self-feedback, which adds odd harmonics and
drives the modulator into chaos at high values.

---

## Internal Exciter and Plucker

When no audio is patched to the IN jack (`performance_state.internal_exciter == true`):

- **Modal mode**: injects a scaled impulse directly into `resonator_input_[0]` at strum time,
  with amplitude proportional to `SemitonesToRatio(filter_cutoff^2 × 24)`.
- **String mode**: triggers `Plucker::Trigger`, which fills a noise burst lasting
  `1/frequency × position` samples, shaped by an SVF lowpass at `filter_cutoff × 8`. The `Plucker`
  also runs a comb filter over the noise burst (delay = `position × 1/frequency`) to pre-color
  the excitation with the string's fundamental period.

---

## Strum and Onset Detection

`Strummer` (`dsp/strummer.h`) decides when to retrigger a voice. It checks three conditions:

1. **External strum jack** — `PerformanceState.strum` set by CV scaler hardware.
2. **V/Oct note change** — if note moves more than 0.4 semitones and no strum jack is connected.
3. **Audio onset** — if audio input is connected but no strum jack, `OnsetDetector` fires.

`OnsetDetector` (`dsp/onset_detector.h`) splits the signal into three bands using cascaded `Svf`
filters, computes a per-band energy derivative (onset detection function), then runs a Z-score
outlier test (`ZScorer`) on the smoothed ODF. A refractory inhibit counter prevents double-triggers
within the configured inter-onset interval.

An inhibit counter in `Strummer` additionally prevents re-triggering the same voice more often
than the configured IOI (10 ms for V/Oct, 40 ms for onset detection).

---

## Pitch Processing (`NoteFilter`)

V/Oct pitch from the ADC is noisy and may glitch at transitions. `NoteFilter` applies:
1. A 4-sample median filter to reject outliers.
2. An adaptive lag with a fast attack (1 ms time constant) that snaps to new notes immediately.
3. A slower lag (10 ms) to extract a `stable_note_` for use at strum time.
4. A short delay line (4 samples at block rate) on `stable_note_` to ensure the latched pitch for
   a new voice does not bleed in from the previous voice's glide.

---

## STRUCTURE, BRIGHTNESS, DAMPING, POSITION at the DSP Level

| Parameter | Modal | String |
|-----------|-------|--------|
| **STRUCTURE** | Selects partial stretch via `lut_stiffness`: 0=compressed (plate), 0.5=harmonic, 1=stretched (bell) | Controls dispersion allpass gain and is mapped to sympathetic string intervals |
| **BRIGHTNESS** | Q-loss per partial: how quickly high harmonics decay relative to low ones | Controls FIR damping filter's `h0`/`h1` balance and the IIR cutoff; also sets excitation filter frequency |
| **DAMPING** | Overall Q via `lut_4_decades` exponential: short decay to infinite sustain | RT60 decay coefficient; crossfades to infinite at 0.95 |
| **POSITION** | CosineOscillator amplitude weights for mode pairs; simulates pickup/excitation point | Second delay-line tap position; affects timbre via comb cancellation |

---

## Reverb

The `Reverb` class (`dsp/fx/reverb.h`) implements the Griesinger plate topology from the Dattorro
1997 paper. It uses `FxEngine` — a compile-time delay-line DSP engine backed by a 32 KiB 16-bit
buffer in CCM RAM.

The topology: input is summed to mono, then diffused through 4 allpass stages (lengths 150, 214,
319, 527 samples at 48 kHz). The diffused signal feeds two crossed feedback loops, each containing
2 allpasses and one long delay (4.5 k and 6.3 k samples), LFO-modulated for chorus-like smearing.
A one-pole lowpass inside each loop (`lp_`) controls the high-frequency decay rate.

In `RESONATOR_MODEL_STRING_AND_REVERB`, the reverb is driven post-mix with:
- amount = 0.1 + damping × 0.5
- time = 0.35 + 0.63 × damping
- lp = 0.3 + brightness × 0.6

and POSITION crossfades the stereo image before the reverb, with the `aux` channel inverted on
output to produce a wide antiphase wet field.

---

## Output Limiter

`Limiter` (`dsp/limiter.h`) is a soft peak limiter applied after all voices are summed. It tracks
the peak of `|L|`, `|R|`, and `|R - L|` with a fast attack (0.05) and very slow decay (0.00002),
then divides both channels by the peak when it exceeds 1.0. Final output is passed through
`stmlib::SoftLimit` at 0.8× scale, which applies a gentle S-curve to avoid hard clipping.
Per-model pre-gain constants range from 0.7× (FM voice) to 1.4× (modal, string).

---

## Easter Egg: String Synth (`StringSynthPart`)

Activated by a long-press combination and stored in settings. Replaces `Part` with
`StringSynthPart`, which is an additive string synth: 12 voices, each generating 3 harmonics with
per-voice amplitude registration. Polyphonic chord selection uses the same chord tables as the
sympathetic string mode. FX options include formant filter, chorus, ensemble, and two reverb
variants. Not detailed further here as it is a separate instrument mode.
