# Peaks — Source Code Architecture

Peaks is a dual function generator for Eurorack. Each of its two independent channels can operate as an ADSR/AD envelope, a free-running or tap-tempo LFO, an 808-style drum synthesizer (bass drum, snare, or hi-hat), an FM drum, a bouncing-ball modulation source, a mini step sequencer, a pulse shaper/randomizer, or the secret "number station" audio easter egg. The two channels share the same four knobs; a mode button cycles through functions and a twin/split button controls how the pots are assigned between channels.

---

## Code Architecture

### File Structure

```
peaks/
  peaks.cc                        — main(), ISR setup, top-level Process()
  processors.h / processors.cc    — Processors class + global processors[2] array
  gate_processor.h                — GateFlags bit definitions and ExtractGateFlags()
  io_buffer.h                     — double-buffered block I/O between ISR and main loop
  ui.h / ui.cc                    — button/pot scanning, mode selection, LED feedback
  calibration_data.h/.cc          — DAC offset calibration stored in flash
  modulations/
    multistage_envelope.h/.cc     — envelope generator (AD, ADSR, and many variants)
    lfo.h/.cc                     — LFO with five waveforms and tap-tempo sync
    bouncing_ball.h               — physics-based modulation source
    mini_sequencer.h              — 2- or 4-step CV sequencer
  drums/
    bass_drum.h/.cc               — 808-style kick synthesis
    snare_drum.h/.cc              — 808-style snare synthesis
    high_hat.h/.cc                — 808-style hi-hat synthesis
    fm_drum.h/.cc                 — sine FM drum with morphable presets
    svf.h                         — templated state-variable filter (LP/BP/HP)
    excitation.h                  — exponential-decay impulse generator
  pulse_processor/
    pulse_shaper.h/.cc            — trigger-to-gate with delay, duration, repetitions
    pulse_randomizer.h/.cc        — stochastic trigger repetition/filtering
  number_station/
    number_station.h/.cc          — hidden spoken-number audio generator
  resources.h/.cc                 — lookup tables (waveforms, envelopes, pitch, SVF coefficients)
  drivers/                        — STM32 peripheral wrappers (ADC, DAC, gate inputs, LEDs, switches)
```

### Dual-Channel Structure and Shared Code

Both channels are instances of the same `Processors` class, declared as the global array `Processors processors[2]` in `processors.cc`. Each `Processors` instance owns one copy of every processor type as a member variable (via the `DECLARE_PROCESSOR` macro), so no allocator is involved. At runtime only one is active per channel; switching functions simply reassigns the `callbacks_` struct of function pointers.

The two channels run through the same `Process()` callback in `peaks.cc`, iterating `for (size_t i = 0; i < kNumChannels; ++i)`. Gate flag state for channel 1 is multiplexed into the high nibble of channel 0's input word, giving `MiniSequencer` on channel 0 a way to observe channel 1's trigger (used for sequencer reset).

Channel-specific initialization occurs in `Processors::Init(uint8_t index)`: `fm_drum_.set_sd_range(index == 1)` shifts the FM drum on channel 2 into a snare-like frequency range, and `number_station_.set_voice(index == 1)` selects the second voice of the number station.

### The Processor / Mode System

`ProcessorFunction` is a plain enum (`PROCESSOR_FUNCTION_ENVELOPE` through `PROCESSOR_FUNCTION_NUMBER_STATION`). Each entry maps to a `ProcessorCallbacks` struct holding three member-function pointers — `init_fn`, `process_fn`, `configure_fn` — stored in the static `callbacks_table_[]` array registered via the `REGISTER_PROCESSOR` macro.

Calling `Processors::set_function()` stores the new enum value, points `callbacks_` at the right row of the table, calls `init_fn` to reset the newly active processor, then calls `Configure()` to push the current pot values into it. `Process()` and `Configure()` subsequently dispatch through those pointers with zero branching overhead in the audio path.

`PROCESSOR_FUNCTION_TAP_LFO` reuses the `Lfo` callbacks entry (same `.process_fn`/`.configure_fn`) but sets `lfo_.set_sync(true)` and skips re-initialisation so the phase predictor state is preserved.

One notable detail in `Configure()`: when the snare drum or hi-hat is active and both the tone and snappy parameters are driven to their maximum (>= 65000), the processor silently switches itself to `PROCESSOR_FUNCTION_HIGH_HAT`. This makes the hi-hat accessible as a hidden extreme of the snare's parameter space rather than needing its own UI slot.

### ADSR Envelope — `MultistageEnvelope`

`MultistageEnvelope` is a generic N-segment envelope engine (up to `kMaxNumSegments = 8` segments). Each segment is defined by a level array, a time array, and a shape array (`ENV_SHAPE_LINEAR`, `ENV_SHAPE_EXPONENTIAL`, or `ENV_SHAPE_QUARTIC`). A `sustain_point_` index marks the segment at which the envelope pauses while the gate is high.

Standard ADSR is configured by `set_adsr()`: 3 segments, `sustain_point_ = 2`, attack with a quartic curve, decay and release with exponential curves. In split-pot mode (`CONTROL_MODE_HALF`) the envelope degrades to `set_ad()` — a 2-segment, no-sustain, exponential AD.

Phase advances via a `phase_increment_` looked up from `lut_env_increments[time >> 8]`, a precomputed table mapping 8-bit time values to 32-bit per-sample increments at 48 kHz. The output sample is a linear interpolation `a + (b - a) * t` where `t` is read from a second table — `lookup_table_table[LUT_ENV_LINEAR + shape]` — that transforms the linear phase into the desired curve shape. Loop variants (`set_ad_loop`, `set_adr_loop`, `set_adar_loop`) set `loop_start_` and `loop_end_` to create cycling envelopes without any extra branching logic; when the segment counter reaches `loop_end_` it wraps to `loop_start_`.

### LFO — `Lfo`

Five waveforms are available, selected by `LfoShape`: sine, triangle, square, steps (quantized triangle), and noise. Each is computed by a dedicated method dispatched through the static `compute_sample_fn_table_[]` function-pointer array.

**Sine** applies wavefolder morphing: positive `parameter_` crossfades toward a folded version of the sine using `wav_fold_sine`; negative `parameter_` crossfades toward `wav_fold_power` applied to a quarter-phase-shifted triangle, creating a power-of-two waveshape.

**Triangle** implements variable skew via pre-computed `attack_factor_` and `decay_factor_` derived from `parameter_`. The rising and falling segments are scaled independently so the waveform can sweep from sawtooth to ramp.

**Square** uses a `parameter_`-adjustable threshold against the 32-bit phase accumulator for pulse-width control, with guards to prevent the pulse collapsing to zero or full width.

**Steps** quantizes the triangle wave to 2–17 discrete levels controlled by `parameter_`.

**Noise** holds a random sample per cycle and interpolates to the next using either linear or raised-cosine interpolation (or a blend) controlled by `parameter_`.

In free-running mode the phase increment is interpolated from `lut_lfo_increments`, covering slow LFO rates. In tap-tempo mode (`sync_ = true`), rising gate edges measure `sync_counter_` samples since the last trigger. Short intervals (< 1920 samples) update the period with a one-pole smoother to reject jitter; longer intervals query `stmlib::PatternPredictor<32,8>`, which detects common clock subdivisions/multiples.

### Drum Synthesis

All drum voices share two utility classes: `Excitation` (an exponentially decaying impulse with a configurable delay and `decay_` coefficient applied as `state = state * decay >> 12` each sample), and `Svf` (a templated state-variable filter supporting LP, BP, and HP modes with a `punch_` parameter that transiently raises the cutoff and damp in proportion to the low-pass output).

**Bass Drum (`BassDrum`)** models the 808 BD's bridged-T resonator. Three `Excitation` objects fire sequentially on each trigger: `pulse_up_` (immediate positive impulse), `pulse_down_` (1 ms delayed negative impulse), and `attack_fm_` (4 ms delayed positive impulse). The SVF `resonator_` runs in bandpass mode; its `frequency_` is boosted by a fixed 17 semitones while `attack_fm_` is active, replicating the 808's frequency sweep on transient. The `punch_` parameter feeds the SVF's self-modulation. Output is post-filtered by a first-order low-pass (`lp_coefficient_`) to tame high-frequency content.

**Snare Drum (`SnareDrum`)** layers two resonant body modes plus a noise burst. `body_1_` and `body_2_` are bandpass-filtered resonators tuned a perfect-fourth apart (52 and 64 semitones MIDI), each fed by its own `Excitation` pair. A third `Excitation` (`excitation_noise_`) gates broadband noise through a higher-frequency bandpass `noise_` SVF to produce the snare rattle. `gain_1_` and `gain_2_` control the relative weight of the two body modes (the `tone` parameter shifts amplitude between them), and `snappy_` scales the noise envelope amplitude.

**Hi-Hat (`HighHat`)** uses six square-wave oscillators at fixed co-prime phase increments (48318382, 71582788, 37044092, 54313440, 66214079, 93952409) to generate a metallic noise signal without a PRNG — each oscillator contributes its sign bit. The sum passes through a bandpass SVF at ~8 kHz for color, then through a half-wave rectifier (clamp to 0 below zero, 808-style VCA), then through a highpass coloration SVF at ~13 kHz. The SVF is clocked twice per sample for stability at the high cutoff frequency.

**FM Drum (`FmDrum`)** is a single sine oscillator with two independent exponential-decay envelopes applied to FM index (`fm_envelope_phase_`) and amplitude (`am_envelope_phase_`). A short fixed-rate auxiliary envelope (`aux_envelope_phase_`, increment 4473924) provides an additional pitch sweep on attack when the base frequency is below mid-range. Phase is initialized to a non-zero offset proportional to `fm_amount_` so the first cycle sounds like a click rather than a zero-crossing. A `noise_` parameter crossfades the sine with white noise; an `overdrive_` parameter applies waveshaping via `wav_overdrive`. The `Morph()` method performs bilinear interpolation across a 5×2 grid of parameter presets (`bd_map` or `sd_map` depending on channel index) to expose two-knob timbre morphing in split mode.

### Bouncing Ball — `BouncingBall`

A simple fixed-point physics simulation: on each sample, `velocity_ -= gravity_` and `position_ += velocity_`. Floor and ceiling collisions negate and attenuate velocity: `velocity = -(velocity >> 12) * bounce_loss_`. `gravity_` is looked up from `lut_gravity` via `Interpolate88` for a log-like feel. `bounce_loss_` maps the pot value through a squared inverse curve so the full range feels intuitive. On a rising gate the ball is reset to `initial_amplitude_` with `initial_velocity_`.

### Trigger Input Detection

`ExtractGateFlags()` in `gate_processor.h` converts a raw boolean gate level into a `GateFlags` byte with distinct bits for `GATE_FLAG_RISING`, `GATE_FLAG_HIGH`, and `GATE_FLAG_FALLING`. This runs in `TIM1_UP_IRQHandler` at 48 kHz, storing results into `gate_flags[2]`. Channel 0's byte also carries channel 1's flags in the high nibble (`GATE_FLAG_AUXILIARY_*`) — used by `MiniSequencer` to reset on channel 2's trigger — plus `GATE_FLAG_FROM_BUTTON` when the gate was sourced from a panel button press rather than an external signal.

The DAC interrupt also handles the button-as-gate feature: `ui.ReadPanelGateState()` returns bits for `panel_gate_control_[0]` and `panel_gate_control_[1]`, which are OR-ed with the hardware gate inputs so pressing the trigger buttons behaves identically to a jack signal.

### UI: Button and Pot Mapping

`Ui` owns the `EditMode` and `Function` state for both channels. Four `EditMode` values govern pot routing:

- `EDIT_MODE_TWIN` — both channels share the same function and all four pots feed both processors identically (`CONTROL_MODE_FULL`).
- `EDIT_MODE_SPLIT` — the left two pots control channel 1 and the right two control channel 2, each receiving `CONTROL_MODE_HALF`.
- `EDIT_MODE_FIRST` / `EDIT_MODE_SECOND` — one channel at a time is addressed with all four pots; the inactive channel freezes its parameter values. A "snap" mode (toggled by holding the twin button at power-on) requires each pot to pass through its saved value before becoming active after a channel switch, preventing parameter jumps.

The `SWITCH_FUNCTION` button short-press cycles through the four primary functions (`FUNCTION_ENVELOPE`, `FUNCTION_LFO`, `FUNCTION_TAP_LFO`, `FUNCTION_DRUM_GENERATOR`). A long press (> 600 ms) jumps to the alternate bank (`FUNCTION_MINI_SEQUENCER`, `FUNCTION_PULSE_SHAPER`, `FUNCTION_PULSE_RANDOMIZER`, `FUNCTION_FM_DRUM_GENERATOR`), cycling within that group on subsequent long presses.

The `SWITCH_TWIN_MODE` button short-press toggles between `EDIT_MODE_TWIN` and `EDIT_MODE_SPLIT`; a long press rotates through `EDIT_MODE_FIRST`/`EDIT_MODE_SECOND`, locking pots on each transition. Holding both buttons simultaneously for 600 ms, repeated three times, activates the `NumberStation` easter egg on both channels.

The `function_table_[FUNCTION_LAST][2]` static array maps each UI-level `Function` to per-channel `ProcessorFunction` values. `FUNCTION_DRUM_GENERATOR` maps channel 0 to `PROCESSOR_FUNCTION_BASS_DRUM` and channel 1 to `PROCESSOR_FUNCTION_SNARE_DRUM`, giving the typical kick+snare pairing when both channels share a function.

Settings (edit mode, functions, pot values, snap mode) are persisted to flash via `stmlib::Storage` using parsimony-save to minimize write cycles.

### DSP Techniques of Note

- **32-bit phase accumulators** throughout: all oscillators and envelope timers use wrapping uint32 counters, making rate computation trivial and avoiding conditionals.
- **Lookup table interpolation**: `Interpolate824` and `Interpolate1022` are fixed-point interpolations into 512-entry tables with sub-sample accuracy, used for sine, envelope, and SVF coefficient tables. The `lut_svf_cutoff` and `lut_svf_damp` tables precompute the SVF's `f` and `damp` coefficients from frequency/resonance to avoid transcendental functions in the audio loop.
- **Double-buffered block I/O**: `IOBuffer` maintains two 4-sample blocks. `TIM1_UP_IRQHandler` fills the I/O block one sample at a time; the main loop processes the render block via `Process()`. Block size of 4 keeps latency at ~83 µs while amortizing the per-block `PollPots()` ADC read overhead.
- **Pattern predictor for tap tempo**: `stmlib::PatternPredictor<32,8>` detects rhythmic patterns in inter-trigger intervals, allowing the LFO to lock to subdivisions of an incoming clock without explicit division controls.
- **SVF overclocking**: `HighHat` runs its filters twice per audio sample to maintain stability at cutoff frequencies that approach the Nyquist limit.
