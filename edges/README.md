# Edges — Quad Chiptune Gate/Oscillator

Edges is a four-channel Eurorack module that generates pitched square waves, PWM signals, and a variety of digital waveforms with 1 V/oct CV control. Three channels (`channel_1`–`channel_3`) produce hardware timer-driven square waves; a fourth (`channel_4`) is a fully software-rendered digital oscillator offering triangle, NES triangle, pitched noise, NES noise, and sine waveforms. Every channel accepts a gate signal that enables or mutes its output. The module runs on an AVR XMEGA microcontroller and is implemented in C++ using the `avrlibx` hardware-abstraction layer.

---

## Code Architecture

### File Structure

| File | Purpose |
|---|---|
| `edges.cc` | Entry point, ISR definitions, main loop |
| `hardware_config.h` | Type aliases for all peripheral hardware (timers, PWM, SPI, USART, GPIO) |
| `timer_oscillator.{h,cc}` | Square/PWM oscillator using hardware timer compare registers |
| `digital_oscillator.{h,cc}` | Software-rendered oscillator written to a 16-bit DAC via SPI |
| `audio_buffer.{h,cc}` | 128-entry `uint16_t` ring buffer bridging the digital oscillator render loop to the audio ISR |
| `adc_acquisition.{h,cc}` | Three-stage SPI state machine for reading the external 12-bit ADC |
| `settings.{h,cc}` | Per-channel calibration, pulse-width, quantization, and arpeggio state; persisted to EEPROM |
| `midi_handler.{h,cc}` | MIDI note/gate/pitch-bend logic with four polyphony modes |
| `note_stack.h` | Templated LIFO note stack with sorted pitch index, used by `MidiHandler` |
| `voice_allocator.{h,cc}` | LRU voice allocator for polyphonic MIDI modes |
| `ui.{h,cc}` | Button debounce, LED output, calibration workflow, arpeggio recording |
| `resources.{h,cc}` | All PROGMEM tables: timer counts, phase increments, bitcrusher increments, bandlimited waveforms |
| `storage.h` | EEPROM load/save helpers templated on arbitrary data structures |
| `midi.h` | Thin MIDI stream parser type definitions |

---

### The Four Channels

Channels 1–3 are instances of `TimerOscillator`. Channel 4 is a `DigitalOscillator`. All four share a common interface in `edges.cc`:

- `UpdatePitch(pitch, pw_or_shape)` — recalculates timer or phase-increment parameters.
- `Gate(bool)` — enables or silences the channel output.

The five hardware objects (`channel_0` through `channel_4`) are declared as global singletons. `channel_0` is a hidden sub-oscillator that tracks `channel_1` at one octave up (the "NES triangle expander" feature described in a comment): it calls `SubFollow<Channel0Timer>(channel_1)` every time channel 1 updates.

---

### Timer Oscillators (Channels 1–3)

`TimerOscillator` drives the XMEGA's dual PWM timers in `TIMER_MODE_DUAL_PWM_T` mode. The hardware pins (`Channel0`–`Channel3` PWM outputs, mapped via `hardware_config.h`) toggle at a frequency determined by the timer period register and the prescaler. This produces a square wave entirely in hardware with zero CPU overhead during audio generation.

**Pitch to timer period conversion** (`timer_oscillator.cc`):

1. Pitch is represented as a fixed-point `int16_t` in units of 1/128 semitone (so MIDI note 60 = `60 << 7 = 7680`).
2. `UpdateTimerParameters` subtracts a base pitch offset that depends on the active prescaler (CLK/8 for normal range, CLK/64 for LFO/sub-bass range). Automatic prescaler switching occurs when pitch crosses E0 (24 << 7) or exceeds note 104.
3. The pitch remainder within one octave is used to index into `lut_res_timer_count` (97 entries covering one octave at the prescaler's reference pitch), with linear interpolation between adjacent entries.
4. Right-shifting the looked-up count by the number of octaves above the table base produces the final `period_`.
5. The compare value (`value_`) for the PWM high time is computed as `U16U8MulShift8(period_, pw)`, where `pw` is one of the fixed duty-cycle constants `{ 128, 85, 64, 32, 13, 10 }` (corresponding to 50 %, 33 %, 25 %, 12.5 %, 5 %, CV-controlled) from the `PulseWidth` enum.

**CV-controlled pulse width** (`PULSE_WIDTH_CV_CONTROLLED`): the ADC value from the fourth CV input (or a dedicated channel in the octal-ADC variant) is downshifted to 8 bits, clamped to 6–250, and stored in `cv_pw_`. It replaces the fixed duty-cycle constant in the `value_` calculation.

---

### Digital Oscillator (Channel 4)

`DigitalOscillator` writes 16-bit samples into `audio_buffer` (a 128-sample ring buffer). `Render()` is called from the main loop; the audio ISR pops one sample per interrupt via `audio_buffer.ImmediateRead()` and streams it to a 12-bit SPI DAC (`AudioDACInterface`).

**Phase accumulator**: pitch is converted to a 24-bit phase increment (`uint24_t`) using `lut_res_oscillator_increments` (97 entries, same 1-octave span as the timer table), with linear interpolation and octave right-shifting. The accumulator wraps naturally at 24-bit overflow.

**Waveform dispatch** uses a PROGMEM function-pointer table `fn_table_[]` indexed by `OscillatorShape`:

| `OscillatorShape` | Render function | Description |
|---|---|---|
| `OSC_TRIANGLE` | `RenderBandlimitedTriangle` | Crossfades between adjacent bandlimited triangle wavetables (8 tables, `wav_res_bandlimited_triangle_0`–`_6`) keyed by MIDI note number to suppress aliasing |
| `OSC_NES_TRIANGLE` | `RenderBandlimitedTriangle` | Same algorithm using the NES-style 4-bit staircase wavetables (`wav_res_bandlimited_nes_triangle_*`) |
| `OSC_SINE` | `RenderSine` | Reads `wav_res_bandlimited_triangle_6` (the highest-harmonic-count triangle, effectively sine-like at top of range) with a bitcrusher effect: a secondary phase accumulator (`aux_phase_`) advances at a rate set by `lut_res_bitcrusher_increments[cv_pw_]`, and the sample is only updated on that secondary overflow, producing sample-rate reduction |
| `OSC_PITCHED_NOISE` | `RenderNoise` | Maximal-length LFSR (`rng_state = (rng_state >> 1) ^ (-(rng_state & 1) & 0xb400)`) clocked by the phase accumulator overflow; produces pitched noise whose spectral density tracks the oscillator frequency |
| `OSC_NES_NOISE_LONG` | `RenderNoiseNES` | 15-bit NES noise LFSR, tap at bit 1 |
| `OSC_NES_NOISE_SHORT` | `RenderNoiseNES` | Same LFSR, tap at bit 6 (short mode, higher pitched noise texture) |

Bandlimited table lookup uses a hand-written AVR assembly interpolator (`InterpolateSample` / `InterpolateSample16`) that performs bilinear interpolation between adjacent table samples in 9 instructions using the hardware multiplier.

---

### Pitch CV Acquisition

An external 12-bit SPI ADC is read non-blockingly from inside the audio ISR through `ADCAcquisition::Cycle()`. The acquisition is split into three pipeline stages (begin conversion → read high nibble → read low byte) so that each ISR invocation advances the state machine by one step, avoiding an 128-cycle SPI stall inside the interrupt. The result is delivered once per three ISR calls (~48 kHz / 3 ≈ 16 kHz per channel).

V/oct conversion lives in `Settings::dac_to_pitch()` (within `ChannelData`): the raw 12-bit ADC code is multiplied by a per-channel `scale` factor and added to an `offset`, both determined during two-point calibration (C2 and C4). The formula `scale = (24 * 128 * 4096) / (code_C4 - code_C2)` maps the code difference across two octaves to the fixed-point pitch unit (1/128 semitone). When the quantizer is enabled, the result is rounded to the nearest semitone (`pitch = (pitch + 64) & 0xff80`) with hysteresis (`kHysteresisThreshold = 32` ADC counts) to prevent jitter near semitone boundaries.

An optional FM (frequency modulation) input, available when the `OCTAL_ADC` compile flag is set and individual FM CV jacks are present, applies a second scaled ADC reading as a pitch addend.

---

### MIDI Control

`MidiHandler` extends `midi::MidiDevice` and supports four polyphony modes configured via `settings.midi_mode()`:

- `MIDI_MODE_MULTITIMBRAL` — each MIDI channel (offset from `base_channel`) maps to a hardware channel independently. Each has its own `NoteStack<10>`.
- `MIDI_MODE_POLYPHONIC` — a single MIDI channel drives all four hardware channels, with note-to-voice mapping managed by `VoiceAllocator` (LRU round-robin, `lru_[]` array).
- `MIDI_MODE_3_1` — channels 1–3 are polyphonic on MIDI ch. 1; channel 4 is monophonic on MIDI ch. 2.
- `MIDI_MODE_CHORDS` — all four hardware channels play the same most-recent note simultaneously.

When a MIDI note is active, `MidiHandler::shift_pitch()` replaces the CV-derived pitch with the MIDI note number (`note << 7`) offset by the difference from C4 (`60 << 7`). Pitch bend is scaled to `(value - 8192) >> 5` (about ±2 semitones at full throw) and added to the final pitch value.

---

### Timer and Interrupt Architecture

| Timer | Role |
|---|---|
| `Channel0Timer` (TCD1) | Hardware square wave for channel 0 (sub-oscillator) |
| `Channel1Timer` (TDD0) | Hardware square wave for channel 1 |
| `Channel2Timer` (TDC1) | Hardware square wave for channel 2 |
| `Channel3Timer` (TDC0) | Hardware square wave for channel 3 |
| `AudioRateTimer` (TCE0) | 48.048 kHz overflow ISR (`665` counts at CLK) |

The `TCE0_OVF_vect` ISR is the system heartbeat: it pops one sample from `audio_buffer` to the DAC, advances `ADCAcquisition` by one pipeline stage, and on completion of each ADC conversion updates the corresponding channel's pitch and gate. Gate scanning and MIDI byte ingestion happen in the `default` case (no ADC result ready).

An optional hard sync feature is implemented by `TCD0_OVF_vect`: when the sync switch is closed, `Channel2Timer::Restart()` is called on every `channel_1` timer overflow, phase-locking channel 2 to channel 1.

The main loop has two responsibilities: calling `channel_4.Render()` to keep `audio_buffer` filled, and calling `ui.Poll()` every 48 audio interrupts (~1 ms) to debounce switches, update LEDs, and advance the arpeggiator.
