# Yarns

Yarns is a MIDI-to-CV/gate interface for Eurorack synthesizers, authored by Emilie Gillet (Mutable Instruments). It runs on an STM32F103 Cortex-M3 microcontroller and converts MIDI note, controller, and transport messages into up to four analog CV outputs and four gate/trigger outputs. The module supports a wide range of voice layouts, a per-part arpeggiator and step sequencer, multiple tuning systems, portamento, and an optional built-in audio oscillator mode on any CV output.

---

## Code Architecture

### File Structure

| File(s) | Role |
|---|---|
| `yarns.cc` | Entry point: `main()`, interrupt handlers, global wiring |
| `multi.h/.cc` | Top-level container (`Multi`): layouts, clock, MIDI dispatch |
| `part.h/.cc` | Per-channel logic: note allocation, arpeggiator, sequencer |
| `voice.h/.cc` | Per-output CV/gate generation, portamento, LFO, oscillator |
| `midi_handler.h/.cc` | MIDI stream parsing and routing to `Multi` |
| `internal_clock.h` | BPM/swing clock generator running at 48 kHz |
| `layout_configurator.h/.cc` | Auto-detects the best layout from incoming MIDI notes |
| `just_intonation_processor.h/.cc` | Adaptive just-intonation tuning algorithm |
| `ui.h/.cc` | Encoder/button event loop, four-character LED display |
| `settings.h/.cc` | Flat parameter descriptors and menu definition |
| `storage_manager.h/.cc` | Flash read/write for presets and calibration data |
| `drivers/` | HAL wrappers for UART (MIDI), SPI DAC, gate GPIO, display, encoder |

---

### MIDI Reception and Parsing

MIDI bytes arrive over USART1 at 31,250 baud. The `MidiIO` driver exposes two inline accessors (`readable()` / `ImmediateRead()`) that check and read the USART data register directly.

Inside `SysTick_Handler()` (fires at 8 kHz), every pending byte is pushed into a 128-byte `RingBuffer` via `midi_handler.PushByte()`. In the main loop, `midi_handler.ProcessInput()` drains that buffer byte-by-byte through `stmlib_midi::MidiStreamParser<MidiHandler>`, a template-based state machine that assembles complete MIDI messages and calls static callback methods on `MidiHandler` (e.g., `NoteOn`, `PitchBend`, `Clock`, `SysExByte`).

MIDI output uses two separate ring buffers: a 128-byte `output_buffer_` for normal messages and a 32-byte `high_priority_output_buffer_` for real-time bytes such as MIDI clock (`0xf8`). The high-priority buffer is drained first in `SysTick_Handler()`, ensuring clock bytes are never delayed behind queued note data.

---

### The Multi and Part System

The global `Multi` object is the central coordinator. It holds:
- Up to four `Part` instances (`part_[kNumParts]`)
- Exactly four `Voice` instances (`voice_[kNumVoices]`)
- A `MultiSettings` struct (layout, clock parameters, custom pitch table)

The `Layout` enum (`LAYOUT_MONO`, `LAYOUT_DUAL_MONO`, `LAYOUT_QUAD_POLY`, `LAYOUT_DUAL_POLYCHAINED`, `LAYOUT_QUAD_TRIGGERS`, `LAYOUT_QUAD_VOLTAGES`, etc.) determines how the four voices are partitioned among parts. When the layout changes, `Multi::ChangeLayout()` calls `Part::AllocateVoices()` on each part, handing it a pointer into the `voice_[]` array along with a count. For example, `LAYOUT_QUAD_POLY` gives all four voices to a single part, while `LAYOUT_DUAL_MONO` assigns one voice each to two independent parts.

Incoming MIDI messages are dispatched to parts by `Multi::NoteOn()` (and sibling methods), which iterates `num_active_parts_` and calls `part_[i].accepts(channel, note, velocity)`. A part accepts a message if the MIDI channel matches (channel `0x10` means omni) and the note and velocity fall within the configured min/max ranges stored in `MidiSettings`.

---

### Voice Allocation

`Part::InternalNoteOn()` implements the full voice allocation policy selected by `VoicingSettings::allocation_mode`:

- **`VOICE_ALLOCATION_MODE_MONO`**: A `stmlib::NoteStack` (`mono_allocator_`) tracks all held notes. The highest-priority note (last, lowest, or highest, controlled by `allocation_priority`) is always the sounding pitch. On note-off, the stack re-evaluates priority and may slide back to a previously held note.
- **`VOICE_ALLOCATION_MODE_POLY`**: Uses `stmlib::VoiceAllocator` with LRU stealing. `poly_allocator_.NoteOn(note, VOICE_STEALING_MODE_LRU)` returns a voice index.
- **`VOICE_ALLOCATION_MODE_POLY_STEAL_MOST_RECENT`**: Same allocator, MRU stealing.
- **`VOICE_ALLOCATION_MODE_POLY_CYCLIC`**: Round-robins through `voice_[]` using `cyclic_allocation_note_counter_`.
- **`VOICE_ALLOCATION_MODE_POLY_RANDOM`**: Picks a voice index via `Random::GetWord()`.
- **`VOICE_ALLOCATION_MODE_POLY_VELOCITY`**: Maps velocity to voice index (`velocity * num_voices >> 7`), so quiet notes go to voice 0 and loud notes to voice 3.
- **`VOICE_ALLOCATION_MODE_POLY_SORTED` / `POLY_UNISON_1` / `POLY_UNISON_2`**: Uses `mono_allocator_` to sort held notes, then distributes them across voices via `Part::DispatchSortedNotes()`. Unison modes spread fewer notes across all voices.

Polychaining is handled transparently: when the `VoiceAllocator` returns an index beyond the local voice count, the note is forwarded out the MIDI TX port to the next module in the chain via `midi_handler.OnInternalNoteOn()`.

---

### CV/Gate Output: MIDI Note to Voltage

`Voice::NoteToDacCode()` converts a fixed-point note value (MIDI note × 128, with sub-semitone resolution) into a 16-bit DAC code using a 11-point calibration table (`calibrated_dac_code_[kNumOctaves]`). Each entry represents one octave boundary (C0 through C10). Linear interpolation between adjacent octave codes gives sub-octave precision:

```cpp
int32_t a = calibrated_dac_code_[octave];
int32_t b = calibrated_dac_code_[octave + 1];
return a + ((b - a) * note / kOctave);
```

This implements V/oct scaling calibrated per-voice. Default initialization sets `calibrated_dac_code_[i] = 54586 - 5133 * i`, placing 0 V at C3 and scaling 1 V/oct.

`Voice::Refresh()` (called every 8 kHz SysTick from `Multi::Refresh()`) computes the final pitch by summing: the portamento-interpolated base note, pitch-bend offset, transpose/fine-tuning, and vibrato LFO. The result is passed to `NoteToDacCode()` to produce `note_dac_code_`, which is written to the `cv[]` array in `SysTick_Handler()` and then latched to the DAC.

Gate output is deliberately written one SysTick after the CV update (the `gate_output.Write(gate)` call uses the values from the *previous* iteration), ensuring the CV has settled before the gate rises.

`Multi::GetCvGate()` maps the four logical `cv[]` and `gate[]` outputs according to the active layout. In `LAYOUT_MONO`, outputs are: CV1=pitch, CV2=velocity, CV3=aux CV (configurable: mod wheel, aftertouch, LFO, etc.), CV4=second aux CV; Gate1=gate, Gate2=trigger pulse, Gate3=clock, Gate4=reset/running. In `LAYOUT_QUAD_TRIGGERS`, all four CV outputs carry the shaped trigger envelope instead of pitch.

---

### Arpeggiator and Sequencer

Both are driven by `Part::Clock()`, which is called from `Multi::Refresh()` whenever a swing-delayed tick fires. A prescaler (`arp_seq_prescaler_`) divides the 24 PPQN internal tick by the `clock_division` setting (values 1–96, from the `clock_divisions[]` table).

**Sequencer**: `ClockSequencer()` steps through the `SequencerSettings::step[]` array (up to 64 `SequencerStep` entries). Each step stores a note byte (`0x00`–`0x7f`) or a flag (`SEQUENCER_STEP_REST = 0x80`, `SEQUENCER_STEP_TIE = 0x81`) and a velocity/slide byte. If a held key is present, the step note is transposed relative to the recorded root: `note += pressed_keys_.most_recent_note().note - root_note`. Slide (portamento between steps) is implemented by calling `InternalNoteOn` before `StopSequencerArpeggiatorNotes`, letting the new note overlap the old.

**Arpeggiator**: `ClockArpeggiator()` reads held keys from `pressed_keys_` and steps through them according to `arp_direction` (up, down, up/down, random, as-played, chord) and `arp_range` (number of octaves). Rhythmic gating comes from a 16-bit pattern word looked up from `lut_arpeggiator_patterns[]`. Alternatively, Euclidean rhythm patterns are read from `lut_euclidean[]` using the `euclidean_length`, `euclidean_fill`, and `euclidean_rotate` parameters.

---

### Clock and MIDI Sync

`InternalClock` is a 32-bit phase accumulator running at 48 kHz (via the `TIM1_UP_IRQHandler`). Its `Process()` method returns `true` on each MIDI-clock tick (24 PPQN). Swing is applied by alternating the half-cycle length across a 12-tick window: the first six ticks of each beat use `half_cycle + swing_amount_`, the next six use `half_cycle - swing_amount_`.

When `settings_.clock_tempo >= 40`, the internal clock is active. Below 40, Yarns slaves to external MIDI clock: `MidiHandler::Clock()` calls `Multi::Clock()` directly on each received `0xf8` byte.

Swing is also applied when receiving external clock: `Multi::Clock()` measures the interval between ticks (`midi_clock_tick_duration_`) and pre-delays each of the 12 swing slots by a proportional amount stored in `swing_predelay_[]`. `Multi::Refresh()` decrements these counters and fires `Part::Clock()` when a slot reaches zero.

Clock and reset CV outputs are generated by `clock_pulse_counter_` and `reset_pulse_counter_` counters maintained in `Multi`. A bar-position counter resets at `clock_bar_duration * 24` ticks, at which point a reset pulse is issued.

---

### Display and Menu System

The UI is built around a four-character 7-segment LED display (`Display`), a rotary encoder (`Encoder`), and three switches (REC, START/STOP, TAP TEMPO). All hardware is polled in `Ui::Poll()` at 1 kHz; events are posted to a 32-entry `stmlib::EventQueue`.

`Ui::DoEvents()` (called from the main loop) dispatches queued events through a mode table (`modes_[]`). Each `Mode` entry holds function pointers for increment, click, and display-refresh actions, plus a `next_mode` for state transitions. The active mode is stored in `mode_` (type `UiMode`).

Settings are described by a flat `Setting` array in `settings.cc`, each entry recording the parameter's name (four characters for the display), associated object (multi or a specific part), byte offset, and value range. The encoder scrolls through the menu (`setting_index_`), and clicking enters edit mode where the encoder directly increments the underlying byte.

Calibration mode (`UI_MODE_CALIBRATION_ADJUST_LEVEL`) overrides the `cv[]` array in `SysTick_Handler()` with a raw value from `voice.calibration_dac_code(note)` so the user can adjust each octave point while observing the output on a voltmeter.

---

### Pitch Bend, Modulation, and Auxiliary CV

`Voice` stores `mod_pitch_bend_` (14-bit, centered at 8192) and `mod_wheel_` (7-bit). In `Voice::Refresh()`, pitch bend contributes `(mod_pitch_bend_ - 8192) * pitch_bend_range_ >> 6` semitones (fixed-point) to the note before DAC conversion. Vibrato is applied as `lfo * mod_wheel_ * vibrato_range_ >> 15`.

The `mod_aux_[]` array (8 slots) accumulates multiple modulation sources: velocity (slot 0), mod wheel (slot 1), LFO raw (slot 7), pitch-bend voltage (slot 5), breath controller (slot 3), foot pedal (slot 4), aftertouch (slot 2), and MIDI-clock-synced LFO (slot 6). `VoicingSettings::aux_cv` and `aux_cv_2` select which slot is routed to CV3 and CV4 in compatible layouts.

The LFO (`lfo_phase_`) is a free-running 32-bit accumulator producing a triangle wave. When `modulation_rate >= 100`, it switches to a PLL mode (`TapLfo()`) that phase-locks to the MIDI clock, allowing tempo-synced modulation.

---

### Tuning Systems

`Part::Tune()` applies one of several tuning systems before passing a note to `Voice::NoteOn()`. Equal temperament is the passthrough default. Just intonation is handled by `JustIntonationProcessor::NoteOn()`, which searches a ±1 quartertone range for the candidate pitch with the lowest "badness" score computed against a 16-entry exponentially-decaying history of recent notes. Other tuning systems (Pythagorean, quarter-tone variants, 27 Indian ragas) apply offsets from lookup tables in `resources.cc`. A fully custom per-pitch-class correction table (12 × int8_t cents, stored in `MultiSettings::custom_pitch_table`) is also available.
