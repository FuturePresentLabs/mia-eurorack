# Stages

Stages is a multi-stage envelope generator for Eurorack. Each of its six channels can operate as an independent segment (ramp, hold, or step) or combine with adjacent channels to form multi-stage envelopes, LFOs, and sequencers up to the full length of the chain. Up to six physical modules can be daisy-chained via serial links to form a single logical instrument with up to 36 segments.

---

## Code Architecture

### File Structure

| File | Role |
|---|---|
| `stages.cc` | Entry point, ISR handlers, top-level `Process` / `ProcessOuroboros` loops |
| `segment_generator.h/.cc` | Core DSP: all segment types, state machine, phase accumulation |
| `chain_state.h/.cc` | Multi-module communication: neighbor discovery, segment grouping, parameter routing |
| `cv_reader.h/.cc` | ADC polling, low-pass filtering, CV + slider summing per channel |
| `settings.h/.cc` | Flash-backed persistent state (`State`, `PersistentData`) |
| `ui.h/.cc` | LED rendering, button debounce, loop and type change requests |
| `io_buffer.h` | Double-buffer structure (`Block`) shared between ISR and process loop |
| `variable_shape_oscillator.h` | PolyBLEP triangle/saw/square oscillator used for audio-rate ALT segments |
| `oscillator.h` | Simpler oscillator for Ouroboros harmonic mode |
| `delay_line_16_bits.h` | 16-bit delay line used by the HOLD delay segment |
| `drivers/` | Hardware abstraction: DAC, ADC, gate inputs, serial link, LEDs, switches |

The sample rate is `kSampleRate = 31250 Hz`. Audio is processed in blocks whose size is `kBlockSize`, driven by the DAC DMA interrupt. The main loop repeatedly calls `io_buffer.Process()`, which dispatches each completed block to either `Process`, `ProcessOuroboros`, or the factory test handler.

---

### The Segment System

Each channel is controlled by one `SegmentGenerator` instance (six total, allocated in `stages.cc`). A `SegmentGenerator` owns an array of up to `kMaxNumSegments = 36` low-level `Segment` structs plus a parallel `Parameters` array. Configuration and parameters are separated: configuration describes the topology (segment type and loop flag), while parameters carry the live CV/pot values.

**`segment::Configuration`** holds two fields:
- `Type type` — one of `TYPE_RAMP`, `TYPE_STEP`, `TYPE_HOLD`, or `TYPE_ALT`
- `bool loop` — whether this segment is a loop boundary

**`segment::Parameters`** holds two normalized floats:
- `primary` — main control (time for RAMP, level for HOLD/STEP), range roughly [-1, 2]
- `secondary` — shape/portamento/time depending on segment type

The low-level **`Segment`** struct uses pointers instead of values, allowing the process loop to read updated parameters on every sample without a reconfigure:

```cpp
struct Segment {
  float* start;       // NULL = start from current value
  float* time;        // NULL = infinite duration
  float* curve;
  float* portamento;
  float* end;
  float* phase;       // NULL = use internal accumulator
  int8_t if_rising;   // segment index to jump to on gate rising edge
  int8_t if_falling;  // segment index to jump to on gate falling edge
  int8_t if_complete; // segment index to jump to when phase reaches 1
};
```

The three `if_*` fields implement the state machine transitions entirely in data. A value of `-1` means "do nothing".

---

### ProcessMultiSegment: The Core State Machine

`ProcessMultiSegment` is the workhorse called for any multi-segment envelope. It runs sample-by-sample inside a tight loop:

1. **Phase advance**: `phase += RateToFrequency(*segment.time)`. `RateToFrequency` does a table lookup (`lut_env_frequency`) mapping a normalized [0,1] rate to a per-sample frequency at `kSampleRate`.

2. **Value computation**: `value = Crossfade(start, *segment.end, WarpPhase(phase, *segment.curve))`. `WarpPhase` implements a variable-shape curve using a rational function `t' = (1 + a) * t / (1 + a * t)` where `a = 128 * (curve - 0.5)^2`. At `curve = 0.5` the output is linear; below 0.5 the curve bends exponential-down; above 0.5 it bends exponential-up. This covers everything from sharp attacks to long decays in a single formula.

3. **Portamento**: `ONE_POLE(lp, value, PortamentoRateToLPCoefficient(*segment.portamento))`. The output is always the low-pass filtered value, enabling slew across all segment types.

4. **Transition logic**: On each sample the code checks `gate_flags` for `GATE_FLAG_RISING`, `GATE_FLAG_FALLING`, or whether `phase >= 1.0`. The matching `if_*` field selects the next segment index. On a transition, `phase` resets to 0 and `start` is set to the destination's `start` pointer (or the current `value` if `start` is NULL, providing seamless cross-fades).

---

### Ramp Segment

A RAMP segment is a timed transition from a start level to an end level with a variable curve. In `Configure`:
- `s->start` is NULL (start from current value), except when the generator has only one segment.
- `s->time` points to `parameters_[i].primary` (the slider + CV).
- `s->curve` points to `parameters_[i].secondary` (the small pot).
- `s->end` is determined by context: if the next segment is not a RAMP, `end` points to that segment's `primary` level; if it is the last segment, `end` is `&zero_`.

When two RAMP segments are adjacent, the first ramps up to `&one_` and the second uses its own secondary parameter as a curve but targets its own secondary for level — forming a raw triangle or multi-slope envelope.

---

### Hold and Step Segments

**HOLD** segments (`TYPE_HOLD`) set `start = end = &parameters_[i].primary` — the output tracks the slider/CV directly. They use `phase = &one_` so `Crossfade` always returns `*end`. When placed in a length-1 self-loop they hold indefinitely (`time = NULL`); otherwise `time` uses `parameters_[i].secondary`.

**STEP** segments (`TYPE_STEP`) also set `start = end = &parameters_[i].primary`. They set `portamento = &parameters_[i].secondary` to allow per-step slew. If a STEP is in a length-1 self-loop, `phase = &zero_` (sample mode — output freezes); otherwise `phase = &one_` (track mode).

---

### Single-Segment Dispatch Table

When a channel has no neighbors claiming its gate input, `ConfigureSingleSegment` selects one of 16 specialized process functions from `process_fn_table_` using a 4-bit index: `type * 4 + has_trigger * 2 + loop`. This avoids the overhead of the general state machine for the common single-channel case:

| Type | No trigger, no loop | No trigger, loop | Trigger, no loop | Trigger, loop |
|---|---|---|---|---|
| RAMP | Zero | FreeRunningLFO | DecayEnvelope | TapLFO |
| STEP | Portamento | Portamento | SampleAndHold | SampleAndHold |
| HOLD | Delay | Delay | TimedPulseGenerator | GateGenerator |
| ALT | Zero | FreeRunningOscillator | DecayEnvelope | PLLOscillator |

The ALT type (`TYPE_ALT`) is only available for single-segment channels. It enables audio-rate oscillation via `VariableShapeOscillator`, a PolyBLEP oscillator ported from Plaits that continuously morphs triangle through saw to square as the secondary parameter sweeps from 0 to 1.

---

### Segment Type Selection (Green / Yellow / Red)

Each channel's type and loop bit are packed into a single byte in `State::segment_configuration[i]`:
- bits [1:0] — `segment::Type` (0=RAMP/green, 1=STEP/yellow, 2=HOLD/red, 3=ALT)
- bit 2 — loop flag

A short button press cycles the type through the three main types (0 → 1 → 2 → 0). A long press (800 ms) toggles the loop bit, creating a loop start/end point. A very long press (3000 ms) on a valid single-segment loop enables the ALT type (`REQUEST_SET_ALT_SEGMENT_TYPE`). The UI maps types to LED colors via `palette_[] = { GREEN, YELLOW, RED, OFF }`. A color-blind mode replaces hue with blink patterns.

---

### Looping

Loop state is managed at two levels. In `segment::Configuration`, the `loop` flag marks a segment as a loop boundary. In `Configure`, the code finds the first and last segments with `loop == true`; the `if_complete` field of the last looping segment is set to `loop_start` instead of `loop_start + 1`, and `if_falling` is wired to `loop_end + 1` (the release segment) when a sustain-release tail exists.

`ChainState` computes a `LoopStatus` per channel (`NONE`, `START`, `END`, `SELF`) for the UI. The LED at the loop-start channel pulses with a slow fade; the loop-end channel pulses at a phase offset; a self-looping single channel pulses symmetrically.

---

### Multi-Module Chaining

`ChainState` implements a four-phase update cycle (one phase per audio block) over two `SerialLink` UART connections running at 921600 baud.

At startup, modules exchange `DiscoveryPacket` frames to establish `index_` (position in chain) and `size_` (chain length). After discovery, the update cycle does:

1. **TransmitRight** — leftmost-through-penultimate module sends `LeftToRightPacket` containing the output phase, active segment, loop geometry, and a bitmask of which inputs are patched.
2. **ReceiveRight / HandleRequest** — receives `RightToLeftPacket` with `ChannelState` (type, loop, pot, CV/slider) from the right neighbor; also processes any change requests forwarded from the rightmost module.
3. **TransmitLeft** — sends local `ChannelState` leftward. Non-leftmost modules relay their neighbors' states.
4. **ReceiveLeft / Configure** — leftmost module assembles the full channel state for all modules and calls `Configure` to rebuild the segment topology.

`Configure` walks the global channel list from left to right. A channel with its gate input patched starts a new segment group and claims as many following unpatched channels as possible. Each claimed channel adds one segment to the first channel's `SegmentGenerator` and one `ParameterBinding`. Unpatched channels with no upstream gate use `ConfigureSlave` — they output a function of the master channel's phase and active segment index for visual feedback. Remote parameters are delivered via `BindRemoteParameters` using the `cv_slider` and `pot` values packed into `ChannelState` at 16-bit resolution.

---

### Sequencer Mode

When `Configure` sees three or more segments where the first is not a STEP and is not looping, and all remaining segments are STEP type, it calls `ConfigureSequencer` instead of the normal path. `ProcessSequencer` then runs: on each gate rising edge it advances the active step index according to the direction set by the small pot — forward, backward, ping-pong, alternating, random, random-without-repeat, or direct address. The secondary pot on each STEP segment controls per-step portamento. The secondary pot on the first (non-STEP) segment controls direction. If the first segment is `TYPE_RAMP`, the output is chromatic-quantized via `step_quantizer_`.

---

### Ouroboros Mode

If chaining is detected but produces an impossible configuration (index >= `kMaxChainSize` or size > 6), the module enters "Ouroboros" mode and switches to `ProcessOuroboros`. This turns all six channels into harmonic oscillators tuned to integer or simple-ratio multiples of a root frequency set by channel 0's slider. Each channel's pot selects the harmonic ratio, the slider sets amplitude, and the gate input triggers an amplitude envelope. The output of channel 0 is the sum of all harmonics; the other five channels output their individual waveform.
