# Grids — Topographic Drum Sequencer

Grids is a Eurorack drum sequencer that generates trigger patterns for three instruments (kick, snare, hi-hat) by navigating a two-dimensional map of rhythmic archetypes. Rather than programming a pattern step-by-step, the performer dials in an X/Y coordinate that blends between up to four pre-authored rhythm "nodes" simultaneously. Density knobs per channel gate how many of those blended hits actually fire, while a randomness/chaos control perturbs the thresholds at the beginning of each pattern cycle. A second mode replaces the map with independent Euclidean rhythm generators per channel.

The module runs on an ATmega microcontroller (AVR, 8-bit) and is written entirely in C++ using Emilie Gillet's `avrlib` hardware-abstraction layer.

---

## Code Architecture

### File Structure

| File | Role |
|---|---|
| `grids.cc` | `main()`, ISR, pot scanning, LED/shift-register output |
| `pattern_generator.h` / `.cc` | All sequencing logic: map interpolation, Euclidean, state output |
| `clock.h` / `.cc` | Phase-accumulator internal clock with swing and tap-tempo locking |
| `hardware_config.h` | Peripheral typedefs (ADC channels, LED/input bit masks, SPI, MIDI) |
| `resources.h` / `.cc` | `PROGMEM` tables: 25 drum-map nodes, Euclidean patterns, BPM phase increments |

---

### The Topographic Map Data Structure

The rhythmic vocabulary lives in 25 hand-authored "nodes" (`node_0` through `node_24` in `resources.cc`), each a 96-byte `PROGMEM` array. The 96 bytes are organized as three blocks of 32 bytes — one per instrument (BD, SD, HH) — where each byte is a **probability/accent level** (0–255) for one of the 32 steps in a pattern cycle.

The 25 nodes are arranged in a logical 5x5 grid inside `drum_map[5][5]` in `pattern_generator.cc`:

```cpp
static const prog_uint8_t* drum_map[5][5] = {
  { node_10, node_8,  node_0,  node_9,  node_11 },
  { node_15, node_7,  node_13, node_12, node_6  },
  { node_18, node_14, node_4,  node_5,  node_3  },
  { node_23, node_16, node_21, node_1,  node_2  },
  { node_24, node_19, node_17, node_20, node_22 },
};
```

This 5x5 grid spans the full 0–255 range of both X and Y coordinates, dividing each axis into four equal segments of 64 values.

---

### X/Y Coordinate Selection and Bilinear Interpolation

`ReadDrumMap(step, instrument, x, y)` is the heart of the drum mode. Given an 8-bit X and Y coordinate plus a step index (0–31) and instrument index (0–2), it:

1. Selects the four surrounding nodes by right-shifting X and Y by 6 bits: `i = x >> 6`, `j = y >> 6`. This gives integer grid indices 0–3, leaving the lower 6 bits as sub-cell fractional position.
2. Reads the step-level byte from each of the four surrounding nodes (a, b, c, d at corners `[i][j]`, `[i+1][j]`, `[i][j+1]`, `[i+1][j+1]`).
3. Performs bilinear interpolation using `U8Mix` (a fixed-point linear blend): first blends a/b and c/d along X (`x << 2` rescales the lower 6 bits to 8-bit range), then blends those two results along Y.

The effect is a smooth, continuous morph across the entire map as X/Y are swept. Every fractional coordinate produces a unique blended step-level for each of the 96 positions (32 steps × 3 instruments).

---

### Density and Chaos Parameters

After interpolation, `EvaluateDrums()` compares the returned level against a per-channel threshold:

```cpp
uint8_t threshold = ~settings_.density[i];
if (level > threshold) { state_ |= instrument_mask; }
```

`density[i]` is the inverted ADC reading from each channel's density pot/CV. A high density value lowers the threshold, allowing more steps to fire. A level above 192 additionally sets an accent bit.

The **randomness** parameter (`settings_.options.drums.randomness`) is applied once per pattern cycle at `step_ == 0`. It scales a fresh random byte into `part_perturbation_[i]` (one per channel) and adds it to every step's level during that cycle. This uniformly raises or lowers apparent density in a non-deterministic way, creating fills and variations without changing the underlying map position. When swing is active, randomness is forced to zero (the randomness knob is repurposed to control swing depth).

---

### Clock Input Handling and Beat Tracking

`HandleClockResetInputs()` in `grids.cc` runs inside the Timer2 ISR at 8 kHz. It handles two clock sources:

- **External clock**: rising edges on `INPUT_CLOCK` are counted and converted to ticks based on `ticks_granularity[]`, which maps `ClockResolution` (4, 8, or 24 PPQN) to tick counts of 6, 3, or 1. MIDI clock byte `0xF8` also contributes ticks.
- **Internal clock** (`Clock`): a 32-bit phase accumulator (`phase_`) that increments by `phase_increment_` each ISR tick. `phase_increment_` is looked up from `lut_res_tempo_phase_increment[]` indexed by BPM. A rising edge is detected when `phase_ < phase_increment_` (i.e., the accumulator just wrapped). Swing is implemented by varying the wrap point: `Clock::Wrap(int8_t amount)` reads the high byte of `phase_` and resets it to zero when it exceeds `128 + amount`, shifting alternate beats slightly earlier or later.

`PatternGenerator::TickClock(num_pulses)` calls `Evaluate()` immediately on every tick, then advances `pulse_` by the tick count. When `pulse_` reaches `kPulsesPerStep` (3, i.e. 24 PPQN internally), it increments `step_` and wraps at `kStepsPerPattern` (32). The `euclidean_step_[]` counters are incremented only on even steps (sixteenth notes). Beat and first-beat flags are derived from `step_ & 0x7 == 0` and `step_ == 0` respectively.

Pulse duration is managed by `IncrementPulseCounter()`, also called from the ISR. In trigger mode (gate_mode = false), `state_` is zeroed after `kPulseDuration` (8) ticks (~1 ms), producing a short trigger pulse. In gate mode, `state_` is cleared on the clock's falling edge via `ClockFallingEdge()`.

---

### Euclidean Rhythm Mode

When `options_.output_mode == OUTPUT_MODE_EUCLIDEAN`, `EvaluateEuclidean()` runs instead of `EvaluateDrums()`. It skips odd steps (fires only on sixteenth notes), then for each of the three channels:

1. Derives pattern `length` (1–32) from `euclidean_length[i] >> 3` and `density` (0–31) from `settings_.density[i] >> 3`.
2. Computes a lookup address: `(length - 1) * 32 + density` into `lut_res_euclidean[]`.
3. Reads a precomputed 32-bit integer from `PROGMEM` and tests the bit at position `euclidean_step_[i]` to decide whether to fire.
4. Wraps `euclidean_step_[i]` modulo `length`.

`lut_res_euclidean` is a 1024-entry table of 32-bit integers (512 `uint32_t`s × 2 entries per row in source), encoding every Euclidean rhythm for lengths 1–32 and densities 0–31. Each integer is a bitmask where set bits indicate active steps.

Reset outputs (bits 3–5 of `state_`) indicate when each channel's Euclidean counter wraps to zero, useful for synchronizing downstream sequencers.

---

### Trigger Output and State Byte

The output byte `state_` is written to a shift register (`ShiftRegister`) via SPI, driving eight digital output lines. The bit layout (from `pattern_generator.h`) is:

```
DRUMS mode (output_clock=false):  RND  CLK  HHAC  SDAC  BDAC  HH  SD  BD
DRUMS mode (output_clock=true):   RND  CLK   CLK   BAR   ACC  HH  SD  BD
EUCLIDEAN (output_clock=false):   RND  CLK  RST3  RST2  RST1 EU3 EU2 EU1
EUCLIDEAN (output_clock=true):    RND  CLK   CLK  STEP   RST EU3 EU2 EU1
```

Bits 0–2 are the three trigger outputs. Bits 3–5 are accent/reset outputs whose meaning depends on mode. Bit 6 is always a clock output. Bit 7 is a random bit from `Random::state()`, updated each step, providing a free-running S&H random gate.

---

### Settings Persistence and Hidden Parameters

`Options::pack()` / `unpack()` encode all non-CV settings (clock resolution, output mode, output clock enable, tap-tempo mode, gate mode, swing) into a single byte stored at EEPROM address 0. A second EEPROM byte at address 1 tracks a `factory_testing_` counter; for the first five power-on cycles the RESET input retriggered outputs immediately (a factory test behavior), after which it becomes transparent.

Hidden parameters are accessed by holding the tap button for 500 ms (detected via `long_press_detected`), which freezes the pot positions. Moving any pot then maps it to a secondary function: BD density selects clock resolution (4/8/24 PPQN), SD density toggles tap-tempo, HH density toggles swing, X selects output mode (drums vs. Euclidean), Y toggles gate vs. trigger mode, and randomness toggles clock output.

---

### Interesting Implementation Details

- **All static members**: `PatternGenerator` and `Clock` use only `static` member variables, making them effectively singletons without heap allocation — appropriate for a microcontroller with 2KB of RAM.
- **PROGMEM reads**: All table accesses use `pgm_read_byte()` / `pgm_read_dword()` because the AVR Harvard architecture requires explicit flash reads; the node and lookup tables would otherwise consume nearly all available RAM.
- **`U8Mix` bilinear interpolation**: The interpolation uses only 8-bit multiply-shift operations, avoiding any division — critical for real-time performance in an 8-bit ISR.
- **Swing as phase modulation**: Swing is implemented entirely in the `Clock` class by adjusting the phase accumulator's wrap threshold, rather than shifting step timestamps — a clean approach that works uniformly with both internal and external clocks.
- **Step interleaving**: The 96-byte node arrays store step data with an implicit stride: `offset = (instrument * kStepsPerPattern) + step`, so all 32 steps for one instrument are contiguous, enabling efficient sequential reads during pattern playback.
