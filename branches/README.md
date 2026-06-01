# Branches

Branches is a dual Bernoulli gate module for Eurorack synthesizers. Each channel receives a trigger or gate signal and routes it to one of two outputs (A or B) based on a probability set by a knob. At minimum probability, all triggers go to output A; at maximum, all go to output B; in between, each trigger is routed randomly. The two channels are independent by default but can be linked (channel 2's input normalled from channel 1's output). A long-press on each channel's button switches the channel into toggle mode, turning the module into a probabilistic flip-flop.

---

## Code Architecture

### File Structure

- **`branches.cc`** — The entire firmware: hardware initialization, main loop, gate detection, probability calculation, LED control, and EEPROM-backed configuration. ~305 lines of C++ targeting an AVR ATmega.
- **`bootloader/bootloader.cc`** — A standalone FSK audio bootloader. It decodes firmware data from the audio input (`in_1`, PortD4), writes pages to flash via SPM instructions, and passes control to the main firmware. Activated by holding `switch_1` on power-up.

### Hardware Setup

All I/O is mapped via the `avrlib::Gpio<Port, Pin>` template:

| Signal | Pin | Direction |
|---|---|---|
| `in_1` | PD4 | Input (pull-up) |
| `in_2` | PD7 | Input (pull-up) |
| `out_1_a`, `out_1_b` | PD3, PD0 | Output |
| `out_2_a`, `out_2_b` | PD6, PD5 | Output |
| `led_1_a`, `led_1_k` | PD1, PD2 | Output |
| `led_2_a`, `led_2_k` | PB1, PB0 | Output |
| `switch_1`, `switch_2` | PC3, PC2 | Input (pull-up) |

ADC channels 0 and 1 read the two probability knobs. The ADC is left-aligned so `ReadOut8()` returns an 8-bit value suitable for indexing into `linear_table`. Timer 1 (`TCCR1B = 5`, clk/1024 prescaler) runs freely and its counter `TCNT1` is used to measure button hold duration. There are no interrupts; the entire firmware runs as a single polling loop.

### Main Processing Loop

`main()` spins in a `while(1)` loop that does four things each iteration, in order:

1. **ADC readout** — If a conversion is ready, read the 8-bit result, look it up in `linear_table` to get a 16-bit probability threshold `p[channel]`, then kick off the next channel's conversion. The ADC channel index is inverted (`channel_index = 1 - adc_channel`) because the physical ADC pins are wired in reverse order relative to the logical channel numbering.

2. **Switch scanning** — For each button, detect press/release edges and measure hold time against `TCNT1`. A short press (≥64 timer ticks, released before `kLongPressTime`) toggles `toggle_mode[i]`. A long press (≥`kLongPressTime` = 6250 ticks, ~800 ms at clk/1024 with 8 MHz clock) toggles `latch_mode[i]`. Both settings are persisted to EEPROM byte 0 via `DisplayConfigurationAndSave()`, packed into the low nibble of a configuration byte (inverted on write).

3. **Gate/trigger processing** — Described in detail below.

4. **LED refresh** — Drive `led_N_a` and `led_N_k` pins to select LED color. Each bi-color LED is controlled by two pins (anode and cathode): cathode high = green, anode high = red, both low = off.

At the end of every loop iteration the RNG state is advanced.

### Gate and Trigger Detection

`Read(channel)` returns the logical gate state, inverting the raw GPIO value because inputs are active-low (pull-up resistors, gate signals pull to ground). `input_state[i]` holds the state from the previous iteration.

On a **rising edge** (`new_input_state && !input_state[i]`), the channel fires:
- A fresh 16-bit random number is extracted from `rng_state` (`random_words & 0xffff`).
- It is compared to the threshold `p[i]`.
- If `random >= threshold && threshold != 65535`, `outcome = true` (route to B); otherwise `outcome = false` (route to A).
- `GateOn(i, outcome)` asserts the chosen output high and the other output low.

On a **falling edge** (and not in latch mode), `GateOff(i)` pulls both outputs low, tracking the gate duration through to the output.

`led_gate_delay[i]` is loaded with `kLedGateDelay` (0x1ff = 511 iterations) whenever the input is high or latch mode is active. It counts down each loop iteration; when it reaches zero, `led_state[i]` is cleared and the LED turns off. This stretches the LED flash so brief triggers remain visible.

### Probability Calculation

The 8-bit ADC reading indexes into `linear_table[]`, a 256-entry `PROGMEM` table of `uint16_t` values stored in flash. The table maps ADC positions 0–254 to threshold values 0–65535 (with entry 0 and 1 both being 0, and entries 255–256 clamped to 65535). This linearizes the ADC's response so that the knob sweeps evenly from 0% to 100% probability.

The random number is a 32-bit value from `rng_state`. The two channels consume the low 16 bits and high 16 bits respectively (`random_words >>= 16` between channels), so both channels draw from the same word but different halves. This means channel routing decisions are correlated within a single loop iteration but not across iterations.

The RNG itself is a Galois LFSR:

```c
rng_state = (rng_state >> 1) ^ (-(rng_state & 1u) & 0xD0000001u);
```

This is a maximal-length 32-bit LFSR with taps at positions 28, 27, and 0 (polynomial 0xD0000001). It produces a full-period sequence of 2^32 − 1 values with no division or multiplication — ideal for AVR. A commented-out alternative using the LCG `rng_state * 1664525 + 1013904223` was considered but not used.

### Toggle Mode vs. Bernoulli Mode

In default **Bernoulli mode**, each rising edge independently rolls the random number and routes the trigger: `outcome = (random >= threshold)`.

In **toggle mode** (`toggle_mode[i] == true`), the outcome is XORed with `previous_state[i]`:

```c
outcome = outcome ^ previous_state[i];
```

`previous_state[i]` is then updated to the new outcome. This makes the channel a probabilistic flip-flop: the output alternates between A and B, but randomly sometimes stays on the same output instead of flipping. At knob center, the flip probability is 50% per trigger, making it a perfect clock divider by 2 on average. At extremes, the output sticks on one side.

### Latch Mode

When `latch_mode[i]` is true, `GateOff()` is not called on the falling edge of the input. The selected output remains high until the next trigger arrives and re-rolls. This lets Branches hold its routed output as a sustained gate rather than a pulse, useful for longer gate signals or CV-style switching.

### Chained Mode

There is no explicit "chained mode" logic in firmware. Chaining is a hardware normalling convention: channel 2's input jack is normalled to one of channel 1's outputs on the PCB, so an unpatched channel 2 input automatically receives whatever channel 1 decided on the previous trigger. The firmware is unaware of this — it simply processes two independent channels.

### EEPROM Configuration

Configuration is stored in EEPROM byte 0 as a bit field, bitwise-inverted on write (so an erased EEPROM byte of 0xFF reads back as `~0xFF = 0x00`, meaning all modes off by default). Bits: 0 = channel 1 toggle mode, 1 = channel 1 latch mode, 2 = channel 2 toggle mode, 3 = channel 2 latch mode.
