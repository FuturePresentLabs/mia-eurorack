# Frames

Frames is a 4-channel keyframer and mixer for Eurorack. It stores up to 64 snapshots ("keyframes") of 4 channel levels and smoothly interpolates between them as the FRAME knob or CV input is swept across a 16-bit position space (0–65535). An alternate Poly LFO mode repurposes the hardware as a quad wavetable LFO with phase spread and cross-channel coupling.

---

## Code Architecture

### File Structure

| File | Role |
|---|---|
| `frames.cc` | `main()`, ISR handlers, top-level render loop |
| `keyframer.h/.cc` | Core data model: keyframe storage, interpolation, DAC conversion |
| `poly_lfo.h/.cc` | Alternate mode: quad wavetable LFO engine |
| `ui.h/.cc` | ADC polling, switch debouncing, event dispatch, LED driving |
| `resources.h/.cc` | Auto-generated lookup tables (easing curves, VCA linearization, LFO wavetables) |
| `drivers/` | Thin HAL wrappers for DAC, ADC, LEDs, switches, trigger output |

---

### Keyframe Data Structures

Each keyframe is a `Keyframe` struct:

```cpp
struct Keyframe {
  uint16_t timestamp;        // position in 0–65535 frame space
  uint16_t id;               // monotonically increasing, used to pick LED color
  uint16_t values[4];        // level for each of the 4 channels
};
```

Up to `kMaxNumKeyframe` (64) keyframes are held in a flat array `keyframes_[]` kept sorted by `timestamp`. Insertions use `std::lower_bound` to find the insertion point and shift elements right; deletions shift left. A `num_keyframes_` counter tracks the live count.

Per-channel interpolation behavior is stored separately in `ChannelSettings`:

```cpp
struct ChannelSettings {
  EasingCurve easing_curve;  // STEP, LINEAR, IN_QUARTIC, OUT_QUARTIC, SINE, BOUNCE
  uint8_t response;          // 0 = linear VCA response, 255 = exponential
};
```

---

### Persistent Storage

All state is persisted to flash via `stmlib::Storage<0x8020000, 4>`, a wear-leveling flash store starting at address `0x8020000` spanning 4 flash pages. On `Init()`, `ParsimoniousLoad()` reads the entire settings block (keyframe array + channel settings + counters + `extra_settings` + calibration offset) into RAM. Saves are triggered by a very long press of the ADD button. The `extra_settings` word also encodes two UI flags: bit 0 = poly LFO mode active, bit 1 = sequencer mode active.

---

### The FRAME Parameter and Position Mapping

The FRAME knob and CV input are both 16-bit ADC values read by the `Ui` class. In `main()`, the CV modulation is DC-offset-corrected and added to the knob position:

```cpp
int32_t frame = ui.frame();
int32_t frame_modulation = (ui.frame_modulation() - dc_offset_frame_modulation) << 1;
frame += frame_modulation;
```

The result is clamped to `[0, 65535]` before being passed to `Keyframer::Evaluate()`. The full 0–65535 range maps linearly to the span between the first and last keyframe timestamps; the keyframe timestamps themselves are also stored as `uint16_t` values in the same range.

---

### Interpolation Algorithm

`Keyframer::Evaluate(uint16_t timestamp)` is the core function:

1. `FindKeyframe(timestamp)` runs `std::lower_bound` on the sorted `keyframes_[]` array to find the index `position` of the first keyframe at or after `timestamp`.
2. If `timestamp` falls before the first keyframe or after the last, the nearest boundary keyframe's values are held flat.
3. Otherwise, the fractional position between the bracketing keyframes A and B is computed as a 16-bit `scale`:
   ```cpp
   uint32_t scale = timestamp - a.timestamp;
   scale <<= 16;
   scale /= (b.timestamp - a.timestamp);
   ```
4. For each channel, `Easing(from, to, scale, curve)` is called with that channel's `EasingCurve`.

The `Easing()` function applies a nonlinear shape to `scale` before the final linear interpolation:
- `EASING_CURVE_STEP`: hard step at the midpoint.
- `EASING_CURVE_LINEAR`: pass-through (no shaping).
- `EASING_CURVE_IN_QUARTIC`, `OUT_QUARTIC`, `SINE`, `BOUNCE`: table-lookup from 1025-entry precomputed tables (`lut_easing_in_quartic[]`, etc.) with linear interpolation between entries. The lookup is `scale >> 6` for the base index with `(scale << 10) & 0xffff` as the fractional part.

The RGB LED color is also interpolated linearly between the colors assigned to the two bracketing keyframes. Colors come from an 8-entry `palette_[]`, indexed by `keyframe.id & 7`, giving each keyframe a unique hue.

---

### DAC Output and Channel Response

After `Evaluate()`, each channel's interpolated `levels_[i]` value passes through `ConvertToDacCode()` before being sent to the SPI DAC:

```cpp
uint16_t Keyframer::ConvertToDacCode(uint16_t gain, uint8_t response) {
  int32_t exponential = 65535 - gain;          // direct to THAT2164 VCA
  // Linearize the 2164's exponential law via lut_vca_linear[]
  int32_t linear = ...;
  uint16_t balance = lut_response_balance[response];
  return (linear + ((exponential - linear) * balance >> 15)) >> 4;
}
```

The `response` field (0–255) blends between a linearized and a raw-exponential VCA control law, allowing each channel to have its own loudness taper. `lut_response_balance[]` maps the 8-bit response setting to the blend coefficient.

---

### Poly LFO Mode

When `poly_lfo_mode_` is set, `PolyLfo::Render(int32_t frequency)` replaces the keyframer in the render loop. The FRAME knob/CV now controls LFO rate. The four channel knobs control:

- **Shape** (`set_shape`): selects a waveform from an 18-waveform wavetable (`wt_lfo_waveforms`, 18 × 257 bytes). Crossfading between adjacent waveform slots is done with `stmlib::Crossfade`.
- **Shape spread** (`set_shape_spread`): offsets the wavetable index by a signed delta for each successive channel, so the four channels can play different waveform shapes simultaneously.
- **Spread** (`set_spread`): controls phase offset between channels. Positive spread stacks channels at equal phase intervals (`phase_difference = spread << 15`). Negative spread runs each channel at a progressively different frequency (frequency-spread mode).
- **Coupling** (`set_coupling`): adds a fraction of the previous (or next) channel's instantaneous value to the current channel's phase accumulator, creating FM-like cross-modulation between the oscillators.

Phase accumulation uses a 32-bit phase register per channel. `FrequencyToPhaseIncrement()` converts the 16-bit frequency parameter to a phase increment via `lut_increments[]` with octave-range shifting.

The RGB LED tracks the LFO rate by interpolating through a 17-entry `rainbow_[]` color table.

---

### UI: Keyframe Editing

The `Ui` class polls two switches (`SWITCH_ADD_FRAME`, `SWITCH_DELETE_FRAME`) and five ADC channels (4 level knobs + 1 FRAME knob) through a 32-entry `EventQueue`. ADC values are filtered with a one-pole IIR (`31/32 * old + 1/32 * new`) and only generate events when they cross a `kAdcThreshold` (64 LSB at 16-bit resolution) hysteresis band.

Switch press duration determines action:

| Duration | ADD button | DELETE button |
|---|---|---|
| Short (<800 ms) | Add keyframe at current FRAME position | Delete nearest keyframe |
| Long (800–3000 ms) | Enter easing curve editor | Enter response curve editor |
| Very long (>3000 ms) | Save to flash | Erase all keyframes |

`FindNearestKeyframe()` calls `Keyframer::FindNearestKeyframe(timestamp, 2048)`. Within a tolerance of 2048 units (~3% of full range), the UI considers the knob to be "on" a keyframe and shows it as active (the KEYFRAME LED brightness pulses relative to distance from the exact timestamp). While a keyframe is active, moving the 4 level knobs directly edits that keyframe's `values[]` in-place.

**Sequencer mode** is toggled by a secret gesture (pressing ADD 5 times while on a keyframe). In sequencer mode, the FRAME CV input becomes a trigger/gate detector: a rising edge above 2/3 of full scale advances to the next keyframe by jumping `frame` directly to that keyframe's `timestamp`.

**Poly LFO mode** is toggled by a different secret gesture (pressing DELETE 10 times with the FRAME knob fully counterclockwise). Both mode flags survive power cycles via the `extra_settings` word in flash.

---

### Trigger Output

The `TIM1_UP_IRQHandler` ISR fires the trigger output whenever `keyframer.nearest_keyframe()` changes and then the position subsequently changes — meaning a trigger fires each time the FRAME position crosses from one keyframe's territory to another's. The pulse is a fixed `kPulseDuration` (128 timer ticks) wide, implemented with a decrementing counter driving `TriggerOutput::High()/Low()`.
