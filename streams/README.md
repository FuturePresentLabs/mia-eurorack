# Streams

Streams is a dual-channel dynamics processor for Eurorack synthesizers, running on an STM32F1 microcontroller. Each channel independently processes an audio signal using one of six selectable algorithms. The two outputs are a gain control signal (sent to an external VCA via DAC) and a frequency control signal (sent to an external VCF via PWM). Neither audio signal passes through the MCU — Streams is a control processor, not an audio path processor.

## Code Architecture

### File Structure

```
streams.cc              — main(), ISR wiring, top-level Process() loop
processor.h/.cc         — polymorphic dispatcher for all processing modes
envelope.h/.cc          — AD/AR envelope generator
follower.h/.cc          — multi-band RMS envelope follower
compressor.h/.cc        — VCA compressor/expander with sidechain
vactrol.h/.cc           — analog vactrol emulation (low-pass gate)
filter_controller.h     — plain CV-to-frequency controller
lorenz_generator.h/.cc  — Lorenz attractor chaotic modulator (easter egg)
cv_scaler.h/.cc         — ADC offset calibration and input scaling
audio_cv_meter.h        — signal type discriminator and peak meter
gain.h                  — shared gain constants and Schmitt trigger threshold
meta_parameters.h       — single-knob attack/decay and amount/offset helpers
svf.h/.cc               — state-variable filter used in follower
ui.h/.cc                — switch/pot event handling, LED painting, settings storage
drivers/                — STM32 peripheral drivers (ADC, DAC, PWM, LEDs, switches)
```

### Dual-Channel Architecture

`streams.cc` declares `Processor processor[2]`, one instance per channel. The ADC interrupt fires at audio rate and calls `Process(uint16_t* cv)`, which iterates over both channels:

```cpp
processor[i].Process(
    cv_scaler.audio_sample(i),
    cv_scaler.excite_sample(i),
    &gain,
    &frequency);
dac.Write(i, cv_scaler.ScaleGain(i, gain));
pwm.Write(i, 65535 - frequency);
```

Every processor algorithm receives the same two int16_t inputs — `audio` and `excite` — and writes two uint16_t outputs — `gain` and `frequency`. This uniform interface is enforced by the `DECLARE_PROCESSOR` macro in `processor.h`, which generates Init/Process/Configure method stubs for each algorithm class and stores function pointers in `callbacks_table_`. Switching modes calls `set_function()`, which swaps in the new callbacks and re-initializes the algorithm.

Two channels can be linked (`set_linked(true)`). In linked mode, the pots act as global parameters shared by both channels (`globals_[4]`), and a change to one channel's mode immediately mirrors to the other via `Ui::Link()`.

### The EXCITE Input

EXCITE is the primary trigger/modulation input. `CvScaler::excite_sample()` subtracts the stored ADC offset so that an unpatched input reads zero. Each algorithm decides independently how to use EXCITE:

- **Envelope**: EXCITE is a gate/trigger. A Schmitt trigger (`kSchmittTriggerThreshold = 32768 * 5 * 2 / 3 / 8`) detects rising edges to start the envelope and monitors gate level to modulate envelope dynamics.
- **Vactrol**: EXCITE drives the simulated photocell. Negative edges are slew-limited to emulate the slow turn-off of a real vactrol LED.
- **Follower**: EXCITE is the signal whose envelope is being tracked (not the audio input).
- **Compressor**: EXCITE acts as an external sidechain. If `sidechain_signal_detector_` stays below a threshold (meaning no signal is present), the compressor falls back to metering the audio input.
- **FilterController**: EXCITE directly modulates the frequency output via a scaled multiply.

### Envelope Mode (`Envelope`)

`Envelope` implements a multi-segment generator, adapted from Peaks. Segments are defined by `level_[]`, `time_[]`, and `shape_[]` arrays. The two standard configurations are AD (attack/decay, no sustain) and AR (attack/release with sustain held at peak while gate is high), selected by `alternate_`.

Phase advances through each segment using `lut_env_increments[time_[segment_] >> 8]`. One interesting detail: the increment is further modulated by the live excite signal:

```cpp
rate_modulation_ += (excite > kSchmittTriggerThreshold ? excite : 0 - rate_modulation_) >> 12;
increment += (increment >> 7) * (rate_modulation_ >> 7);
```

This makes a harder trigger velocity produce a faster envelope.

The gain output applies two passes of the transform `compressed = 32767 - ((32767 - value)^2 >> 15)`, a gentle square-law waveshaper, then blends toward the waveshaped value according to `gate_level_` (the smoothed amplitude of the gate signal while held). This causes a stronger gate to produce a punchier, more distorted envelope curve.

The `frequency` output is a linear mapping of the envelope value scaled by `frequency_amount_` and shifted by `frequency_offset_`, providing simultaneous VCF modulation. The amount/offset are computed from a single knob via `ComputeAmountOffset()` in `meta_parameters.h`.

### Vactrol / Low-Pass Gate Mode (`Vactrol`)

`Vactrol` models the nonlinear, asymmetric response of a resistive optocoupler (vactrol) driving both a VCA and a VCF. The simulation has two sub-modes selected by `alternate_`:

**Normal mode**: EXCITE is low-pass-filtered on its falling edge (instant attack, slow decay via `decay_coefficient_`) to emulate the slow LED turn-off. A two-pole IIR (`state_[0]` and `state_[1]`) tracks the excite signal with separate attack and decay coefficients that themselves depend on whether the photocell is in a "sensitized" state (`state_[2]`). A 60-second desensitization time constant and 1-second resensitization time constant model the light-history memory of real vactrols. The final amplitude passes through `wav_gompertz`, a Gompertz sigmoid lookup table, capturing the logarithmic response of an LDR. VCF cutoff uses a squared version of the same control signal for a more pronounced brightness-with-amplitude coupling.

**Plucked mode**: On a gate trigger EXCITE, `state_[0]` and `state_[1]` are set immediately to full scale. They then decay independently at different rates, giving separate envelopes to VCF and VCA. A cross-term `(state_[3] >> 15) * (state_[1] >> 15)` adds overshoot to the cutoff for a plucked-string articulation.

### Envelope Follower Mode (`Follower`)

`Follower` splits the EXCITE signal into three frequency bands using two cascaded `Svf` instances set at ~200 Hz and ~800 Hz. Per-band energy is computed as the square of the band-pass signal. A peak-hold detector rides ascending peaks and snaps to local maxima, then a leaky integrator smooths the held peak using separate `attack_coefficient_[i]` and `decay_coefficient_[i]` values — with the low-band using slower attack and the high band using slower decay (to reject noise).

The three smoothed band energies are summed to produce an overall `envelope`, which is sqrt-scaled via `lut_square_root` before being output as `gain`. A spectral centroid is computed from the per-band energy ratios and tracked as a frequency output, providing automatic VCF tracking that follows the tonal center of the excite signal.

In `alternate_` mode (`only_filter_`), the centroid is used for the gain output instead, and `frequency` is fixed at full open, turning the channel into a spectrum-weighted filter controller.

### Compressor Mode (`Compressor`)

The compressor implements classic RMS compression with optional external sidechain. Level detection uses a leaky peak integrator on the squared signal. The core gain computation in `Compress()` is log-domain:

1. `Log2(squared_level) >> 1` converts energy to a dB-proportional integer (using a shift-and-LUT implementation).
2. `position = level - threshold_` computes dB above threshold.
3. `attenuation = position - (position * ratio_ >> 8)` applies the compression ratio.
4. An optional soft knee blends the attenuation using `lut_soft_knee`.
5. `Exp2()` converts the resulting gain back to linear, again via shift-and-LUT.

The `ratio_` field is the reciprocal of the compression ratio in 8.8 fixed point; a ratio of 1.0 means no compression (full pass-through above threshold), while 0 means a brickwall limiter. When `amount > 32768`, makeup gain is added and the ratio is computed from the knee position. At extreme settings, `attack_coefficient_` is forced to -1, which means the `detector_` snaps instantly to any new peak (brickwall limiting).

The `gain_reduction_` value exposed through `Processor::gain_reduction()` drives the LED bar in compressor mode (a downward bar showing dB of compression).

### Filter Controller Mode (`FilterController`)

A degenerate mode that routes EXCITE directly to frequency control with no gain output (`*gain = 0`). EXCITE is scaled by `frequency_amount_` and added to `frequency_offset_`, both controlled from the two pots. Used to drive an external VCF directly from a CV or audio-rate signal without any dynamics processing.

### Metering and LED Display

`Ui::PaintMonitor()` selects from four monitor sources via `MonitorMode`: EXCITE input, audio input, VCA CV jack, or output. Each sample is passed through `AudioCvMeter::Process()`, which maintains:

- A peak follower with fast attack (~10 ms) and slow decay (~250 ms) via asymmetric first-order IIR coefficients.
- A zero-crossing interval estimator. When the average interval exceeds 400 samples the signal is classified as CV (`cv_ = true`); below 200 samples it switches back to audio.

In audio mode, `leds_.PaintPositiveBar()` displays a log-scaled bar using the `wav_db` table. In CV mode, `leds_.PaintCv()` shows a bipolar bar. In compressor mode, `PaintNegativeBar()` displays gain reduction directly from `gain_reduction_`.

### Signal Path Summary

Streams operates entirely in the control domain. The audio signal is read by the ADC only for metering and sidechain detection. The DAC controls an external VCA (gain path), and a PWM output controls an external VCF (frequency path). This design means the dynamic range and noise floor of the audio path depend entirely on the quality of the external VCA/VCF, not the STM32's ADC.
