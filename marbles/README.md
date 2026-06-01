# Marbles

Marbles is a Mutable Instruments Eurorack module that generates random clocks and random voltages with controllable randomness, looping, and quantization. The t section produces two gate outputs with probabilistic timing relationships. The X section produces three random CV outputs sampled from a beta distribution and optionally quantized to a musical scale. A shared Deja Vu control lets both sections loop back over a captured sequence of random values.

## Code Architecture

### Directory and File Structure

```
marbles/
  marbles.cc                    # Top-level: Init(), main loop, Process() block callback
  cv_reader.cc / .h             # ADC filtering, scaling, and CV+gate input handling
  cv_reader_channel.h           # Per-channel LP filter and attenuverter logic
  clock_self_patching_detector.h# Detects when a t gate output is patched to the X clock
  note_filter.h                 # LP filter for the external pitch CV input
  scale_recorder.h              # Records a user-played scale from an external CV source
  settings.cc / .h              # Persistent state (calibration, scale data, UI state)
  ui.cc / .h                    # Button/LED UI state machine
  io_buffer.h                   # Double-buffered sample block management
  ramp/
    ramp.h                      # kMaxRampValue constant
    ramp_extractor.cc / .h      # Converts external gate/clock into a continuous ramp
    ramp_divider.h              # Divides/multiplies a ramp by a rational ratio p/q
    ramp_generator.h            # Simple free-running sawtooth phase accumulator
    slave_ramp.h                # Ramp that tracks a master at a given ratio; drives gate outputs
  random/
    random_generator.h          # LCG fallback PRNG (used when hardware RNG buffer is empty)
    random_stream.h             # Unified random source: hardware RNG ring buffer + LCG fallback
    random_sequence.h           # Circular buffer of random values with Deja Vu looping
    distributions.h             # Beta distribution sampler via precomputed inverse-CDF tables
    t_generator.cc / .h         # Generates t1/t2 gate outputs and master/slave ramps
    x_y_generator.cc / .h       # Generates X1/X2/X3 and Y CV outputs
    output_channel.cc / .h      # Per-channel voltage generation, quantization, lag
    quantizer.cc / .h           # Variable-resolution pitch quantizer with weighted scale degrees
    lag_processor.cc / .h       # STEPS smoothing: linear ramp or one-pole LP between voltages
    discrete_distribution_quantizer.cc / .h  # Weighted discrete distribution sampler
  drivers/                      # STM32F4 hardware drivers (ADC, DAC, RNG, clocks, gates, LEDs)
```

The system runs at 32 kHz sample rate. The DAC interrupt calls `FillBuffer()` to read hardware clocks and queue a new sample block. The main loop calls `io_buffer.Process()` which dispatches to `Process()` once per block. All signal generation happens inside `Process()`.

### The t Section: Clock Generation

`TGenerator` (`random/t_generator.cc`) produces a master ramp and two slave ramps (T1, T2) along with two gate signals. It operates in one of seven behavioral modes selected by `TGeneratorModel`:

- `T_GENERATOR_MODEL_COMPLEMENTARY_BERNOULLI` (default): a single uniform random draw is compared to `bias_`; T1 fires when the draw is above the threshold, T2 fires when it is below. The two channels are exactly complementary.
- `T_GENERATOR_MODEL_INDEPENDENT_BERNOULLI`: separate draws for T1 and T2, giving independent probabilities.
- `T_GENERATOR_MODEL_THREE_STATES`: three outcomes per clock — T1 only, T2 only, or neither — with the bias controlling the mixture.
- `T_GENERATOR_MODEL_DRUMS`: a fixed 8-step drum machine pattern that is randomly selected each bar. The bias selects from 18 built-in patterns ranging from sparse to dense.
- `T_GENERATOR_MODEL_MARKOV`: a 16-step history buffer drives a logistic-regression probability for each channel. Four contextual factors are weighted: periodicity (repeat what happened 8 steps ago), simultaneous suppression (avoid both channels at once), density suppression (avoid consecutive hits), and echo (one channel reflects the other at a 4-step offset). The bias controls the weight of all four factors.
- `T_GENERATOR_MODEL_CLUSTERS` / `T_GENERATOR_MODEL_DIVIDER`: assign integer ratios from precomputed `DividerPattern` tables to the two slave ramps instead of per-step Bernoulli draws. In Clusters mode the pattern is randomly chosen from 17 options; in Divider mode a `HysteresisQuantizer` selects from 17 symmetric ratio pairs centered on 1:1.

**Jitter.** On each master-ramp wrap-around, `TGenerator` draws a jitter sample from a `FastBetaDistributionSample` (a pre-baked beta(3,3) ICDF table) and converts it to a frequency multiplier via `SemitonesToRatio`. Crucially, the multiplier is further scaled by `phase_difference_`, the accumulated drift of the jittery clock relative to the straight master clock. This ensures the jittery clock stays statistically centered on the master even under heavy jitter.

**Pulse shaping.** Each slave ramp is driven by `SlaveRamp`, which holds its own phase accumulator and a target pulse width. In Bernoulli mode (`bernoulli_ = true`) the ramp rises toward 1.0 with a slope that adapts based on whether this step is expected to fire — if not, it rises slowly; if yes, it rises quickly to complete the cycle. In ratio mode it simply scales the master frequency by the rational ratio. The gate output is high while `phase < pulse_width`.

**External clock sync.** When a cable is present at the T clock input, `RampExtractor` takes over. It maintains a 16-pulse history ring buffer and runs 13 concurrent predictors: a slow moving average (α=0.1), a fast moving average (α=0.5), a trigram hash table that maps (bucket(t-2), bucket(t-1)) to predicted period, and 10 periodicity detectors for repeat intervals 1–10. Each predictor is scored with `accuracy = 1 / (1 + 100 * error²)`, which is slowly incremented when accurate and quickly decremented when wrong (`SLOPE` macro). The best predictor at each pulse determines the phase increment for the next interval. The ratio control adjusts the external clock by a `HysteresisQuantizer` that snaps to 9 rational ratios between 1:4 and 4:1.

### The X Section: Random Voltage Generation

`XYGenerator` (`random/x_y_generator.cc`) manages four `OutputChannel` instances: X1, X2, X3, and Y. Each channel has its own `RandomSequence` and `OutputChannel`.

**Clock routing.** The XY section clock source is selected each block:
- If nothing is patched to the X clock input, each of X1/X2/X3 follows one of the three t ramps (T1, master, T2 respectively), so the three channels change at different rates set by the t divider patterns.
- If a cable is patched, `ClockSelfPatchingDetector` checks whether the signal matches any of the three t gate outputs by counting synchronous transitions. If a match score of 8+ is found, the XY section treats the corresponding t ramp as internal (effectively locking the two sections together without going through the `RampExtractor` again). Otherwise, the external clock goes through its own `RampExtractor` instance, and all three X channels share that ramp.

**Voltage generation.** `OutputChannel::GenerateNewVoltage()` draws a value from `RandomSequence::NextValue()`, then passes it through `BetaDistributionSample(u, spread, bias)`. The beta distribution is sampled via a 2D table of precomputed ICDFs indexed by bias (5 levels) and spread (9 levels), with bilinear interpolation. The tails get higher-resolution sub-tables. As spread approaches 0, a degenerate blend collapses the distribution toward a delta at `bias`. As spread approaches 1, a Bernoulli blend produces only the extreme values.

**STEPS and lag.** The `steps_` parameter controls quantization and smoothing. Above 0.5 the output is hard-quantized to the current scale. Below 0.5 the output is sent through `LagProcessor`, which performs a linear ramp from the previous voltage to the new one over one clock period, with an additional one-pole lowpass whose cutoff is controlled by `1 - 2 * steps`. This gives a smooth glide at steps = 0.

**Register mode (shift register).** When `register_mode` is enabled, the X spread CV input becomes a pitch input rather than a spread control. `NextValue()` is called with `deterministic = true` and the current `register_value` as its argument. The loop buffer stores `1.0f + value` to distinguish "externally written" entries from internally generated ones. When the sequence replays a loop entry, if it holds an externally written value it returns that value; if it holds a random value it returns 0.5 (center pitch). This creates a shift-register melody effect where the external pitch gradually fills and displaces the random content.

**Control modes.** The three X channels can be configured in three ControlMode patterns:
- `CONTROL_MODE_IDENTICAL`: all three channels use the same spread, bias, and steps.
- `CONTROL_MODE_BUMP`: the center channel (X2) gets the full parameter value; X1 and X3 get the inverted amount, creating a bump distribution across the three outputs.
- `CONTROL_MODE_TILT`: the amount varies linearly from -1 (X1) to +1 (X3), tilting the bias across the three channels.

**Y output.** Y is a fourth channel with separate `spread`, `bias`, `steps` settings and a clock divider ratio controlled by a 12-position `y_divider` state. The ratio is selected from the `y_divider_ratios[]` table (ranging from 1:64 to 1:1). `RampDivider` generates a ramp at that fraction of the X clock rate, and Y changes voltage on each Y-clock tick. Y has no Deja Vu.

### Controlled Randomness: the RandomSequence and Deja Vu

`RandomSequence` (`random/random_sequence.h`) is the core abstraction for all random value generation in both sections. It maintains:

- `loop_[16]`: a circular buffer of the most recent 16 random values (the "Deja Vu loop").
- `history_[16]`: a secondary ring buffer recording every value returned from `NextValue()`.
- `loop_write_head_`: write pointer into the loop.
- `length_`: how many steps of the loop are currently active (1–16).
- `step_`: current read position within the loop.
- `deja_vu_`: the Deja Vu knob value, in [0, 1].

`NextValue()` computes `p = (2 * deja_vu - 1)²` and draws a uniform random to decide whether to `mutate`:

- If `mutate` and `deja_vu < 0.5`: generate a new value, overwrite the oldest position in the loop, and advance `step_` to that position. This makes the loop evolve continuously at low Deja Vu.
- If not `mutate` (middle of knob): advance `step_` cyclically. The loop repeats exactly. This is the full Deja Vu locked state.
- If `mutate` and `deja_vu > 0.5`: jump `step_` to a random position within the loop. The sequence repeats from the loop but in a scrambled order.

At exactly 0.5, `deja_vu` is set to 0.5 (enforced by a dead band in `Process()`), which always takes the non-mutate branch — perfect looping. A secondary `DEJA_VU_LOCKED` state in the UI forces `deja_vu` to 0.5 regardless of the knob.

The shared `loop_length` quantizer in `marbles.cc` maps the Deja Vu Length CV to integer loop lengths from 1 up to 16 (with weighted probabilities favoring musically useful values like 2, 3, 4, 6, 8, 12, 16).

**Sequence sharing across X channels.** When all three X channels follow the same clock, they share a single `RandomSequence` (channel 0's sequence). Channels X2 and X3 call `ReplayPseudoRandom(hash)` with fixed hash constants (`0xbeca55e5` and `0xf0cacc1a`). This replays the same history but XORs each stored value with the hash before applying an LCG step, deriving independent-but-deterministic sequences from one loop. In register mode, channels instead call `ReplayShifted(i)`, which reads `history_[head - i]` — offsetting the read pointer to create a shift register effect.

### Quantization

`Quantizer` (`random/quantizer.cc`) implements a variable-resolution musical scale quantizer. A `Scale` stores up to 16 `Degree` entries, each with a voltage and a weight byte. The quantizer precomputes 7 bitmask levels at fixed weight thresholds (0, 16, 32, 64, 128, 192, 255). The `steps_` parameter is mapped to one of these levels by a `HysteresisQuantizer`. At each level only degrees with weight above the threshold are active. Quantization finds the tightest active lower and upper neighbor and picks the nearer one. This creates a musically useful hierarchy where the most structurally important scale degrees (weight 255) appear even at low `steps_` values, and chromatic passing tones only appear at high values. Six scale slots are stored per channel, and the active slot is selected by the `x_scale` state. Users can record a custom scale by playing notes into the CV input while the module is in scale record mode (`ScaleRecorder`).

### CV Input Modulation

`CvReader` (`cv_reader.cc`) reads all ADC channels through `CvReaderChannel` instances that apply a configurable one-pole lowpass filter and optional attenuverter scaling. The resulting float parameters feed directly into `TGenerator` and `XYGenerator` every block. Seven primary parameters have CV inputs: T Rate, T Bias, T Jitter, X Spread, X Bias, X Steps, and Deja Vu Amount. Gate inputs (T clock, X clock) are read as edge-detected `GateFlags` bitfields by the `ClockInputs` driver and processed per-sample within each block. A factory calibration procedure stores per-channel ADC scale and offset coefficients in flash via `Settings`.

### Non-Obvious Algorithmic Choices

**Ramp-centric architecture.** Rather than working with gate events, all internal timing is represented as continuous sawtooth ramps in [0, 1]. The gate outputs are derived from the ramp phases (gate high while phase < pulse_width). This makes it easy for the XY section to observe exactly where the t clocks are within their cycles and to derive coherent divided/multiplied clocks without edge-detection artifacts.

**Beta distribution for voltage generation.** The choice of the beta distribution gives the spread and bias parameters intuitive musical meanings: at low spread the output clusters near the bias (like a tuned oscillator with slight vibrato); at medium spread the output covers the full range with a smooth hill (like a random walk); at high spread the output tends toward the extremes (like a random binary choice between two pitch regions). The ICDF table approach avoids the computational cost of rejection sampling on an MCU.

**Jitter with mean-reversion.** The phase difference accumulator in `TGenerator` ensures that jitter does not cause long-term tempo drift. A jittery clock that has been running fast will systematically apply a slow-down correction factor on the next step, keeping the jittered clock centered on the master.

**Self-patching detection.** The `ClockSelfPatchingDetector` compares the XY clock input against each t gate output sample by sample, counting synchronous matches and error streaks. This lets the firmware detect that a user has patched, say, T2 into the X clock, and use the already-computed T2 ramp directly rather than re-running `RampExtractor` on what is effectively the same signal — avoiding double-processing and maintaining phase coherence.

**Reacquisition counter for shift register.** `OutputChannel` sets a 20-sample reacquisition window after a gate rising edge. During this window `RewriteValue()` is called instead of `NextValue()`, allowing the output to track the still-slewing CV input and overwrite the just-recorded loop entry. This compensates for the notoriously bad CV slew on common sequencers and MIDI-CV interfaces.
