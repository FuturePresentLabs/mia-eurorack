# Clouds — Code Architecture

Clouds is a granular audio processor for the Eurorack modular synthesizer format. It continuously
captures stereo (or mono) audio into a circular RAM buffer and plays back overlapping "grains" —
short windowed excerpts of that buffer — with independent control over playhead position, grain
size, pitch transposition, density, and texture. Four distinct playback modes share the same
hardware and CV interface: granular synthesis, WSOLA-based time-stretching, a looping delay, and
a phase vocoder for spectral processing. A Dattorro-topology plate reverb and an allpass diffuser
are applied globally as post-processing.

Target platform: STM32F4 at 32 kHz, 32-sample audio blocks. No dynamic allocation — all memory is
pre-partitioned at startup.

---

## Code Architecture

### Directory and File Structure

```
clouds/
  clouds.cc                    — main() and interrupt handlers; hardware wiring
  cv_scaler.cc / .h            — ADC reading, CV/pot scaling, blend parameter logic
  settings.cc / .h             — persistent calibration and user settings (flash)
  ui.cc / .h                   — LED feedback, button handling, factory-test mode
  meter.h                      — VU metering
  resources.cc / .h            — pre-computed lookup tables (grain size, pitch, windows)

  drivers/
    adc.cc / .h                — 10-channel ADC driver (DMA-based)
    codec.cc / .h              — WM8731 I2S codec driver; owns the audio callback
    gate_input.cc / .h         — trigger/gate edge detection
    leds.cc / .h               — RGB LED driver
    switches.cc / .h           — button driver with debounce

  dsp/
    parameters.h               — Parameters struct (POD): all knob/CV values as floats
    frame.h                    — FloatFrame / ShortFrame (stereo sample pair structs)
    audio_buffer.h             — AudioBuffer<Resolution>: circular buffer with r/w heads
    grain.h                    — Grain: single grain renderer (envelope + interpolated read)
    granular_sample_player.h   — GranularSamplePlayer: grain pool scheduler
    wsola_sample_player.h      — WSOLASamplePlayer: WSOLA time-stretch player
    looping_sample_player.h    — LoopingSamplePlayer: looping delay with tap-tempo sync
    sample_rate_converter.h    — SampleRateConverter<ratio>: polyphase FIR SRC
    correlator.cc / .h         — Correlator: sign-bit XOR correlation for splice-point search
    window.h                   — Window helper used by WSOLA player
    mu_law.cc / .h             — mu-law encode/decode LUT
    granular_processor.cc / .h — GranularProcessor: top-level DSP class
    fx/
      fx_engine.h              — FxEngine<size>: templated delay-line engine for FX
      diffuser.h               — Diffuser: 4-stage allpass stereo diffuser
      reverb.h                 — Reverb: Griesinger plate reverb
      pitch_shifter.h          — PitchShifter: used in looping-delay mode
    pvoc/
      stft.h / stft.cc         — STFT: short-time Fourier transform (overlap-add)
      frame_transformation.h / .cc — FrameTransformation: per-frame spectral operations
      phase_vocoder.h / .cc    — PhaseVocoder: ties STFT + FrameTransformation together
```

### Audio I/O Pipeline

The codec driver (`Codec`) operates at 32 kHz with 32-sample blocks. It calls `FillBuffer()` (in
`clouds.cc`) from the DMA interrupt on every block boundary.

```
DMA ISR → FillBuffer(input, output, 32)
            ├── CvScaler::Read()          — sample ADC, update Parameters struct
            └── GranularProcessor::Process(ShortFrame*, ShortFrame*, 32)
                  ├── int16→float conversion
                  ├── mono mix-down (if num_channels_ == 1)
                  ├── feedback injection (hp-filtered, soft-limited)
                  ├── optional 2× downsample (SampleRateConverter src_down_)
                  ├── ProcessGranular()    — mode-specific playback
                  ├── optional 2× upsample (src_up_)
                  ├── Diffuser::Process()
                  ├── optional PitchShifter::Process()
                  ├── LP/HP filter pair (looping-delay and stretch modes only)
                  ├── Reverb::Process()
                  └── dry/wet crossfade → SoftConvert → int16 output
```

The main loop calls `GranularProcessor::Prepare()` between interrupts — this does the heavy
background work (WSOLA correlator search, phase-vocoder FFT buffering) that is too expensive to
run inside the ISR.

### Buffer Management

`AudioBuffer<Resolution>` (`dsp/audio_buffer.h`) is a statically-allocated circular buffer
parameterised by storage format. Three formats are supported:

- `RESOLUTION_16_BIT` — 16-bit signed PCM; best quality, lowest record time.
- `RESOLUTION_8_BIT_MU_LAW` — 8-bit mu-law compressed; doubles record time at perceptible but
  musically useful fidelity loss. Encode/decode via `Lin2MuLaw` / `MuLaw2Lin` LUTs.
- `RESOLUTION_8_BIT_DITHERED` — 8-bit with error diffusion dithering.

`GranularProcessor` owns two sets of these (`buffer_8_[2]` and `buffer_16_[2]`), one per channel.
The actual backing memory comes from two statically allocated blobs in `clouds.cc`:

- `block_mem[118784]` in main SRAM — used as channel 0 sample buffer (or both channels in mono).
- `block_ccm[65408]` in CCM SRAM — used as channel 1 sample buffer (stereo) or FX workspace (mono).

`GranularProcessor::Prepare()` uses `stmlib::BufferAllocator` to sub-allocate from the workspace
portion: 2048 floats for the Diffuser, 16384 uint16s for the Reverb, and the correlator/pitch-
shifter data from the remainder.

`WriteFade()` handles the freeze transition cleanly: when `freeze` is asserted, recording halts but
256 samples are saved to a `tail_buffer_`. When recording resumes, those tail samples are
crossfaded back in over 256 samples to avoid clicks.

Buffer read positions are expressed as `(integral_sample << 16) | fractional_u16` — a fixed-point
Q16.16 representation used uniformly across all players and interpolators.

### Grain Pool and Scheduling — GranularSamplePlayer / Grain

`GranularSamplePlayer` maintains a flat array of up to 64 `Grain` objects (`kMaxNumGrains = 64`).
The actual limit is computed at init time: 40 grains for mono, 32 for stereo, scaled by
`low_fidelity` quality (up to 23/16 × base). Active vs. inactive grain tracking is done by
`FillAvailableGrainsList()`, which builds an index array of inactive slots each block.

Grain scheduling is dual-mode depending on the DENSITY knob position (below/above 0.5):

- **Deterministic (density < 0.5):** a phasor increments each sample; a new grain fires when it
  crosses `space_between_grains = grain_size_hint / target_num_grains`. This produces evenly
  spaced, repeating grains.
- **Probabilistic (density > 0.5):** each sample rolls `Random::GetFloat() < p` where
  `p = target_num_grains / grain_size_hint`. This produces stochastic clouds with irregular timing.

An external trigger input bypasses both modes and fires one grain immediately on the rising edge.

`ScheduleGrain()` translates normalized parameters into per-grain integers. Pitch is converted via
`SemitonesToRatio()` → stored as a Q16.16 fixed-point `phase_increment`. Position is mapped to a
start sample offset behind the current write head, accounting for both `eaten_by_play_head`
(grain_size × pitch_ratio samples consumed by the faster/slower playhead) and
`eaten_by_recording_head` (grain_size samples newly written during playback), so grains never
collide with the write cursor.

Each `Grain::OverlapAdd()` is a tightly templated inner loop dispatched on `num_channels` (1 or 2)
and `GrainQuality` (LOW/MEDIUM/HIGH). Quality determines the interpolation method:
zero-order hold, linear, or 4-point Hermite (`ReadHermite()` uses Laurent de Soras's cubic
polynomial). Grains in the upper quarter of the pool get `GRAIN_QUALITY_HIGH`; the lower three
quarters get `GRAIN_QUALITY_MEDIUM` to manage CPU budget.

The envelope is a triangular ramp (0→1→0) over the grain width, modified by `window_shape`:
- `window_shape < 0.5` sharpens the attack/decay (trapezoidal tendency via `envelope_slope`).
- `window_shape >= 0.5` interpolates toward a Hann window using a `lut_window` lookup
  (`envelope_smoothness` controls the crossfade amount).

Output normalization uses Carmack's fast inverse square root: `gain = fast_rsqrt_carmack(num_grains - 1)`,
applied per-sample through a one-pole smoother to avoid gain clicks.

Stereo panning for mono-source grains uses a sin/cos LUT; for stereo sources, a mid-side–like
matrix mixes `l` and `r` channels weighted by a random pan position.

### WSOLA Time-Stretching — WSOLASamplePlayer / Correlator

`WSOLASamplePlayer` implements Waveform Similarity Overlap-Add (WSOLA). It maintains two
overlapping `Window` segments that advance through the buffer at a rate determined by the pitch
ratio. When a window expires, a new splice point is found by `Correlator`.

`Correlator` uses sign-bit XOR correlation — each 32-sample window is packed into a single
`uint32_t` word (one bit per sign), so 32 samples are compared in a single XOR + `__builtin_popcount`
operation. This is fast enough to evaluate hundreds of splice candidates spread across the background
`Prepare()` call via `EvaluateSomeCandidates()`.

`LoadCorrelator()` downsamples the search region to sign bits via `ReadSignBits<N>()`, which
strides through the buffer at a rate that compresses the window to ≤2048 words regardless of the
actual window size. The resulting `best_match()` offset is used by `ScheduleAlignedWindow()` as the
start point for the next grain.

### Looping Delay — LoopingSamplePlayer

In non-frozen state, `LoopingSamplePlayer` operates as a smoothly interpolating variable delay line.
`position` maps to a target delay up to `buffer_size - 64` samples; `current_delay_` tracks it with
a 0.00005/sample slew to suppress audible pitch modulation from fast sweeps. Hermite interpolation
is used for the fractional delay read.

In frozen state, it becomes a looper: a `phase_` counter advances at the rate of `SemitonesToRatio(pitch)`,
looping within `[loop_point, loop_point + loop_duration]`. Crossfade at loop boundary uses a 64-sample
tail buffer. Tap-tempo synchronization via the trigger input sets `tap_delay_` (in samples) and locks
`loop_duration` to that measured period, bypassing the size knob.

### Spectral Mode — PhaseVocoder / FrameTransformation / STFT

`PhaseVocoder` wraps two `STFT` instances (one per channel) backed by the full `block_mem`/`block_ccm`
memory (the sample buffers are reused as FFT workspace in this mode). The FFT size scales with
available memory and is computed at `Init()`. A `lut_sine_window_4096` Hann window is used for
analysis.

`FrameTransformation::Process()` performs the per-frame spectral manipulations in polar form:

1. **RectangularToPolar** — converts FFT output to magnitudes + phase deltas using fast atan2.
2. **StoreMagnitudes** — writes magnitude frames into a ring of up to 6 texture buffers at the
   position indexed by `parameters.position`. `density` controls how quickly new frames replace old
   ones (stochastic gating at low density, smooth crossfade at high density).
3. **ReplayMagnitudes** — reads back a crossfade between two adjacent texture buffers.
4. **WarpMagnitudes** — applies a cubic frequency warp controlled by `parameters.size`. Six
   polynomial coefficient sets are interpolated via `kWarpPolynomials` to morph bin-remapping curves.
5. **ShiftMagnitudes** — pitch-shifts by resampling the magnitude spectrum.
6. **QuantizeMagnitudes** — at low texture: reduces amplitude resolution (integer quantization); at
   high texture: applies a non-linear spectral reshaping curve.
7. **SetPhases** — propagates stored phase deltas scaled by pitch_ratio; optionally adds random
   phase noise controlled by `parameters.spectral.phase_randomization` (mapped from density).
8. **PolarToRectangular** — converts back using a sin LUT (`fast_p2r`). High-frequency bins above
   `kHighFrequencyTruncation = 16` from the top are zeroed (implicit anti-aliasing).

A gate (trigger hold) activates `AddGlitch()`, which applies one of four randomly pre-selected
spectral destructors: magnitude trails with exponential blow-up, aliased spectral shift, dominant
harmonic annihilation, or stochastic high-pass.

### Reverb

`Reverb` uses the Griesinger plate topology described in the Dattorro (1997) paper: four allpass
diffusers on the input (lengths 113, 162, 241, 399 samples), then a symmetric recirculating loop
of two allpass pairs (1653+2038 and 1913+1663) and two long delays (3411 and 4782 samples). Two
LFO-modulated taps create the characteristic shimmer/chorus. Storage is 12-bit compressed via
`FxEngine<16384, FORMAT_12_BIT>`.

`FxEngine` is a compile-time–typed delay memory allocator. `Reserve<N, Tail>` recursively encodes
delay lengths as template parameters; `DelayLine<Memory, index>` computes each line's base offset
at compile time so all delay reads/writes reduce to a single masked array index. The `Context`
object accumulates a running float register (`accumulator_`) across `Read`, `Write`, `WriteAllPass`,
and `Lp` operations, making the reverb topology readable as nearly linear pseudocode. LFOs update
every 32 samples (one per block) using `CosineOscillator`.

The `Diffuser` uses the same `FxEngine` infrastructure with `FORMAT_32_BIT` (no compression), eight
allpass delays per stereo channel, fixed coefficient 0.625.

### CV and Parameter Mapping — CvScaler

`CvScaler::Read()` is called every audio block from the DMA ISR. It reads 10 ADC channels
(potentiometers and CV inputs) through a per-channel `CvTransformation` struct that specifies:
sign inversion (`flip`), offset removal for calibrated CV inputs (`remove_offset`), and a one-pole
LP filter coefficient (ranging from 0.01 for the V/oct CV to 0.2 for blend CV — faster for
real-time performance, slower for slow-moving knobs).

The V/oct input uses a `calibration_data_->pitch_scale / pitch_offset` linear fit stored in flash,
with a hysteresis-guarded one-pole smoother that snaps immediately on large jumps (>0.5 semitone)
to avoid missing fast glides. Pitch from both the pot and V/oct are summed and passed through
`lut_quantized_pitch` (a 1024-entry LUT) for quantization to semitones.

The Blend knob controls one of four parameters (dry/wet, reverb, feedback, stereo spread) selected
by button. The knob uses a "pickup" mode — it tracks relative movement once it reaches the stored
value — implemented by computing a `skew_ratio` that scales incoming delta proportional to remaining
headroom on each side of the current stored value.

Freeze, trigger, and gate inputs are read through `GateInput` edge detectors, with `kAdcLatency`-
sample pipeline delay compensation (a small shift register of booleans) to align gate timing with
the ADC conversion cycle.

### Notable Implementation Details

- **No malloc:** all memory is pre-allocated as static arrays in `clouds.cc` and sub-divided using
  `stmlib::BufferAllocator`. Changing quality or channel count sets `reset_buffers_ = true`;
  `Prepare()` re-partitions the memory safely from the main loop.
- **Quality / fidelity tradeoff:** `set_quality()` encodes two independent bits — mono/stereo
  (`num_channels_`) and normal/low-fidelity (`low_fidelity_`). Low-fidelity mode halves the
  effective sample rate via `SampleRateConverter<-2>` / `<+2>` and switches to `RESOLUTION_8_BIT_MU_LAW`
  storage, doubling available record time at the cost of bandwidth.
- **Feedback path:** the output before reverb is copied to `fb_[]` and injected into the input on
  the next block, high-pass filtered at a frequency that rises with feedback amount to prevent DC
  buildup. At freeze, `fb_gain` is faded to zero via `freeze_lp_` to prevent screaming feedback.
- **Grain size LUT:** grain sizes are not linear in samples — `lut_grain_size` maps `[0,1]` to a
  range covering musically useful durations.
- **Hermite interpolator:** `AudioBuffer::ReadHermite()` implements Laurent de Soras's optimized
  cubic Hermite using only 3 multiplies and 8 adds via the identity factoring
  `((((a*t) - b_neg)*t + c)*t + x0)`.
- **Persistence:** `GetPersistentData()` / `LoadPersistentData()` serialize the write-head position
  and the raw sample buffer memory into tagged blocks (`FourCC<'s','t','a','t'>`,
  `FourCC<'b','u','f','f'>`), saved to one of four flash "sample memory" slots via `Settings`.
