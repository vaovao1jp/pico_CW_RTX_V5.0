# pico_40m_CW_RTXv5_0_J
## 7MHz CW Transceiver (Raspberry Pi Pico) — Technical Documentation V5.0

---

## Table of Contents

1. [Program Structure](#1-program-structure)
2. [Hardware Configuration](#2-hardware-configuration)
3. [Operation Flow (Detailed)](#3-operation-flow-detailed)
4. [How to Operate](#4-how-to-operate)
5. [Code Style and Naming Conventions](#5-code-style-and-naming-conventions)
6. [Parameter Reference and Effects of Changes](#6-parameter-reference-and-effects-of-changes)

---

## 1. Program Structure

### 1-1. File Layout

```
pico_40m_CW_RTXv5_0/
├── pico_40m_CW_RTXv5_0_EN.ino   Main sketch (all code, approx. 1200 lines)
└── DOCUMENT_V5.0_EN.md          This document
```

### 1-2. Sections Inside the Sketch

The `.ino` file is divided into nine sections separated by `// === [N] ===` comments.

| Section | Approx. lines | Contents |
|---|---|---|
| `[1]` Pin Definitions & Basic Settings | ~65 | `#define` pin numbers, DSP constants, defaults, EEPROM addresses |
| `[2]` Global Variables | ~165 | Frequency management, AGC state, IIR filter arrays, inter-core shared buffers, CW event queue, display variables |
| `[3]` Hardware & EEPROM | ~240 | EEPROM read/write utilities, Si5351 frequency calculation and setup, TX/RX switching functions |
| `[4]` DSP & Signal Processing | ~285 | CW demodulation (Hilbert transform + IIR filter), AGC function |
| `[5]` Keyer Processing | ~360 | WPM-to-timing calculation, iambic/straight-key state machine |
| `[6]` UI & Display | ~720 | Bandscope, waterfall, S-meter drawing; volume/WPM setting UI; button handling |
| `[7]` setup / loop (Core0) | ~850 | Initialization, UI main loop (FFT processing, display update, input handling) |
| `[8]` setup1 / loop1 (Core1) | ~1045 | ADC sampling, real-time DSP, PWM output (40 kHz) |
| `[9]` CW Decoder (Core0) | ~1191 | Morse code table, character conversion, text buffer management, event-driven decoder, text display |

### 1-3. Libraries Used

| Library | Version | Purpose |
|---|---|---|
| `Arduino.h` | Bundled with RP2040 core | Core API (`analogRead`, `digitalRead`, `millis`, etc.) |
| `Rotary.h` | Brianlow fork | Rotary encoder direction detection (interrupt-driven) |
| `U8g2lib.h` | Latest | 128×64 OLED monochrome rendering (SSD1306 / SH1106) |
| `Wire.h` | Bundled with RP2040 core | I2C bus shared by OLED and Si5351 |
| `arduinoFFT.h` | v2.0.2 | FFT computation for bandscope and waterfall |
| `si5351.h` | Etherkit fork | Si5351 clock generator control (3-channel independent output with phase setting) |
| `EEPROM.h` | Bundled with RP2040 core | Non-volatile settings storage (Flash-emulated EEPROM, up to 512 bytes) |

---

## 2. Hardware Configuration

### 2-1. Pin Assignment

| GPIO | Direction | Function | Notes |
|---|---|---|---|
| GPIO 0 | Input | Rotary encoder — channel A | Interrupt (CHANGE) |
| GPIO 1 | Input | Rotary encoder — channel B | Interrupt (CHANGE) |
| GPIO 2 | Input | Button 1 (STEP / WPM setting) | INPUT_PULLUP, LOW = pressed |
| GPIO 3 | Input | Button 2 (KEY MODE / Volume) | INPUT_PULLUP, LOW = pressed |
| GPIO 6 | Input | Paddle — dot input | INPUT_PULLUP, LOW = pressed |
| GPIO 7 | Input | Paddle — dash input / straight key | INPUT_PULLUP, LOW = pressed |
| GPIO 15 | Output | TX/RX antenna relay | HIGH = TX, LOW = RX |
| GPIO 16 | Output | Speaker PWM output | 12-bit resolution / 44.1 kHz |
| GPIO 25 | Output | Process indicator LED | HIGH during FFT computation |
| GPIO 26 | Input | I-signal ADC input | 12-bit (0–4095), direct-conversion RX I component |
| GPIO 27 | Input | Q-signal ADC input | 12-bit (0–4095), direct-conversion RX Q component |
| SDA/SCL | I2C | OLED & Si5351 shared bus | Default I2C bus |

### 2-2. Si5351 Clock Assignment

| Channel | TX / RX | Frequency | Description |
|---|---|---|---|
| CLK0 | ON during RX / OFF during TX | VFO − 70 kHz | Local oscillator (I component, phase 0°) |
| CLK1 | ON during RX / OFF during TX | VFO − 70 kHz | Local oscillator (Q component, phase 90°) |
| CLK2 | ON during TX / OFF during RX | VFO | Transmit carrier (CW keying) |

**Meaning of the BFO offset (CW_AUDIO_OFFSET = 70 kHz):**  
If a CW signal falls exactly on the VFO frequency, the audio output would be 0 Hz (silence).  
Setting the VFO 700 Hz above the signal would give a 700 Hz demodulated tone.  
In practice, CLK0/1 are set to VFO − 70 kHz (to maintain Si5351 phase accuracy).  
The IF after the mixer is then 70 kHz ± signal offset, and the IIR bandpass filter passes only the 700 Hz region.

**Setting the 90° phase (Freq_Set function):**
```cpp
int phase = pll_freq / (FREQ_ULL - CW_AUDIO_OFFSET) + 0.5;
si5351.set_phase(SI5351_CLK0, 0);
si5351.set_phase(SI5351_CLK1, phase);
si5351.pll_reset(SI5351_PLLA);
```
The Si5351 phase register specifies "how many PLL clock cycles to delay."  
`phase = PLL frequency ÷ output frequency` gives an exact 90° shift.

### 2-3. Direct Conversion Receiver Architecture

```
Antenna
  │
  ├─── Bandpass filter (external, 7 MHz band)
  │
  ├─── Mixer ── Si5351 CLK0 (I, 0°)  ───→ GPIO26 (ADC)
  │        └── Si5351 CLK1 (Q, 90°) ───→ GPIO27 (ADC)
  │
  └─── TX: Si5351 CLK2 (carrier) → Antenna
```

Using two quadrature paths (I and Q) enables USB/LSB selection.  
This transceiver uses `audioSignal = I − Q_delayed` (I minus 90°-delayed Q) to cancel the LSB component, extracting the CW signal (equivalent to USB).

### 2-4. OLED Screen Layout (128 × 64 pixels)

```
 y=  0 ┌──────────────────────────────────────────────────────────────────────────────┐
       │ 7.014.000          STEP: 1000Hz                                              │  ← VFO frequency (large)
 y=  1 │                   S: ████████░░░░░░░░░░░░░░░░░░░░░░░░░░░░                 │  ← S-meter
 y=  6 │──────────────────────────────────────────────────────────────────────────────│
       │   B a n d s c o p e  ( p o l y l i n e  +  p e a k  h o l d )               │
       │   Left = VFO−15 kHz    Center = VFO    Right = VFO+15 kHz                   │
 y= 26 ├──────────────────────────────────────────────────────────────────────────────┤  ← divider line
       │   W a t e r f a l l  ( 2 1  r o w s ,  t o p  =  m o s t  r e c e n t )     │
 y= 47 │                                                                              │
 y= 48 ├──────────────────────────────────────────────────────────────────────────────┤  ← divider line
 y= 49 │ 6.999         7.014         7.029                                            │  ← frequency scale
 y= 54 ├──────────────────────────────────────────────────────────────────────────────┤
       │                            C Q C Q D E J R 3 X N W                          │  ← CW decoded text
 y= 63 └──────────────────────────────────────────────────────────────────────────────┘
```

**Characteristics of the CW decoded text area:**
- 5×8 pixel font, 7 px per character (5 px glyph + 2 px gap)
- Maximum 18 characters per line
- The newest character is always anchored to the right edge; older characters scroll left
- Previously decoded text remains visible during TX

---

## 3. Operation Flow (Detailed)

### 3-1. Dual-Core Design Overview

The RP2040 has two cores, each running its own independent loop.

```
┌──────────────────────────────┐       ┌────────────────────────────────┐
│  Core0 (loop)                │       │  Core1 (loop1)                 │
│  Priority: UI & control      │       │  Priority: real-time DSP       │
│                              │       │                                │
│  Cycle: variable (~50 ms)    │       │  Cycle: exact 25 µs (40 kHz)  │
│                              │       │                                │
│  · Button / encoder handling │       │  · I/Q ADC sampling            │
│  · Iambic keyer control      │◀─────▶│  · DC offset removal           │
│  · CW event consume & decode │       │  · Write to shared buffer      │
│  · FFT computation           │       │  · CW demodulation (IIR)       │
│  · OLED rendering            │       │  · SNR ratio calculation       │
│  · EEPROM save               │       │  · CW event generation         │
│                              │       │  · AGC                         │
│                              │       │  · PWM output                  │
└──────────────────────────────┘       └────────────────────────────────┘

       Inter-core shared data (volatile variables)
       ┌────────────────────────────────────┐
       │ sharedBufferI[256]                 │ Core1→Core0 (I/Q data for FFT)
       │ sharedBufferQ[256]                 │
       │ sharedBufferReady                  │ Core1→Core0 (buffer-ready flag)
       │ cwEvBuf[16] (CWEvent ring buffer)  │ Core1→Core0 (mark/space duration events)
       │ cwEvWr / cwEvRd (buffer pointers)  │
       │ cwEnvelope                         │ Core1→Core0 (SNR ratio, read-only)
       │ transmitting                       │ Core0→Core1 (TX state)
       │ volumeMultiplier                   │ Core0→Core1 (volume multiplier)
       │ muteCounter                        │ Core0→Core1 (tuning mute counter)
       └────────────────────────────────────┘
```

**Why dual-core is necessary:**  
FFT computation and OLED rendering together take 30–50 ms. During this time Core0 stalls, but Core1 runs independently, preserving real-time audio output and CW detection.  
In particular, CW element timing (measuring dot/dash durations) requires accuracy within ±2 ms — impossible for a Core0 that pauses for 50 ms. Using Core1's 40 kHz sample counter solves this.

---

### 3-2. Core1 Detailed Flow (loop1 function)

#### Timing Control

```cpp
const unsigned long targetInterval = 25;   // 25 µs = 40 kHz
if (now - lastSampleTime < targetInterval) return;
lastSampleTime += targetInterval;
```

Incrementing `lastSampleTime` by a fixed step prevents cumulative timing drift caused by variable processing time.  
(Using `lastSampleTime = now` would accumulate error each iteration.)

RX processing (4× oversampling, etc.) actually takes about 26–30 µs, so `lastSampleTime` gradually falls behind `now`. The loop self-corrects by immediately re-running when `now - lastSampleTime` exceeds `targetInterval`.

**TX start timestamp synchronization:**  
TX processing takes only ~2 µs, so the lag built up in `lastSampleTime` during RX is discharged all at once when TX starts. This causes hundreds of catch-up iterations, causing `toneAngle` to advance rapidly and produce an extremely high-pitched tone.  
Detecting TX start with `transmitting && !prevTxState` and resetting `lastSampleTime = now` prevents this.

#### Step 1: ADC Sampling (4× Oversampling)

```cpp
long sumI = 0, sumQ = 0;
for (int i = 0; i < 4; i++) {
    sumI += analogRead(inputPinI);
    sumQ += analogRead(inputPinQ);
}
float rawI = ((float)sumI / 4.0f / 2047.5f) - 1.0f;
float rawQ = ((float)sumQ / 4.0f / 2047.5f) - 1.0f;
```

**Effect of 4× oversampling:**  
Summing four samples and averaging them multiplies the signal by 4 and random noise by √4 = 2, improving SNR by a factor of 2 (+6 dB theoretically).  
Real ADCs include correlated noise, so the practical improvement is closer to +3 dB.

ADC output (0–4095) is normalized to −1.0…+1.0 using `(value / 2047.5) − 1.0`.

#### Step 2: Mute and DC Offset Removal

**What is DC offset?**  
In addition to the received signal, the ADC input includes a DC bias from the circuit.  
Left uncorrected, this produces a large spike at 0 Hz in the FFT, ruining the bandscope display.  
A very slow low-pass filter (time constant τ ≈ 1000 samples ≈ 25 ms) tracks and removes the DC component.

```cpp
dcOffsetI = (dcOffsetI * 0.999f) + (rawI * 0.001f);  // equivalent to 1000-sample moving average
rawI -= dcOffsetI;
```

**How smoothMute works:**  
When the frequency changes, Core0 sets `muteCounter = 500`.  
While `muteCounter > 0`, `smoothMute *= 0.80f` drives audio rapidly toward 0 (muted).  
When `muteCounter == 0`, `smoothMute` recovers gradually via `0.80 × prev + 0.20 × 1.0`, suppressing tuning pop/click noise.

| State | smoothMute change | Effect |
|---|---|---|
| muteCounter > 0 | × 0.80 per loop (rapid decay) | Nearly 0 in ~10 loops (0.25 ms) |
| muteCounter = 0 | 0.80 × prev + 0.20 (slow recovery) | τ ≈ 5 loops (0.125 ms) from 0 to 1 |

#### Step 3: Writing to the Shared Buffer

```cpp
sharedBufferI[sharedIndex] = mutedI;
sharedBufferQ[sharedIndex] = mutedQ;
sharedIndex++;
if (sharedIndex >= SAMPLES) {       // 256 samples = 6.4 ms worth
    sharedIndex = 0;
    sharedBufferReady = true;        // notify Core0
}
```

`sharedBufferReady = true` is set every time 256 samples accumulate, notifying Core0.  
Core1 begins filling the next buffer immediately; it is not affected by how long Core0 takes to process.  
(Note: this is a single buffer, not double-buffered.)

#### Step 4a: CW Demodulation (cwDemodulate function)

**① Hilbert transform approximation (90° delay of Q signal)**

```
I signal (current)        : cos(ωt)
Q signal (14 samples ago) : cos(ωt − 90°) = sin(ωt)  ← delayed via ring buffer

audioSignal = I − Q_delayed
            = cos(ωt) − sin(ωt) × j  (LSB component cancelled)
```

At 700 Hz sampled at 40 kHz, 90° corresponds to exactly 14.28 samples ≈ 14 samples.  
`Q_DELAY_SAMPLES = 14` is derived from this calculation.

In theory this processing provides infinite rejection of the mirror image (unwanted sideband), but in practice amplitude and phase imbalances between I and Q limit the cancellation ratio (improved by the `qGain` correction described later).

**② 4th-order IIR bandpass filter**

Two cascaded 2nd-order biquad sections form a 4th-order filter.

```
Filter specification:
  Sampling frequency : 40 kHz
  Center frequency   : 700 Hz
  Q factor           : 3.5 (bandwidth ±100 Hz)
  Type               : Bandpass (Butterworth equivalent)

Difference equation (each stage):
  y[n] = b0 × x[n] + b2 × x[n-2] − a1 × y[n-1] − a2 × y[n-2]
  b0 = b2 = 0.024122
  a1 = −1.939356 (sign-inverted addition)
  a2 =  0.951756

State variables:
  iir_x1[2], iir_x2[2]  inputs 1 and 2 samples ago
  iir_y1[2], iir_y2[2]  outputs 1 and 2 samples ago
  [0] = first stage, [1] = second stage
```

**Comparison with FIR filter:**

| Property | IIR 4th-order | Equivalent FIR quality |
|---|---|---|
| Computation | 8 multiplies, 6 adds | 100–200 multiplies |
| Memory | 8 variables | 100–200 variables |
| Phase response | Non-linear (acceptable) | Linear phase possible |
| Stability | Depends on coefficients (this design is stable) | Always stable |

IIR is optimal for CW demodulation because linear phase response is not required.

**To change the bandwidth:**  
The filter coefficients (`b0`, `b2`, `a1`, `a2`) must be recalculated using an online biquad filter design tool.  
Simply modifying parameters in the code does not change the center frequency or Q.  
(If `CW_TONE` is changed, the coefficients must also be recalculated to match.)

#### Step 4b: CW Signal Detection (SNR-Ratio Method)

Rather than comparing against a fixed threshold, detection uses the ratio to the noise floor (SNR ratio), enabling stable detection even as signal strength varies widely.

```
Processing flow:
  absD = |demodulated|  (absolute value)

  cwEnvLP: envelope tracking via asymmetric low-pass filter
    absD > cwEnvLP  → fast tracking (attack):  τ = 1/0.05  = 20 samples ≈ 0.5 ms
    absD ≤ cwEnvLP  → slow tracking (decay):   τ = 1/0.002 = 500 samples ≈ 12.5 ms

  cwNoise: estimated noise floor
    SNR ratio < 1.5 (no signal): faster update  τ = 1/0.0005  = 2000 samples ≈ 50 ms
    SNR ratio ≥ 1.5 (signal present): very slow  τ = 1/0.00001 = 100000 samples ≈ 2.5 s

  cwEnvelope = cwEnvLP / cwNoise  → shared with Core0
```

**Role of the asymmetric LPF:**  
When the key is pressed, the envelope rises quickly (tracked within 0.5 ms).  
When the key is released, it falls slowly (decays over 12.5 ms).  
This introduces hysteresis in key-up/down detection, reducing noise-induced false triggers.

#### Step 4c: CW Event Generation (debounce inside Core1)

```
raw = (cwEnvelope > CW_DETECT_THRESHOLD)

State change detected (raw ≠ cwSt):
  cwDB++
  When cwDB ≥ DB_TH (80):
    ms = cwCnt / 40  (convert sample count → ms)
    Write event to ring buffer:
      type = cwSt ? 1 : 0   (type of state that just ended)
        type=1: mark ended (key-up confirmed) → reports mark duration
        type=0: space ended (key-down confirmed) → reports space duration
      durMs = ms
    cwSt = raw (confirmed new state)
    cwCnt = 0, cwDB = 0

No state change (raw == cwSt):
  cwDB = 0 (reset debounce counter)
cwCnt++ (incremented every loop)
```

**Why debouncing is necessary:**  
Received signals can momentarily cross the detection threshold due to noise or fading.  
Requiring `DB_TH = 80` consecutive samples (= 2 ms) of the same state before confirming ignores short-lived fluctuations.

**Ring buffer (CW_EVBUF_SIZE = 16):**
```
cwEvBuf[0..15]  event storage array
cwEvWr          Core1 write position (0–15, wraps around)
cwEvRd          Core0 read position (0–15, wraps around)

Full check:  (cwEvWr + 1) % SIZE == cwEvRd  → skip write
Empty check: cwEvRd == cwEvWr               → skip read
```
Even though Core0 reads only every ~50 ms, a 16-event buffer is sufficient that normal CW speeds (5–40 WPM) will never overflow it.

#### Step 5: AGC (Automatic Gain Control)

```cpp
float error = targetAmplitude - fabs(input);
if (error > 0) agcGain += attackRate * error;   // weak signal → increase gain
else           agcGain += decayRate  * error;   // strong signal → decrease gain
agcGain = constrain(agcGain, minGain, maxGain);
return input * agcGain;
```

**Asymmetric time constants:**  
`attackRate (0.01) > decayRate (0.001)` to quickly reduce gain when a strong signal arrives and prevent clipping.  
Approximate gain change speeds:

| Condition | Time constant (τ) | Approximate settling time |
|---|---|---|
| Weak signal → gain increase | 1/0.01 = 100 loops ≈ 2.5 ms | Reaches target in a few ms |
| Strong signal → gain decrease | 1/0.001 = 1000 loops ≈ 25 ms | Settles in tens of ms |

While `smoothMute < 0.9` (fade-in period immediately after TX→RX), AGC is bypassed and the held `agcGain` value is used directly, preventing gain hunting during the fade-in.

#### Step 6: PWM Output

```cpp
uint16_t pwmOutput = (uint16_t)((agcOutput + 1.0f) * 2047.5f);
analogWrite(speakerPin, constrain(pwmOutput, 0, 4095));
```

`agcOutput` ranges from −1.0 to +1.0; multiplying by `(value + 1.0) × 2047.5` maps it to 0–4095 (12-bit).  
PWM frequency is `pwmFrequency = 44100 Hz`, which is above the audible range, so no external LPF is needed to drive a speaker.

---

### 3-3. TX Operation Flow (Core1)

#### TX Start to Steady State

```
Key pressed
  ↓
startTransmit():
  transmitting = true
  CLK0/1 (RX LO) disabled
  GPIO15 HIGH (antenna relay → TX)
  CLK2 (TX carrier) set to FREQ and enabled
  ↓
Core1 loop1():
  TX start detected: lastSampleTime = now (timestamp sync)
  txEnvelope += 0.002 per loop
  txEnvelope: 0 → 1 over ~500 loops (12.5 ms) — fade-in

  toneAngle += 2π × 700 / 40000 per loop (accurate 700 Hz)
  sineValue = sinf(toneAngle)
  txPwm = (sineValue × 0.1 × txEnvelope + 1.0) × 2047.5

  Crossfade at TX start (while txEnvelope < 0.05):
    blend = txEnvelope / 0.05  (0 → 1)
    blendedPwm = lastRxPwm × (1 − blend) + txPwm × blend
    → Smooth transition from last RX output to 700 Hz tone (prevents pop)
```

#### TX End to RX Return

```
Key released
  ↓
stopTransmit():
  transmitting = false
  CLK2 disabled
  CLK0/1 (RX LO) re-enabled
  GPIO15 LOW (antenna relay → RX)
  ↓
Core1 loop1():
  txEnvelope -= 0.002 per loop (fade-out)
  After txEnvelope reaches 0:
    if wasTransmitting == true, execute TX→RX cleanup:
      dcOffsetI/Q = 0 (reset DC tracking)
      sharedIndex = 0 (reset FFT buffer)
      Zero all IIR filter state (prevent transient response)
      Zero qDelayBuffer (reset Hilbert delay buffer)
      smoothMute = 0.0 (triggers RX fade-in)
      lastRxPwm = 2047.5 (reset to midpoint for next TX start)
```

---

### 3-4. Core0 Detailed Flow (loop function)

```
Executes every loop (~50 ms period):
  1. handleKeyModeButton()   Edge detection + long-press detection for Button 2
  2. handleKeyer()           Iambic/straight-key control
  3. handleCWDecoder()       Consume CW events, decode characters
  4. Frequency change detection
     If FREQ ≠ FREQ_OLD:
       muteCounter = 500 (sent to Core1 to mute audio)
       Freq_Set() updates Si5351
       saveFrequencyToEEPROM()
  5. STEP_BUTTON check → Fnc_Stp()

  When sharedBufferReady == true (every ~6.4 ms = 256 samples / 40 kHz):
  6. Q amplitude correction
     Compute qGain from RMS ratio of I and Q (clamped to 0.8–1.25)
     vImag[] = sharedBufferQ[] × qGain
  7. FFT computation
     Apply Hamming window → inverse FFT → complex magnitude
  8. OLED rendering (skipped if less than 50 ms since last draw)
     u8g2.clearBuffer()
     showS_meter()
     showScope()
     displayWaterfall()
     showGraphics()
     displayCWText()
     u8g2.sendBuffer()
```

#### I/Q Data and Complex FFT

An ordinary FFT operates on real-valued data, but this transceiver provides two channels: I and Q.  
By using `vReal[] = I` and `vImag[] = Q` as the real and imaginary parts of a complex FFT:

- Positive frequency bins → I + jQ components (USB equivalent)
- Negative frequency bins → I − jQ components (LSB equivalent)

A single FFT thus yields the spectrum in both the +15 kHz and −15 kHz directions simultaneously — which is why the bandscope extends to both sides.

**Role of the Hamming window:**  
FFT assumes the signal repeats periodically. If the samples at the buffer boundaries are discontinuous, "spectral leakage" spreads signal energy into adjacent frequency bins.  
Applying a Hamming window brings the buffer edges toward zero, suppressing leakage.

#### Bandscope Drawing Method (showScope function)

```
BIN_HZ = sampleRate / SAMPLES = 40000 / 256 = 156.25 Hz/bin

Right half (positive frequencies):
  for xi = 0 to 63:
    exactBin = xi × binsPerPixel + offsetBins (CW_TONE offset)
    bin = round(exactBin)
    d = (barLength(vReal[bin]) + barLength(vImag[bin])) / 2
    peakR[xi] = max(peakR[xi], d)  (peak hold)
    x = 63 + xi
    Also written to the waterfall history buffer

Left half (negative frequencies):
  for xi = 1 to 63:
    exactBin = −(xi × binsPerPixel) + offsetBins
    Negative frequencies are stored at vReal[SAMPLES − |bin|]
    x = 63 − xi
```

**Meaning of the CW_TONE offset (offsetBins):**  
Because the VFO is offset from the signal by 700 Hz, the 700 Hz value is converted to FFT bins and added as an offset so that the center of the display (x = 63) corresponds to the actual VFO frequency (the receive center frequency).

---

### 3-5. CW Decoder Flow (handleCWDecoder function)

```
handleCWDecoder() is called every loop (~50 ms) by Core0

During TX → flush cwEvRd = cwEvWr (clear buffer) and return
(Old events are discarded so they are not processed on RX resumption)

═══ Event processing (while loop consumes all buffered events) ═══

type == 1 (mark ended = key-up confirmed):
  dur = ev.durMs (duration of the preceding key-down period)
  if dur ≥ dotEst × 0.3 and morseLen < 8:
    dur < dotEst × 2.0 → dot '.'
      dotEst = 0.85 × dotEst + 0.15 × dur
    dur ≥ dotEst × 2.0 → dash '−'
      dotEst = 0.85 × dotEst + 0.15 × (dur/3)
  dotEst = constrain(dotEst, 30, 240)  ← 5–40 WPM range
  keyUpMs = now
  keyIsUp = true,  charDecoded = false,  wordAdded = false

type == 0 (space ended = key-down confirmed):
  keyIsUp = false

═══ Character/word confirmation by silence timeout ═══

while keyIsUp == true:
  silMs = now − keyUpMs (elapsed time since key-up)

  if !charDecoded and morseLen > 0
     and silMs ≥ dotEst × 2.5:
       decode morse[] via decodeMorse() → addCWDecodedChar()
       morseLen = 0, charDecoded = true

  if charDecoded and !wordAdded
     and silMs ≥ dotEst × 6.0:
       addCWDecodedChar(' ')
       wordAdded = true
```

**Self-learning dotEst:**  
Starting from 60 ms (20 WPM), it tracks the actual mark lengths and automatically estimates WPM.  
Because only 15% of the new value is blended in each time, it takes a few characters to converge after an abrupt speed change.

| dotEst value | WPM | Dot duration |
|---|---|---|
| 240 ms | 5 WPM | 240 ms |
| 120 ms | 10 WPM | 120 ms |
| 60 ms | 20 WPM | 60 ms |
| 40 ms | 30 WPM | 40 ms |
| 30 ms | 40 WPM | 30 ms |

**Meaning of timing multipliers:**

| Decision | Multiplier | Purpose |
|---|---|---|
| Dot / dash boundary | dotEst × 2.0 | Shorter → dot; equal or longer → dash |
| Minimum mark length | dotEst × 0.3 | Shorter than this → treated as noise and ignored |
| Inter-character space | dotEst × 2.5 | This silence confirms the current character |
| Inter-word space | dotEst × 6.0 | This silence inserts a space character |

---

### 3-6. Display Rendering Detail (showScope)

**Spectrum amplitude calculation in barLength:**

```cpp
float fy = SCOPE_SENSITIVITY × (log10(d) + SCOPE_OFFSET);
int y = constrain((int)fy, 0, 20);
```

Displaying on a logarithmic scale (dB) allows both weak and strong signals to coexist on the same screen.  
`d` is the complex magnitude from the FFT (average of `barLength` on the real and imaginary parts).

**Peak hold:**

```cpp
if ((uint8_t)d >= pk) pk = (uint8_t)d;          // update: current ≥ peak
else if (doDecay && pk > 0) pk--;                // decay: 1 px per PEAK_DECAY_FRAMES frames
peakR[xi] = pk;
```

The peak is updated instantly and decays by 1 px every `PEAK_DECAY_FRAMES` frames.  
Drawn as a polyline, it visually tracks the highest point of the received signal.

---

### 3-7. Q Amplitude Correction (I/Q Image Reduction)

In a direct-conversion receiver, differences in ADC characteristics between GPIO26 and GPIO27 cause the I and Q amplitudes to be unequal, producing a "mirror image" in the FFT (a ghost signal appearing on the opposite side of the VFO).

```cpp
// Track average I² and Q² with a very slow moving average (τ = 2000 samples ≈ 0.4 s)
avgI2 = avgI2 * 0.9995f + si*si * 0.0005f;
avgQ2 = avgQ2 * 0.9995f + sq*sq * 0.0005f;

// Compute correction factor from RMS ratio of I and Q (clamped to 0.8–1.25)
float qGain = (avgQ2 > 0.0001f)
              ? constrain(sqrtf(avgI2 / avgQ2), 0.8f, 1.25f) : 1.0f;

vImag[i] = sharedBufferQ[i] * qGain;
```

`qGain > 1.0` → Q was smaller than I; scale up  
`qGain < 1.0` → Q was larger than I; scale down

**Why phase correction is not applied:**  
The Gram-Schmidt method can simultaneously correct both amplitude and phase imbalance and theoretically achieve perfect cancellation. However, if initialized incorrectly, the correction coefficients can explode by tens of thousands of times and break the display.  
Limiting to amplitude-only correction ensures safety. In practice, amplitude error is the dominant cause of mirror images, and the contribution of phase error is small.

---

## 4. How to Operate

### 4-1. Power On

At startup, a splash screen ("7MHz CW TRX v5.0" etc.) is shown for about 1 second, then the transceiver resumes the last saved state (frequency, STEP, WPM, key mode, and volume) from EEPROM.

---

### 4-2. Rotary Encoder (Frequency Tuning)

| Action | Effect |
|---|---|
| Clockwise (CW) | Frequency up by one STEP |
| Counter-clockwise (CCW) | Frequency down by one STEP |

- Range limit: 7.000 MHz – 7.200 MHz (`LOW_FREQ` / `HI_FREQ`)
- Frequency is saved to EEPROM on every change (mind the flash write cycle limit)
- Mute is applied for ~500 loops (~12.5 ms) on each frequency change to suppress pop noise

---

### 4-3. Button 1 (GPIO2 / STEP_BUTTON)

#### Short press: Frequency step cycle

| Step size | stepMode | Display |
|---|---|---|
| 1000 Hz | 0 | 1000 |
| 100 Hz | 1 | 100 |
| 10 Hz | 2 | 10 |

Three steps cycle in order: 1000 → 100 → 10 → 1000.

#### Long press (1 second or more): WPM setting mode

1. The display switches to a "WPM: 20" style settings screen.
2. Rotate the encoder to change WPM (range: 5–40 WPM).
3. **Short press** saves and exits (writes EEPROM + recalculates timing via `initKeyer()`).
4. **Long press (800 ms)** cancels (discards the change, keeps current value).

---

### 4-4. Button 2 (GPIO3 / KEY_MODE_BUTTON)

#### Short press: Key mode toggle

| Mode | straightKeyMode | Behavior |
|---|---|---|
| Iambic keyer | false | GPIO6 (dot) / GPIO7 (dash) used as paddle |
| Straight key | true | GPIO7 used as a straight key (LOW = TX, HIGH = RX) |

When the mode changes, "KEY MODE / STRA (or KEY)" is shown on the display for about 600 ms.

#### Long press (800 ms or more): Volume adjustment mode

1. A volume bar display appears.
2. Rotate the encoder to adjust volume (×1.0 – ×3.0 in 0.1 steps).
3. **Short press** saves and exits (writes EEPROM).

| Internal value | volumeMultiplier | Meaning |
|---|---|---|
| 10 | 1.0 | Standard volume (default) |
| 20 | 2.0 | Double |
| 30 | 3.0 | Triple (maximum) |

---

### 4-5. CW Transmission (Iambic Keyer Mode)

```
State 0 (Idle):
  Dot paddle pressed  → State 1 (sending dot)
  Dash paddle pressed → State 1 (sending dash)

State 1 (Transmitting):
  Si5351 CLK2 ON (TX carrier)
  Core1 outputs 700 Hz monitor tone via PWM
  After dotDuration or dashDuration elapses → stopTransmit() → State 2

State 2 (Inter-element space):
  After elementSpace (= dotDuration) elapses → State 0

Inter-word space (wordSpace = dotDuration × 7) forms
  naturally by holding the paddle idle during State 0
```

**Relationship between WPM and timing (`calculateTiming` function):**

| WPM | dotDuration | dashDuration | elementSpace | charSpace | wordSpace |
|---|---|---|---|---|---|
| 5 WPM | 240 ms | 720 ms | 240 ms | 720 ms | 1680 ms |
| 20 WPM | 60 ms | 180 ms | 60 ms | 180 ms | 420 ms |
| 40 WPM | 30 ms | 90 ms | 30 ms | 90 ms | 210 ms |

Formula: `dotDuration = 1200 / wpm` (PARIS standard)

---

### 4-6. Automatic CW Reception Decoding

- When a CW signal near 700 Hz is received, decoding begins automatically.
- Decoded text is shown in the bottom strip of the display (y = 54–63) using a 5×8 font.
- The newest character is anchored to the right edge; older characters scroll off to the left.
- WPM (speed) is estimated automatically — no manual adjustment needed.
- Decoding is suspended during TX and the event buffer is flushed.

---

### 4-7. EEPROM Storage Contents

| Item | Address | Data type | Saved when |
|---|---|---|---|
| VFO frequency (Hz) | 0–3 | unsigned long (4 bytes) | On every frequency change |
| Frequency step (Hz) | 4–7 | int (4 bytes) | On STEP toggle |
| WPM | 8–11 | int (4 bytes) | On WPM save operation |
| Key mode | 12–15 | int (4 bytes, 0 = iambic, 1 = straight) | On mode toggle |
| Volume (integer × 10) | 16–19 | int (4 bytes) | On volume save operation |

---

## 5. Code Style and Naming Conventions

### 5-1. `#define` vs. `const`

```cpp
// Fixed values unlikely to change → #define (preprocessor substitution, no type)
#define SAMPLES        256
#define CW_TONE        700
#define CW_EVBUF_SIZE  16

// Constants needing type safety → const variables
const long LOW_FREQ = 7000000;
const long HI_FREQ  = 7200000;
```

### 5-2. Meaning and Usage of `volatile`

`volatile` tells the compiler "do not optimize away accesses to this variable."  
In a dual-core environment, when one core modifies a variable, the other core must not read a stale cached copy — hence `volatile`.

```cpp
volatile float cwEnvelope;       // Core1 writes, Core0 reads → volatile
volatile bool sharedBufferReady; // Core1 sets true, Core0 resets to false
volatile uint8_t cwEvWr;         // Core1's write index
volatile uint8_t cwEvRd;         // Core0's write index (Core1 also reads it)
volatile int muteCounter;        // Core0 writes, Core1 reads → volatile
volatile float volumeMultiplier; // same

float agcGain;                   // only Core1 reads/writes → volatile not needed
bool transmitting;               // Core0 writes, Core1 reads (bool is atomic in practice)
```

### 5-3. Accessing Fields of a `volatile` Struct

```cpp
// BAD: copying a volatile array element triggers a compile error [-fpermissive]
CWEvent ev = cwEvBuf[cwEvRd];   // error

// GOOD: copy fields individually
CWEvent ev;
ev.type  = cwEvBuf[cwEvRd].type;   // each field is a scalar — copy is valid
ev.durMs = cwEvBuf[cwEvRd].durMs;
```

The struct's copy constructor cannot accept a `volatile` source, so fields must be copied one by one.

### 5-4. `static` Local Variables

A local variable declared `static` behaves like a "global variable accessible only within the function."  
It is initialized only on the first call; its value is retained across all subsequent calls.

```cpp
void loop1() {
    static float toneAngle = 0.0f;   // TX 700 Hz phase accumulator (persists across calls)
    static float cwEnvLP   = 0.0f;   // CW envelope low-pass value
    static bool  cwSt      = false;  // confirmed CW detection state
    static uint32_t cwCnt  = 0;      // current-state sample count
}
```

Using `static` locals keeps DSP state variables scoped to the function that owns them, avoiding global namespace pollution.

### 5-5. Function Naming Conventions

| Pattern | Example | Meaning |
|---|---|---|
| `handle〇〇()` | `handleKeyer()` | Polled every loop iteration |
| `show〇〇()` | `showScope()` | OLED drawing (called between clearBuffer and sendBuffer) |
| `display〇〇()` | `displayCWText()` | Same as above |
| `change〇〇()` | `changeVolume()` | Interactive settings-change UI mode |
| `〇〇_Set()` | `Freq_Set()` | Apply settings to hardware |
| `save〇〇ToEEPROM()` | `saveWPMToEEPROM()` | Write to EEPROM |
| `read〇〇FromEEPROM()` | `readFrequencyFromEEPROM()` | Read from EEPROM |
| `start/stop〇〇()` | `startTransmit()` | State-transition function |
| `init〇〇()` | `initKeyer()` | Initialization |
| `apply〇〇()` | `applyAGC()` | Apply a transformation and return the result |

### 5-6. Section Comments and Change Markers

```cpp
// ==============================================================================
// [N] Section Name (major category)
// ==============================================================================

// --- Sub-category description ---
```

Key modifications and points of interest are marked with `★`:

```cpp
// ★ TX start timestamp synchronization
// ★ Change 1: Expanded range
static float lastRxPwm = 2047.5f;  // ★ Added: for TX start crossfade
```

---

## 6. Parameter Reference and Effects of Changes

### 6-1. Frequency / VFO

```cpp
const long LOW_FREQ  = 7000000;   // Lower encoder limit (Hz)
const long HI_FREQ   = 7200000;   // Upper encoder limit (Hz)
#define CW_AUDIO_OFFSET 70000ULL  // BFO offset (Hz)
#define CW_TONE         700       // CW monitor tone (Hz)
```

| Parameter | Effect of changing |
|---|---|
| `LOW_FREQ` / `HI_FREQ` | Changes the tunable frequency range. Modify when repurposing for another band (e.g., 30 m: 10,100,000–10,150,000). |
| `CW_AUDIO_OFFSET` | Changes the IF center frequency of the Si5351. The IIR filter coefficients must also be recalculated when this is changed (the design is fixed at 700 Hz). |
| `CW_TONE` | Changes the pitch of the monitor tone (TX sidetone). Because the receive IIR filter is also centered at this frequency, the filter coefficients must be redesigned to match any new value. |

---

### 6-2. Bandscope Display

```cpp
#define SCOPE_SPAN_HZ     15000.0f
#define SCOPE_SENSITIVITY 15.0f
#define SCOPE_OFFSET       3.0f
```

**SCOPE_SPAN_HZ (display bandwidth):**

| Value | Display range | Recommended use |
|---|---|---|
| 10000 | ±10 kHz | Pico (200 MHz) |
| 15000 | ±15 kHz | Pico2 (150 MHz) — default |
| 20000 | ±20 kHz | Wide-band monitoring (lower resolution) |

Increasing the value raises the Hz-per-pixel ratio, making it harder to separate closely spaced signals.  
Decreasing it shows finer detail but reduces the visible frequency span.

**SCOPE_SENSITIVITY (scope gain):**

`barLength = SCOPE_SENSITIVITY × (log10(d) + SCOPE_OFFSET)`

| Value | Effect |
|---|---|
| 10 | Low waveform (only strong signals visible) |
| 15 | Standard — default |
| 20 | Taller waveform (noise becomes visible) |
| 30 | Weak signals appear, but strong signals saturate |

Increasing this makes the entire waveform taller, revealing weaker signals.  
Setting it too high causes strong signals to hit the ceiling constantly, making relative comparison impossible.

**SCOPE_OFFSET (noise floor offset):**

| Value | Effect |
|---|---|
| 0 | Very weak signals are invisible (noise disappears) |
| 3 | Standard — noise floor slightly visible (default) |
| 5 | Noise floor is prominent, hard to distinguish from signals |

This is added after the `log10(d)` logarithmic conversion, raising the floor for very small values.  
A small value gives a flat baseline during silence; a large value keeps noise permanently visible.

---

### 6-3. Waterfall Display

```cpp
#define WATERFALL_HEIGHT  21
#define THRESHOLD          2
const uint8_t PEAK_DECAY_FRAMES = 1;
```

**WATERFALL_HEIGHT (number of waterfall rows):**

| Value | Past frames shown | Time per frame (≈50 ms) |
|---|---|---|
| 10 | 10 frames | Total ≈ 0.5 s of history |
| 21 | 21 frames | Total ≈ 1.05 s of history — default |
| ~35 (max) | ↑ | Limited by screen space |

Increasing the value extends the signal history.  
However, the display uses y = 27–47 (21 pixels), so values above 21 will extend beyond the screen area (the corresponding drawing coordinates must also be changed).

**THRESHOLD (display threshold):**

| Value | Effect |
|---|---|
| 0 | All FFT bins shown as dots (noise is visible) |
| 2 | Standard — default |
| 5 | Only strong signals appear in the waterfall |

Only pixels where the `barLength` return value (0–20) exceeds `THRESHOLD` are drawn.  
In noisy environments, increasing this value makes the waterfall cleaner.

**PEAK_DECAY_FRAMES (peak decay speed):**

```cpp
if (doDecay && pk > 0) pk--;
// doDecay becomes true once every PEAK_DECAY_FRAMES frames
```

| Value | Peak decay speed |
|---|---|
| 1 | 1 px per frame (fastest; peak line disappears quickly) — default |
| 3 | 1 px every 3 frames (slow) |
| 5 | 1 px every 5 frames (very slow; long peak hold) |

A larger value keeps the peak line visible longer, making it easier to identify peak signal positions.  
A smaller value reflects the current signal strength in near real-time.

---

### 6-4. CW Reception and Detection

```cpp
#define CW_DETECT_THRESHOLD  1.5f
#define Q_DELAY_SAMPLES       14
const uint8_t DB_TH = 80;        // inside loop1
static float cwNoise = 0.05f;    // inside loop1
```

**CW_DETECT_THRESHOLD (signal detection threshold):**

When the SNR ratio (`cwEnvelope`) exceeds this value, the key is considered "down."

| Value | Effect |
|---|---|
| 1.2 | High sensitivity — detects weak signals but more noise false-triggers |
| 1.5 | Standard — default |
| 2.0 | Low sensitivity — only strong signals detected, fewer false triggers |
| 3.0 | Strong signals only — most signals will be missed |

Raise the value in noisy environments; lower it to decode weak signals.  
Setting it too low causes noise to be misread as CW.

**Q_DELAY_SAMPLES (Q-channel delay in samples):**

At 700 Hz sampled at 40 kHz, 90° = 40000 / 700 / 4 = 14.28 samples → 14 is the optimal integer value.

| Value | Effect |
|---|---|
| 14 | Optimal 90° delay at 700 Hz — default |
| Changed | The optimal frequency shifts. Recalculate when `CW_TONE` changes. |

Formula when recalculating: `Q_DELAY_SAMPLES = sampleRate / CW_TONE / 4`

**DB_TH (debounce threshold in samples):**

Number of consecutive samples required to confirm a CW state transition.

| Value (samples) | Corresponding time (@40 kHz) | Effect |
|---|---|---|
| 40 | 1 ms | Fast debounce (captures short symbols reliably) |
| 80 | 2 ms | Standard — default |
| 160 | 4 ms | Slow debounce (more noise-immune; may miss symbols at high WPM) |

A dot at 40 WPM is 30 ms, so even 4 ms debounce is safe.  
However, in environments with sub-1 ms noise pulses, 80–160 is recommended.

**cwNoise initial value (0.05f):**

The estimated noise floor immediately after startup. Setting it slightly high prevents false detections before the noise tracker converges.  
Convergence time to actual noise floor: `1 / 0.0005 = 2000 samples ≈ 50 ms`.  
In quiet environments the value can be lowered, but in most cases it converges automatically within 50 ms, so no change is needed.

---

### 6-5. AGC (Automatic Gain Control)

```cpp
const float targetAmplitude = 0.5;
const float maxGain         = 20.0;
const float minGain         =  0.1;
const float attackRate      = 0.01;
const float decayRate       = 0.001;
```

**targetAmplitude (AGC target amplitude):**

The AGC adjusts its gain to stabilize the signal amplitude at this value.

| Value | Effect |
|---|---|
| 0.3 | Modest volume (large headroom) |
| 0.5 | Standard — default |
| 0.8 | Loud output (higher risk of clipping) |

**maxGain (maximum gain multiplier):**

| Value | Max amplification (dB) | Effect |
|---|---|---|
| 5.0 | +14 dB | Limited amplification of weak signals (suitable for high-noise environments) |
| 20.0 | +26 dB | Default — amplifies down to very weak signals |
| 50.0 | +34 dB | Hears very weak signals, but noise is also greatly amplified |

Increasing `maxGain` improves weak-signal sensitivity, but noise is amplified equally — the SNR does not improve; you just hear louder noise in the absence of a signal.

**attackRate / decayRate (AGC response speed):**

| Variable | Effect of increasing | Effect of decreasing |
|---|---|---|
| `attackRate` | Gain increases faster (quicker response to weak signals); prone to gain hunting | Gain increases slowly (sluggish tracking) |
| `decayRate` | Gain recovers faster after a strong signal (quickly ready for the next weak signal) | Gain recovers slowly (better protection against clipping) |

---

### 6-6. TX Envelope and Crossfade

```cpp
const float envelopeStep = 0.002f;   // inside loop1
const float fadeWindow   = 0.05f;    // inside loop1
```

**envelopeStep (envelope ramp rate):**

Controls the speed of the fade-in/fade-out at TX start/stop.  
`txEnvelope` ranges from 0.0 to 1.0 and changes by `envelopeStep` per loop.

| Value | Fade time (0→1) | Effect |
|---|---|---|
| 0.002 | ~500 loops ≈ 12.5 ms | Default — no click noise |
| 0.01 | ~250 loops ≈ 6.25 ms | Faster rise/fall |
| 0.001 | ~2500 loops ≈ 62.5 ms | Very smooth (soft CW key shape) |

Smaller values make ON/OFF smoother but dull the keying waveform rise time.  
Larger values give crisper keying but may introduce pop noise.

**fadeWindow (TX start crossfade width):**

The interval over which the last RX PWM output is blended into the CW tone at TX start.  
While `txEnvelope < fadeWindow`, `blend = txEnvelope / fadeWindow` linearly interpolates.

| Value | Crossfade time (@envelopeStep = 0.002) | Effect |
|---|---|---|
| 0 | 0 ms (instant switch) | Pop noise likely |
| 0.05 | 25 loops ≈ 0.625 ms | Default |
| 0.1 | 50 loops ≈ 1.25 ms | Very smooth |

---

### 6-7. CW Decoder Timing

```cpp
static float dotEst = 60.0f;   // inside handleCWDecoder
```

**dotEst initial value (60.0 ms = 20 WPM):**

The WPM estimate immediately after startup. It converges automatically to the received signal speed, but if the initial value is far from the actual speed, the first few characters may be decoded incorrectly.

| Initial value | WPM equivalent | Recommended when |
|---|---|---|
| 120 ms | 10 WPM | Mostly slow CW |
| 60 ms | 20 WPM | General use — default |
| 40 ms | 30 WPM | Mostly fast CW |

**Adjusting the timing multipliers:**

The magic-number multipliers written directly in `handleCWDecoder` can be tuned to change the decoding criteria.

| Code location | Current value | Meaning and adjustment |
|---|---|---|
| `dur < dotEst * 2.0f` | ×2.0 | Dot/dash boundary. Adjustable from 1.5 to 2.5. Smaller → dash is decided earlier |
| `dur >= dotEst * 0.3f` | ×0.3 | Minimum mark length (noise floor). Larger → more likely to miss dots |
| `silMs >= dotEst * 2.5f` | ×2.5 | Inter-character space detection. Larger → character confirmation is delayed |
| `silMs >= dotEst * 6.0f` | ×6.0 | Inter-word space detection. Larger → word spaces are inserted less often |

---

### 6-8. I/Q Amplitude Correction

```cpp
static float avgI2 = 0.01f, avgQ2 = 0.01f;   // inside loop()
float qGain = constrain(sqrtf(avgI2/avgQ2), 0.8f, 1.25f);
```

**qGain clamped range (0.8–1.25):**

| Limit | Current value | Effect of changing |
|---|---|---|
| Upper limit 1.25 | → expand to 1.5 | Can correct larger amplitude differences (higher risk of instability) |
| Upper limit 1.25 | → narrow to 1.1 | Conservative correction (safer) |
| Tracking constant 0.9995 | → change to 0.999 | Faster convergence (but may affect audio quality) |

Removing the clamping too aggressively can make the coefficient numerically unstable and break the display.  
±25% (0.8–1.25) is the practical safe range.

---

### 6-9. OLED Display Selection

```cpp
//U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0);                // 0.91-inch 128×32
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);     // 0.96-inch (default)
//U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);    // 1.3-inch SH1106
```

Uncomment only the line that matches the driver IC of your OLED module.  
Using the 128×32 version requires a full redesign of all screen layout y-coordinates.

---

*Document version: V5.0 / Date: 2026-05-29*
