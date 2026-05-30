/*
  pico 40M monoband CW Transceiver V5.0 May 25, 2026: Increased sensitivity of the CW filter by ±100 Hz
  OLED 128x64 VFO + RX_Bandscope + Keyer + Implementation of CW Decoding Functionality
  Adjust the volume range + Oversampling 2x → 4x AGC ← +14 dB sensitivity improvement
  Add a waterfall chart
  pico  click 200MHz 607,609lines 10000 Bandwidth range（±10kHz）
  pico2 click 150MHz 607,609lines 15000 Bandwidth range（±15kHz）
  Button1 Frequency Step Switch / WPM Setting (Long Press)
  Button2 Keyer Mode Switch / Volume Control (Long Press)

  --Libraries used--
  Arduino.h
  Rotary.h       : https://github.com/brianlow/Rotary
  U8g2lib.h      : https://github.com/olikraus/U8g2_Arduino
  Wire.h
  arduinoFFT.h   : v2.0.2
  si5351.h       : https://github.com/etherkit/Si5351Arduino
  EEPROM.h
*/

#include <Arduino.h>
#include <Rotary.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <arduinoFFT.h>
#include <si5351.h>
#include <EEPROM.h>

// ==============================================================================
// [1] Pin Definitions and Basic Settings (Hardware and Constants)
// ==============================================================================

// --- Pin layout ---
#define I_IN              26  // I-channel analog signal input
#define Q_IN              27  // Q-channel analog signal input
#define inputPinI         26  // (Alias)
#define inputPinQ         27  // (Alias)
#define speakerPin        16  // Audio PWM output (GPIO 16 / 14)
#define PIN_IN1           0   // Rotary encoder channel A
#define PIN_IN2           1   // Rotary encoder channel B
#define STEP_BUTTON       2   // Frequency step switch / WPM setting (long press)
#define KEY_MODE_BUTTON   3   // Keyer mode switch / Volume control (long press)
#define PADDLE_DOT        6   // Paddle dot input
#define PADDLE_DASH       7   // Paddle dash input / navigation key input
#define RX_SW             15  // TX/RX switch (High during TX)
#define LED_INDICATOR     25  // Processing indicator LED

// --- SDR・Signal Processing Settings ---
#define SAMPLES           256     // Number of samples for FFT and shared buffer
#define sampleRate        40000   // Effective sampling frequency (Hz)
#define pwmFrequency      44100   // PWM frequency (Hz)
#define CW_AUDIO_OFFSET   70000ULL// BFO offset frequency
#define CW_TONE           700     // CW monitor tone frequency (Hz)

// --- Default Settings ---
const long LOW_FREQ     = 7000000;
const long HI_FREQ      = 7200000;
#define DEFAULT_WPM       20

// --- EEPROM address ---
const int EEPROM_ADDRESS_FREQ    = 0;
const int EEPROM_ADDRESS_STEP    = 4;
const int EEPROM_ADDRESS_WPM     = 8;
const int EEPROM_ADDRESS_KEYMODE = 12;
const int EEPROM_ADDRESS_VOL     = 16;


// ==============================================================================
// [2] Global Variables
// ==============================================================================

// --- Frequency / VFO ---
unsigned long FREQ = 7000000;
unsigned long FREQ_OLD = FREQ;
unsigned long long FREQ_ULL = 700000000ULL;
unsigned long long pll_freq = 86400000000ULL;
int STEP = 1000;
int stepMode = 0; // 0:1kHz, 1:100Hz, 2:10Hz

// ★ Customizable: Bandscope settings
#define SCOPE_SPAN_HZ     15000.0f // One-side display span of the scope (e.g., ±15 kHz for 15,000)
#define SCOPE_SENSITIVITY 15.0f    // Amplitude gain of the scope waveform (larger = taller waveform)
#define SCOPE_OFFSET      3.0f     // Raises the scope noise floor (makes otherwise invisible micro-noise visible)

// --- Status Management ---
bool transmitting = false;
volatile int muteCounter = 0;     // Mute counter for frequency changes

// --- Audio and DSP Processing ---
volatile float volumeMultiplier = 1.0;
float agcGain = 1.0;
float dcOffsetI = 0.0;
float dcOffsetQ = 0.0;
// tonePhase/tonePhaseIncrement moved to static float inside loop1() (eliminates 32-bit integer overflow)

// ===== IIR filter parameters (fs=40kHz, fc=700Hz, Q≈7 (±50Hz bandwidth), 3-stage 6th-order cascade: QRM-hardened) =====
// ★ BPF preset reference table (change both loop count and coefficients together)
//  Stages  BW         b0        a1_code   a2_code
//  2-stage ±100 Hz   0.024122  1.939356  0.951756  ← original (standard)
//  2-stage ±50 Hz    0.012032  1.964050  0.975910
//  3-stage ±50 Hz    0.015134  1.957930  0.969710  ← current (QRM rejection)
//  3-stage ±30 Hz    0.009137  1.970800  0.981730  ← ultra-narrow (extreme QRM)
// Note: narrower BW demands more precise VFO tuning (±50 Hz is fine with typical Si5351)
float iir_x1[3]={0}, iir_x2[3]={0}, iir_y1[3]={0}, iir_y2[3]={0};

// ===== 90-degree delay buffer for Q signal (Hilbert transform approximation) =====
// Optimal delay for 90° at 700 Hz with 40 kHz sampling is 14 samples
#define Q_DELAY_SAMPLES 14
float qDelayBuffer[Q_DELAY_SAMPLES] = {0};
int qDelayIndex = 0;

// --- Inter-core Data Sharing (Core1 → Core0) ---
float sharedBufferI[SAMPLES];
float sharedBufferQ[SAMPLES];
volatile bool sharedBufferReady = false;
int sharedIndex = 0;

// --- CW Decoding Global Variables ---
volatile float cwEnvelope = 0.0f;     // Core1→Core0: CW detection value (SNR ratio)
#define CW_DETECT_THRESHOLD 2.0f      // SNR ratio threshold: signal detected when ratio exceeds this value
#define CW_DECODED_MAX      36        // Maximum characters in the decoded text buffer
char cwDecodedBuf[CW_DECODED_MAX + 1] = "";
int  cwDecodedLen = 0;

// ★ CW Event Buffer (Core1→Core0, ring buffer)
// Passes precisely measured mark/space durations from Core1 to Core0
struct CWEvent {
  uint8_t  type;    // 0=space ended (key-down confirmed), 1=mark ended (key-up confirmed)
  uint16_t durMs;   // Duration that state was held [ms]
};
#define CW_EVBUF_SIZE 16
volatile CWEvent cwEvBuf[CW_EVBUF_SIZE];
volatile uint8_t cwEvWr = 0;   // Write index (updated by Core1)
volatile uint8_t cwEvRd = 0;   // Read index (updated by Core0)

// --- Display and FFT ---
double vReal[SAMPLES];
double vImag[SAMPLES];
static uint8_t peakR[64];
static uint8_t peakL[64];
static uint8_t peakDecayDiv = 0;
const uint8_t PEAK_DECAY_FRAMES = 1;  // Peak decay speed (larger value = slower decay)
const uint8_t DC_BLANK_BINS = 0;      // Number of bins around center to suppress from display
const int VFO_MARKER_X = 63;          // X coordinate of the VFO center marker
const int PEAK_Y_OFFSET = 10;
const int MARKER_TOP_MARGIN = 22;

// --- Key Management ---
int wpm = DEFAULT_WPM;
bool straightKeyMode = false;
bool sending = false;
bool sendingDot = false;
bool sendingDash = false;
unsigned long dotDuration, dashDuration;
unsigned long elementSpace, charSpace, wordSpace;
bool lastPaddleDot = false;
bool lastPaddleDash = false;

// --- Object Instances ---
Rotary r = Rotary(PIN_IN1, PIN_IN2);

// --- Waterfall Variables ---
#define WATERFALL_HEIGHT 21  // 24→21: Raised waterfall bottom by 6 px to secure CW text display area
#define THRESHOLD 2
uint8_t waterfallHistory[WATERFALL_HEIGHT][128] = {0};
int waterfallIndex = 0;

Si5351 si5351;

//U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0); // Standard OLED (0.91 inch)
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE); // 0.96 inch
//U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0,U8X8_PIN_NONE); // 1.3 inch

ArduinoFFT<double> FFT;

// ==============================================================================
// [3] Hardware and EEPROM Control
// ==============================================================================

// --- Basic EEPROM Operations ---
void saveToEEPROM(int address, int data) {
  for (int i = 0; i < sizeof(data); i++) {
    EEPROM.write(address + i, (data >> (8 * i)) & 0xFF);
  }
  EEPROM.commit();
}

int readFromEEPROM(int address) {
  int data = 0;
  for (int i = 0; i < sizeof(data); i++) {
    data |= EEPROM.read(address + i) << (8 * i);
  }
  return data;
}

// --- Save / Load Custom Settings ---
void saveFrequencyToEEPROM(unsigned long freq) { saveToEEPROM(EEPROM_ADDRESS_FREQ, freq); }
unsigned long readFrequencyFromEEPROM()        { return readFromEEPROM(EEPROM_ADDRESS_FREQ); }

void saveStepToEEPROM(int step)                { saveToEEPROM(EEPROM_ADDRESS_STEP, step); }
int readStepFromEEPROM()                       { return readFromEEPROM(EEPROM_ADDRESS_STEP); }

void saveWPMToEEPROM(int wpmValue)             { saveToEEPROM(EEPROM_ADDRESS_WPM, wpmValue); }
int readWPMFromEEPROM() {
  int val = readFromEEPROM(EEPROM_ADDRESS_WPM);
  return (val >= 5 && val <= 40) ? val : DEFAULT_WPM;
}

void saveKeyModeToEEPROM(bool mode)            { saveToEEPROM(EEPROM_ADDRESS_KEYMODE, mode ? 1 : 0); }
bool readKeyModeFromEEPROM()                   { return readFromEEPROM(EEPROM_ADDRESS_KEYMODE) == 1; }

void saveVolToEEPROM(int volValue)             { saveToEEPROM(EEPROM_ADDRESS_VOL, volValue); }
int readVolFromEEPROM() {
  int val = readFromEEPROM(EEPROM_ADDRESS_VOL);
  // ★ Changed upper limit to 30; returns default value of 10 (×1.0) if out of range
  return (val >= 10 && val <= 30) ? val : 10;
}

// --- Si5351 Frequency Control ---
void Freq_Set() {
  si5351.set_freq_manual(FREQ_ULL - CW_AUDIO_OFFSET, pll_freq, SI5351_CLK0);
  si5351.set_freq_manual(FREQ_ULL - CW_AUDIO_OFFSET, pll_freq, SI5351_CLK1);
  int phase = pll_freq / (FREQ_ULL - CW_AUDIO_OFFSET) + 0.5; // Formula Correction
  si5351.set_phase(SI5351_CLK0, 0);
  si5351.set_phase(SI5351_CLK1, phase);
  si5351.pll_reset(SI5351_PLLA);
}

void startTransmit() {
  if (!transmitting) {
    transmitting = true;
    si5351.output_enable(SI5351_CLK0, 0); // RX LO OFF
    si5351.output_enable(SI5351_CLK1, 0);
    digitalWrite(RX_SW, HIGH);            // TX Switch ON
    si5351.output_enable(SI5351_CLK2, 1); // TX Carrier ON
    si5351.set_freq_manual(FREQ_ULL, pll_freq, SI5351_CLK2);
  }
}

void stopTransmit() {
  if (transmitting) {
    transmitting = false;
    si5351.output_enable(SI5351_CLK2, 0); // TX Carrier OFF
    si5351.output_enable(SI5351_CLK0, 1); // RX LO ON
    si5351.output_enable(SI5351_CLK1, 1);
    digitalWrite(RX_SW, LOW);             // RX Switch ON
  }
}

// ==============================================================================
// [4] DSP and Signal Processing
// ==============================================================================

// CW demodulation (ultra-fast, high-sensitivity version using cascaded 2nd-order IIR filters)
float cwDemodulate(float iSignal, float qSignal, unsigned long currentFreq) {
  // 1. Canceling the reverse sideband using the Hilbert transform approximation (90-degree delay)
  float qDelayed = qDelayBuffer[qDelayIndex];
  qDelayBuffer[qDelayIndex] = qSignal;
  qDelayIndex++;
  if (qDelayIndex >= Q_DELAY_SAMPLES) qDelayIndex = 0;

  float audioSignal = iSignal - qDelayed;

  // 2. IIR 6th-order BPF (3-stage cascade, ±50Hz) — strong QRM rejection
  // To revert to ±100 Hz standard: change loop to 2, coefficients to 0.024122/1.939356/0.951756
  float out = audioSignal;
  for (int i = 0; i < 3; i++) {
    float y = 0.015134f * out - 0.015134f * iir_x2[i]
              - (-1.957930f) * iir_y1[i] - (0.969710f) * iir_y2[i];
    iir_x2[i] = iir_x1[i];
    iir_x1[i] = out;
    iir_y2[i] = iir_y1[i];
    iir_y1[i] = y;
    out = y;
  }
  return out;
}

float applyAGC(float input) {
  const float targetAmplitude = 0.5;
  const float maxGain = 30.0;   // 10.0→20.0: Expanded weak-signal amplification ceiling by +14 dB
  const float minGain = 0.1;
  const float attackRate = 0.01;
  const float decayRate = 0.001; // Fast gain reduction on strong signals to prevent clipping

  float error = targetAmplitude - fabs(input);
  if (error > 0) agcGain += attackRate * error;
  else           agcGain += decayRate * error;

  agcGain = constrain(agcGain, minGain, maxGain);
  return input * agcGain;
}

// ==============================================================================
// [5] Keyer Processing
// ==============================================================================

void calculateTiming() {
  dotDuration = 1200 / wpm;
  dashDuration = dotDuration * 3;
  elementSpace = dotDuration;
  charSpace = dotDuration * 3;
  wordSpace = dotDuration * 7;
}

void initKeyer() {
  calculateTiming();
  sending = false;
  sendingDot = false;
  sendingDash = false;
  transmitting = false;
}

void handleKeyer() {
  static unsigned long stateStartTime = 0;
  static int keyerState = 0; // 0: Idle, 1: Sending (Mark), 2: Space

  unsigned long currentTime = millis();

  // --- Vertical-stroke telegraph key mode ---
  if (straightKeyMode) {
    bool straightKeyDown = (digitalRead(PADDLE_DASH) == LOW);
    if (straightKeyDown && !transmitting)      startTransmit();
    else if (!straightKeyDown && transmitting) stopTransmit();
    return;
  }

  // --- Electric Mode ---
  bool currentDot  = (digitalRead(PADDLE_DOT)  == LOW);
  bool currentDash = (digitalRead(PADDLE_DASH) == LOW);

  // State 0: Idle (waiting for paddle press)
  if (keyerState == 0) {
    if (currentDot) {
      keyerState = 1;
      sendingDot = true;
      sendingDash = false;
      stateStartTime = currentTime;
      startTransmit();
    } else if (currentDash) {
      keyerState = 1;
      sendingDot = false;
      sendingDash = true;
      stateStartTime = currentTime;
      startTransmit();
    } else {
      sendingDot = false;
      sendingDash = false;
    }
  }
  // State 1: Transmitting (short or long beep playing)
  else if (keyerState == 1) {
    unsigned long duration = sendingDash ? dashDuration : dotDuration;
    if (currentTime - stateStartTime >= duration) {
      stopTransmit();
      keyerState = 2; // Always transition to Space state after transmission completes
      stateStartTime = currentTime;
    }
  }
  // State 2: Space (inter-element gap)
  else if (keyerState == 2) {
    if (currentTime - stateStartTime >= elementSpace) {
      keyerState = 0; // Return to Idle after the inter-element space has elapsed
    }
  }
}

// ==============================================================================
// [6] User Interface and Display
// ==============================================================================

static String fmtMHz(unsigned long f_hz){
  char buf[16];
  unsigned long mhz = f_hz / 1000000UL;
  unsigned long khz = (f_hz % 1000000UL) / 1000UL;
  snprintf(buf, sizeof(buf), "%lu.%03lu", mhz, khz);
  return String(buf);
}

int barLength(double d) {
  float fy = SCOPE_SENSITIVITY * (log10(d) + SCOPE_OFFSET);
  int y = constrain((int)fy, 0, 20);
  return y;
}

// --- Rotary Encoder Interrupt ---
void rotary_encoder() {
  unsigned char result = r.process();
  if (result) {
    if (result == DIR_CW) FREQ += STEP;
    else                  FREQ -= STEP;
  }
  FREQ = constrain(FREQ, LOW_FREQ, HI_FREQ);
  FREQ_ULL = FREQ * 100ULL;
}

// --- Volume Adjustment Mode ---
void changeVolume() {
  detachInterrupt(0); detachInterrupt(1); // Temporarily disable encoder interrupts

  // ★ Change 1: Range changed to 10–30 (×1.0 – ×3.0)
  int currentVolInt = constrain(volumeMultiplier * 10, 10, 30);

  auto drawVol = [&](int val) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_8x13B_tr);
    u8g2.setCursor(18, 6);
    u8g2.print("VOLUME: ");
    u8g2.print(val / 10.0, 1);
    u8g2.print(" x");
    u8g2.drawFrame(14, 20, 100, 8);

    // ★ Change 2: Formula corrected so input 10 gives bar width 0, input 30 gives bar width 100
    u8g2.drawBox(14, 20, (val - 10) * 5, 8);

    u8g2.sendBuffer();
  };

  drawVol(currentVolInt);
  bool btnPrev = HIGH;

  while (true) {
    unsigned char res = r.process();
    if (res) {
      if (res == DIR_CW) currentVolInt++;
      else               currentVolInt--;

      // ★ Change 3: Limit the range to 10–30
      currentVolInt = constrain(currentVolInt, 10, 30);

      volumeMultiplier = currentVolInt / 10.0;
      drawVol(currentVolInt);
      delay(10);
    }

    bool btn = (digitalRead(KEY_MODE_BUTTON) == LOW);
    if (!btn && btnPrev == LOW) { // Click to exit
      saveVolToEEPROM(currentVolInt);
      break;
    }
    btnPrev = btn;
    delay(5);
  }

  attachInterrupt(0, rotary_encoder, CHANGE);
  attachInterrupt(1, rotary_encoder, CHANGE);
  u8g2.clearBuffer();
  u8g2.sendBuffer();
}

// --- WPM Change Mode ---
void changeWPM() {
  detachInterrupt(0); detachInterrupt(1);
  int newWpm = wpm;

  auto draw = [&](int val){
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_8x13B_tr);
    u8g2.setCursor(38, 6);
    u8g2.print("WPM: "); u8g2.print(val);
    u8g2.setFont(u8g2_font_micro_tr);
    u8g2.setCursor(15, 20); u8g2.print("Rotate: +/-");
    u8g2.setCursor(15, 26); u8g2.print("Press: Save  Hold: Cancel");
    u8g2.sendBuffer();
  };

  draw(newWpm);
  unsigned long holdStart = 0;
  bool btnPrev = HIGH;

  while (true) {
    unsigned char res = r.process();
    if (res) {
      if (res == DIR_CW) newWpm++;
      else               newWpm--;
      newWpm = constrain(newWpm, 5, 40);
      draw(newWpm);
      delay(10);
    }

    bool btn = (digitalRead(STEP_BUTTON) == LOW);
    unsigned long now = millis();
    if (btn && !btnPrev) {
      holdStart = now;
    } else if (!btn && btnPrev) {
      if (holdStart && (now - holdStart) >= 800) break; // Long press: cancel
      else {
        wpm = newWpm;
        saveWPMToEEPROM(wpm);
        initKeyer();
        break; // Save
      }
    }
    btnPrev = btn;
    delay(5);
  }

  attachInterrupt(0, rotary_encoder, CHANGE);
  attachInterrupt(1, rotary_encoder, CHANGE);
}

// --- Button Press Detection Routine ---
void handleKeyModeButton() {
  static unsigned long pressStartTime = 0;
  static bool isPressing = false;
  static bool longPressHandled = false;
  static bool lastReading = HIGH;
  const unsigned long debounceDelay = 30;
  const unsigned long longPressDelay = 800;

  bool reading = digitalRead(KEY_MODE_BUTTON);
  if (reading == LOW && lastReading == HIGH) {
    pressStartTime = millis();
    isPressing = true;
    longPressHandled = false;
  } else if (reading == LOW && isPressing) {
    if (!longPressHandled && (millis() - pressStartTime) > longPressDelay) {
      longPressHandled = true;
      changeVolume();
    }
  } else if (reading == HIGH && lastReading == LOW) {
    isPressing = false;
    if (!longPressHandled && (millis() - pressStartTime) > debounceDelay) {
      straightKeyMode = !straightKeyMode;
      saveKeyModeToEEPROM(straightKeyMode);

      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_8x13B_tr);
      u8g2.setCursor(20, 4);  u8g2.print("KEY MODE");
      u8g2.setCursor(40, 18); u8g2.print(straightKeyMode ? "STRA" : "KEY");
      u8g2.sendBuffer();
      delay(600);
    }
  }
  lastReading = reading;
}

void Fnc_Stp() {
  unsigned long pressStart = millis();
  while (digitalRead(STEP_BUTTON) == LOW) {
    if (millis() - pressStart > 1000) { changeWPM(); return; }
    delay(10);
  }

  if (stepMode == 0)      { stepMode = 1; STEP = 100; }
  else if (stepMode == 1) { stepMode = 2; STEP = 10; }
  else                    { stepMode = 0; STEP = 1000; }

  saveStepToEEPROM(STEP);
  delay(10);
}

// --- Scope Meter Drawing ---
void showScope() {
  const int BASE_Y = 26; // 30→26: Raised bandscope bottom by 4 px to secure CW text display area
  const int PX_PER_SIDE = 63;
  const int MAX_D = 25;  // Increased scope maximum height by 4 px
  const float BIN_HZ = (float)sampleRate / (float)SAMPLES;
  int binsTarget = constrain((int)(SCOPE_SPAN_HZ / BIN_HZ + 0.5f), 0, SAMPLES / 2);
  if (binsTarget > (SAMPLES / 2)) binsTarget = (SAMPLES / 2);
  float binsPerPixel = (float)binsTarget / (float)PX_PER_SIDE;

  peakDecayDiv++;
  bool doDecay = (peakDecayDiv >= PEAK_DECAY_FRAMES);
  if (doDecay) peakDecayDiv = 0;

  float offsetBins = (float)CW_TONE / BIN_HZ;

  // Clear the waterfall row for this frame
  for(int i=0; i<128; i++) waterfallHistory[waterfallIndex][i] = 0;

  //=======================================
  // Draw right half (positive frequencies)
  //=======================================
  int prevX_now = -1, prevY_now = -1, prevX_pk = -1, prevY_pk = -1;

  // ★ Change 1: Start xi at 0; center x coordinate is 63
  for (int xi = 0; xi <= PX_PER_SIDE; xi++) {
    float exactBin = (xi * binsPerPixel) + offsetBins;
    int bin = constrain((int)(exactBin + 0.5f), 0, (SAMPLES / 2 - 1));
    int d = 0;

    if (bin > DC_BLANK_BINS) {
      d = constrain((barLength(vReal[bin]) + barLength(vImag[bin])) / 2, 0, MAX_D);
    } else {
      // ★ Change 2: Interpolate using adjacent bin to smooth DC spike at the center joint.
      // (Remove this if-else and compute d directly if you want to inspect center noise.)
      int nextBin = DC_BLANK_BINS + 1;
      d = constrain((barLength(vReal[nextBin]) + barLength(vImag[nextBin])) / 2, 0, MAX_D);
    }

    uint8_t pk = peakR[xi];
    if ((uint8_t)d >= pk) pk = (uint8_t)d;
    else if (doDecay && pk > 0) pk--;
    peakR[xi] = pk;

    // ★ Change 3: Draw from center (63) toward the right
    int x = 63 + xi;
    waterfallHistory[waterfallIndex][x] = d;

    int y_now = BASE_Y - d;
    int y_pk = constrain(BASE_Y - (int)pk - 1 + PEAK_Y_OFFSET, 0, BASE_Y);
    if (prevX_now >= 0) u8g2.drawLine(prevX_now, prevY_now, x, y_now);
    if (prevX_pk >= 0)  u8g2.drawLine(prevX_pk, prevY_pk, x, y_pk);
    prevX_now = x; prevY_now = y_now; prevX_pk = x; prevY_pk = y_pk;
  }

  //=======================================
  // Draw left half (negative frequencies)
  //=======================================
  prevX_now = -1; prevY_now = -1; prevX_pk = -1; prevY_pk = -1;

  // ★ Change 4: Start xi at 1 to avoid overlap since center (xi=0) was already drawn on the right
  for (int xi = 1; xi <= PX_PER_SIDE; xi++) {
    float exactBin = -(xi * binsPerPixel) + offsetBins;
    int bin = constrain((int)(abs(exactBin) + 0.5f), 0, (SAMPLES / 2 - 1));
    int d = 0;

    if (bin > DC_BLANK_BINS) {
      if (exactBin >= 0) {
        d = constrain((barLength(vReal[bin]) + barLength(vImag[bin])) / 2, 0, MAX_D);
      } else {
        d = constrain((barLength(vReal[SAMPLES - bin]) + barLength(vImag[SAMPLES - bin])) / 2, 0, MAX_D);
      }
    } else {
      // Similarly, on the left side the DC region is interpolated from the adjacent bin
      int nextBin = SAMPLES - (DC_BLANK_BINS + 1);
      d = constrain((barLength(vReal[nextBin]) + barLength(vImag[nextBin])) / 2, 0, MAX_D);
    }

    uint8_t pk = peakL[xi];
    if ((uint8_t)d >= pk) pk = (uint8_t)d;
    else if (doDecay && pk > 0) pk--;
    peakL[xi] = pk;

    // ★ Change 5: Draw lines from center (63) toward the left
    int x = 63 - xi;
    waterfallHistory[waterfallIndex][x] = d;

    int y_now = BASE_Y - d;
    int y_pk = constrain(BASE_Y - (int)pk - 1 + PEAK_Y_OFFSET, 0, BASE_Y);
    if (prevX_now >= 0) u8g2.drawLine(prevX_now, prevY_now, x, y_now);
    if (prevX_pk >= 0)  u8g2.drawLine(prevX_pk, prevY_pk, x, y_pk);
    prevX_now = x; prevY_now = y_now; prevX_pk = x; prevY_pk = y_pk;
  }

  // Center vertical line removed (commented out)
  //u8g2.drawLine(VFO_MARKER_X, (BASE_Y - MAX_D) + MARKER_TOP_MARGIN, VFO_MARKER_X, BASE_Y);

  // Frequency scale labels (y=49: 1 px below bottom line y=48, 5 px above CW text area y=54)
  unsigned long leftHz  = (FREQ > 15000) ? (FREQ - 15000) : 0;
  unsigned long midHz   = FREQ;
  unsigned long rightHz = FREQ + 15000;

  u8g2.setFont(u8g2_font_micro_tr);
  u8g2.drawStr(0, 49, fmtMHz(leftHz).c_str());

  String midS = fmtMHz(midHz);
  int midX = constrain(64 - (midS.length() * 4) / 2, 34, 128);
  u8g2.drawStr(midX, 49, midS.c_str());

  String rS = fmtMHz(rightHz);
  int rX = constrain(128 - (rS.length() * 4), 98, 128);
  u8g2.drawStr(rX, 49, rS.c_str());

  // Upper section info display
  u8g2.setFont(u8g2_font_8x13B_tr);
  String freqt = String(FREQ);
  u8g2.setCursor(2, 1);
  u8g2.print(freqt.substring(0, 1) + "." + freqt.substring(1, 4) + "." + freqt.substring(4));

  u8g2.setFont(u8g2_font_micro_tr);
  if (transmitting) {
    u8g2.setCursor(90, 6);
    u8g2.print("TX ");
    if (sendingDot) u8g2.print("DOT");
    else if (sendingDash) u8g2.print("DASH");
    else u8g2.print("KEY");
    u8g2.print(" "); u8g2.print(wpm); u8g2.print("WPM");
  }

  u8g2.setCursor(78, 0); u8g2.print("STEP:");
  if (STEP == 1000)      u8g2.drawStr(102, 0, "1000");
  else if (STEP == 100)  u8g2.drawStr(102, 0, " 100");
  else                   u8g2.drawStr(106, 0, " 10");
}

void showS_meter() {
  for (int xi = 1; xi < 64; xi++) {
    int d = (barLength(vReal[xi*2]) + barLength(vImag[xi*2+1])) * 2.0;
    u8g2.drawBox(86, 6, d, 6);
  }
}

void displayWaterfall() {
  for (int y = 0; y < WATERFALL_HEIGHT; y++) {
    // Compute index so y=0 is the most recent (top) and y=20 is the oldest (bottom)
    int histY = (waterfallIndex - y + WATERFALL_HEIGHT) % WATERFALL_HEIGHT;
    for (int x = 0; x < 128; x++) {
      if (waterfallHistory[histY][x] > THRESHOLD) {
        // Plot in the range from Y=27 to Y=47
        u8g2.drawPixel(x, 27 + y);
      }
    }
  }
  // Advance index for the next frame
  waterfallIndex = (waterfallIndex + 1) % WATERFALL_HEIGHT;
}

void showGraphics() {
  // Upper divider line y=26 (scope bottom), lower divider line y=48 (waterfall bottom)
  u8g2.drawHLine(0, 26, 128);
  u8g2.drawHLine(0, 48, 128);
  u8g2.drawFrame(86, 6, 42, 6);
  u8g2.drawBox(0, 26, 2, 2);
  u8g2.drawBox(126, 26, 2, 2);

  u8g2.setFont(u8g2_font_micro_tr);
  u8g2.drawStr(78, 6, "S:");
  u8g2.drawStr(78, 0, "STEP:");
  u8g2.drawStr(122, 0, "Hz");

  String freqt = String(FREQ);
  u8g2.setFont(u8g2_font_8x13B_tr);
  u8g2.setCursor(2, 1);
  u8g2.print(freqt.substring(0, 1) + "." + freqt.substring(1, 4) + "." + freqt.substring(4));
}

// ==============================================================================
// [7] Setup and Loop (Core0: UI and Control)
// ==============================================================================

void setup() {
  pinMode(LED_INDICATOR, OUTPUT);
  pinMode(RX_SW, OUTPUT);
  pinMode(PADDLE_DOT, INPUT_PULLUP);
  pinMode(PADDLE_DASH, INPUT_PULLUP);
  pinMode(KEY_MODE_BUTTON, INPUT_PULLUP);
  pinMode(STEP_BUTTON, INPUT_PULLUP);

  EEPROM.begin(512);
  Wire.begin();

  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 25001042, 0);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);
  si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_8MA);
  si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_8MA);
  si5351.output_enable(SI5351_CLK2, 0);
  digitalWrite(RX_SW, LOW);

  r.begin();
  attachInterrupt(0, rotary_encoder, CHANGE);
  attachInterrupt(1, rotary_encoder, CHANGE);

  // Load settings from EEPROM
  unsigned long savedFreq = readFrequencyFromEEPROM();
  FREQ = (savedFreq >= LOW_FREQ && savedFreq <= HI_FREQ) ? savedFreq : 7000000;
  wpm = readWPMFromEEPROM();
  straightKeyMode = readKeyModeFromEEPROM();
  volumeMultiplier = readVolFromEEPROM() / 10.0;

  int s = readStepFromEEPROM();
  if (s == 10 || s == 100 || s == 1000) {
    STEP = s;
    if (STEP == 1000)      stepMode = 0;
    else if (STEP == 100)  stepMode = 1;
    else                   stepMode = 2;
  } else {
    STEP = 1000;
  }

  initKeyer();
  FREQ_OLD = FREQ;
  Freq_Set();

  analogReadResolution(12);

  u8g2.begin();
  u8g2.setFlipMode(0);
  u8g2.setFont(u8g2_font_micro_tr);
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();

  u8g2.clearBuffer();
  u8g2.drawStr(34, 16, "7MHz CW TRX v5.0");
  u8g2.drawStr(28, 24, "BandScope & Waterfall");
  u8g2.drawStr(52, 32, "JR3XNW");
  u8g2.sendBuffer();
  delay(1000);
}

void loop() {
  static unsigned long lastUIUpdate = 0;

  handleKeyModeButton();
  handleKeyer();
  handleCWDecoder();        // CW auto-decode (called every loop)

  if (!transmitting) {
    // Detect frequency change and update immediately to prevent display freeze
    if (FREQ != FREQ_OLD) {
      muteCounter = 500;  // Apply mute to reduce pop noise while tuning
      Freq_Set();
      FREQ_OLD = FREQ;
      saveFrequencyToEEPROM(FREQ);
    }

    if (digitalRead(STEP_BUTTON) == LOW) Fnc_Stp();

    // When 256 samples have been collected by Core1, copy them to the FFT buffer
    if (sharedBufferReady) {
      // ★ Q amplitude correction (reduces mirror image caused by I/Q amplitude imbalance)
      // Compares RMS of I and Q, auto-adjusts Q gain to match I within ±25%.
      // Safe because no phase correction is performed — no risk of coefficient explosion.
      static float avgI2 = 0.01f, avgQ2 = 0.01f;
      for (int i = 0; i < SAMPLES; i++) {
        float si = sharedBufferI[i], sq = sharedBufferQ[i];
        avgI2 = avgI2 * 0.9995f + si * si * 0.0005f;
        avgQ2 = avgQ2 * 0.9995f + sq * sq * 0.0005f;
      }
      // qGain: clamped to safe range 0.8–1.25 (no correction if avgQ2 is negligibly small)
      float qGain = (avgQ2 > 0.0001f) ? constrain(sqrtf(avgI2 / avgQ2), 0.8f, 1.25f) : 1.0f;

      for (int i = 0; i < SAMPLES; i++) {
        vReal[i] = sharedBufferI[i];
        vImag[i] = sharedBufferQ[i] * qGain;
      }
      sharedBufferReady = false;

      // Update display every 50 ms to reduce rendering load
      if (millis() - lastUIUpdate >= 50) {
        digitalWrite(LED_INDICATOR, HIGH);
        FFT.windowing(vReal, SAMPLES, FFTWindow::Hamming, FFTDirection::Forward);
        FFT.windowing(vImag, SAMPLES, FFTWindow::Hamming, FFTDirection::Forward);
        FFT.compute(vReal, vImag, SAMPLES, FFTDirection::Reverse);
        FFT.complexToMagnitude(vReal, vImag, SAMPLES);

        u8g2.clearBuffer();
        showS_meter();
        showScope();
        displayWaterfall();
        showGraphics();
        displayCWText();    // CW decoded text (bottom 10 px strip)
        u8g2.sendBuffer();
        digitalWrite(LED_INDICATOR, LOW);
        lastUIUpdate = millis();
      }
    }
  } else {
    // Display update during transmission
    if (millis() - lastUIUpdate >= 50) {
      u8g2.clearBuffer();
      showGraphics();
      displayCWText();      // Keep showing decoded text during TX
      u8g2.sendBuffer();
      lastUIUpdate = millis();
    }
  }
}

// ==============================================================================
// [8] Setup1 and Loop1 (Core1: Audio and DSP Processing)
// ==============================================================================

void setup1() {
  pinMode(speakerPin, OUTPUT);
  pinMode(inputPinI, INPUT);
  pinMode(inputPinQ, INPUT);

  analogReadResolution(12);
  analogWriteResolution(12);
  analogWriteFreq(pwmFrequency);
}

void loop1() {
  static unsigned long lastSampleTime = 0;
  unsigned long now = micros();

  // ★ TX start timestamp synchronization
  // The RX path takes ~26–30 µs per loop due to 4× oversampling, causing lastSampleTime
  // to fall tens of ms behind now. At TX start, the fast TX path catches up hundreds of
  // times instantly, advancing toneAngle rapidly and producing an extremely high-pitched tone.
  // Detect TX start and reset lastSampleTime = now to prevent this catch-up burst.
  static bool prevTxState = false;
  if (transmitting && !prevTxState) {
    lastSampleTime = now;  // Timestamp reset: eliminates catch-up burst
  }
  prevTxState = transmitting;

  // ★ Clock interval: execute at exactly 25 µs (40 kHz) intervals to eliminate jitter
  const unsigned long targetInterval = 25;
  if (now - lastSampleTime < targetInterval) return;
  lastSampleTime += targetInterval; // Increment by fixed step instead of using now, preventing error accumulation

  static bool wasTransmitting = false;
  static float smoothMute = 1.0f;
  static float lastRxPwm = 2047.5f;  // Last RX output value (used for TX start crossfade)

  // ★ Envelope variable for smooth volume fade-in and fade-out
  static float txEnvelope = 0.0f;
  const float envelopeStep = 0.002f; // Transition speed: ~12.5 ms fade over 500 loops at 0.002/loop

  // ★ Smoothly ramp envelope between 0.0 and 1.0 based on transmit state
  if (transmitting) {
    txEnvelope += envelopeStep;
    if (txEnvelope > 1.0f) txEnvelope = 1.0f;
  } else {
    txEnvelope -= envelopeStep;
    if (txEnvelope < 0.0f) txEnvelope = 0.0f;
  }

  // ★ Keep playing tone as long as envelope is above zero (instead of hard on/off switching)
  if (txEnvelope > 0.0f) {
    // Float phase accumulator: 2π × CW_TONE / sampleRate = 2π × 700 / 40000 = 0.10996 rad/sample
    // No integer overflow; generates accurate 700 Hz
    static float toneAngle = 0.0f;
    toneAngle += 2.0f * (float)PI * (float)CW_TONE / (float)sampleRate;
    if (toneAngle >= 2.0f * (float)PI) toneAngle -= 2.0f * (float)PI;
    float sineValue = sinf(toneAngle);

    float currentVol = 0.1f * txEnvelope; // Monitor volume (0.1) × envelope
    float txPwm = (sineValue * currentVol + 1.0f) * 2047.5f;

    // Pop noise prevention at TX start: crossfade from last RX output to TX tone
    // Blend over txEnvelope 0→0.05 (~25 samples = 0.6 ms)
    const float fadeWindow = 0.05f;
    float blend = (txEnvelope < fadeWindow) ? (txEnvelope / fadeWindow) : 1.0f;
    float blendedPwm = lastRxPwm * (1.0f - blend) + txPwm * blend;
    uint16_t pwmOutput = (uint16_t)constrain((int)blendedPwm, 0, 4095);

    analogWrite(speakerPin, pwmOutput);
    wasTransmitting = true;
    return;
  }

  if (wasTransmitting) {
    dcOffsetI = 0; dcOffsetQ = 0;
    sharedIndex = 0;
    // TX→RX transition cleanup: reset residual state to prevent pop noise and IIR transient response
    memset(iir_x1, 0, sizeof(iir_x1));
    memset(iir_x2, 0, sizeof(iir_x2));
    memset(iir_y1, 0, sizeof(iir_y1));
    memset(iir_y2, 0, sizeof(iir_y2));
    memset(qDelayBuffer, 0, sizeof(qDelayBuffer));
    qDelayIndex = 0;
    smoothMute = 0.0f;   // Fade in on RX resume (pop noise prevention)
    lastRxPwm = 2047.5f; // Reset to midpoint for next TX start crossfade
    wasTransmitting = false;
  }

  // 1. Sampling (4× oversampling for +3 dB SNR improvement)
  long sumI = 0, sumQ = 0;
  for (int i = 0; i < 4; i++) {
    sumI += analogRead(inputPinI);
    sumQ += analogRead(inputPinQ);
  }
  float rawI = ((float)sumI / 4.0f / 2047.5f) - 1.0f;
  float rawQ = ((float)sumQ / 4.0f / 2047.5f) - 1.0f;

  // 2. Mute (pop noise suppression) and DC offset control
  if (muteCounter > 0) {
    muteCounter--;
    smoothMute *= 0.80f;
    rawI -= dcOffsetI;
    rawQ -= dcOffsetQ;
  } else {
    smoothMute = (smoothMute * 0.80f) + (1.0f * 0.20f);
    dcOffsetI = (dcOffsetI * 0.999f) + (rawI * 0.001f);
    dcOffsetQ = (dcOffsetQ * 0.999f) + (rawQ * 0.001f);
    rawI -= dcOffsetI;
    rawQ -= dcOffsetQ;
  }

  float mutedI = rawI * smoothMute;
  float mutedQ = rawQ * smoothMute;

  // 3. Write data to shared buffer (for FFT)
  sharedBufferI[sharedIndex] = mutedI;
  sharedBufferQ[sharedIndex] = mutedQ;
  sharedIndex++;
  if (sharedIndex >= SAMPLES) {
    sharedIndex = 0;
    sharedBufferReady = true;
  }

  // 4. CW demodulation (★ no downsampling; signal passed directly through 40 kHz IIR filter)
  float demodulated = cwDemodulate(mutedI, mutedQ, FREQ);

  // 4b. CW signal detection (SNR ratio method)
  // Problem: absolute level before AGC can be negligibly small depending on signal strength,
  //          making a fixed threshold unusable.
  // Solution: track noise floor at very low speed; share SNR ratio = current / noise floor.
  {
    static float cwEnvLP  = 0.0f;
    static float cwNoise  = 0.05f;   // Estimated noise floor (starts high, converges to measured value)
    float absD = fabs(demodulated);
    // Asymmetric LPF: attack ~0.5 ms, decay ~12.5 ms
    cwEnvLP = (absD > cwEnvLP)
              ? cwEnvLP * 0.95f  + absD * 0.05f
              : cwEnvLP * 0.998f + absD * 0.002f;
    // Noise floor tracking: update only when SNR ratio < 1.5 (no signal present)
    float ratio = cwEnvLP / max(cwNoise, 0.0001f);
    if (ratio < 1.5f)
      cwNoise = cwNoise * 0.9995f + cwEnvLP * 0.0005f; // No signal: converges in ~50 ms
    else
      cwNoise = cwNoise * 0.99999f + cwEnvLP * 0.00001f; // Signal present: nearly frozen
    cwEnvelope = ratio; // Share SNR ratio with Core0 (≥1.5 = signal detected)
  }

  // 4c. CW event generation (Core1 measures precise timing; passes to Core0 via ring buffer)
  // 80-sample (2 ms @ 40 kHz) debounce prevents noise false-triggers
  {
    static bool     cwSt  = false;   // Current confirmed state (true = key down)
    static uint32_t cwCnt = 0;       // Sample count for the current state
    static uint8_t  cwDB  = 0;       // Debounce counter
    const  uint8_t  DB_TH = 80;      // Debounce threshold: 2 ms (80 samples @ 40 kHz)

    bool raw = (cwEnvelope > CW_DETECT_THRESHOLD);

    if (raw != cwSt) {
      // State change candidate: increment debounce counter
      if (++cwDB >= DB_TH) {
        // Debounce confirmed: write event to ring buffer
        uint16_t ms = (uint16_t)min(cwCnt / 40UL, 60000UL); // Convert sample count to ms
        uint8_t nw = (cwEvWr + 1) % CW_EVBUF_SIZE;
        if (nw != cwEvRd) {  // Write only if buffer is not full
          cwEvBuf[cwEvWr].type  = cwSt ? (uint8_t)1 : (uint8_t)0;
          cwEvBuf[cwEvWr].durMs = ms;
          cwEvWr = nw;
        }
        cwSt  = raw;
        cwCnt = 0;
        cwDB  = 0;
      }
    } else {
      cwDB = 0;  // No state change: reset debounce counter
    }
    cwCnt++;
  }

  // 5. Apply AGC
  float agcOutput;
  if (smoothMute < 0.9f) {
    agcOutput = demodulated * agcGain; // Bypass AGC during fade-in to prevent gain hunting
  } else {
    agcOutput = applyAGC(demodulated);
  }
  agcOutput *= volumeMultiplier;

  // 6. PWM output (★ generated at 40 kHz every iteration to eliminate 10 kHz current-noise artifacts)
  uint16_t pwmOutput = (uint16_t)((agcOutput + 1.0f) * 2047.5f);
  lastRxPwm = (float)constrain((int)pwmOutput, 0, 4095); // Record last RX output for TX start crossfade
  analogWrite(speakerPin, constrain(pwmOutput, 0, 4095));
}

// ==============================================================================
// [9] CW Decoding (executed on Core0)
// ==============================================================================

// --- Morse Code Table ---
struct MorseEntry { const char* code; char ch; };
const MorseEntry MORSE_TABLE[] = {
  // Letters
  {".-",   'A'}, {"-...", 'B'}, {"-.-.", 'C'}, {"-..",  'D'}, {".",    'E'},
  {"..-.", 'F'}, {"--.",  'G'}, {"....", 'H'}, {"..",   'I'}, {".---", 'J'},
  {"-.-",  'K'}, {".-..", 'L'}, {"--",   'M'}, {"-.",   'N'}, {"---",  'O'},
  {".--.", 'P'}, {"--.-", 'Q'}, {".-.",  'R'}, {"...",  'S'}, {"-",    'T'},
  {"..-",  'U'}, {"...-", 'V'}, {".--",  'W'}, {"-..-", 'X'}, {"-.--", 'Y'},
  {"--..", 'Z'},
  // Digits
  {"-----", '0'}, {".----", '1'}, {"..---", '2'}, {"...--", '3'},
  {"....-", '4'}, {".....", '5'}, {"-....", '6'}, {"--...", '7'},
  {"---..", '8'}, {"----.", '9'},
  // Punctuation
  {".-.-.-", '.'}, {"--..--", ','}, {"..--..", '?'}, {"-..-.",  '/'},
  {"-....-", '-'}, {".----.", '\''},{".-..-.", '"'}, {"---...", ':'},
  {"-.-.--", '!'}, {".--.-.", '@'},
  // Amateur radio prosigns (mapped to printable symbols)
  // AR(+)=end of message, BT(=)=new paragraph, KN(()=go ahead specific station, AS(~)=wait, SK(*)=end of contact
  {".-.-.",  '+'}, // AR: end of message
  {"-...-",  '='}, // BT: paragraph / new line
  {"-.--.",  '('}, // KN: go ahead (specific station)
  {".-...",  '~'}, // AS: wait
  {"...-.-", '*'}, // SK: end of contact
  {nullptr,  '\0'}
};

// Convert a Morse code string to a character (returns '?' if unknown)
char decodeMorse(const char* code) {
  for (int i = 0; MORSE_TABLE[i].code != nullptr; i++) {
    if (strcmp(code, MORSE_TABLE[i].code) == 0) return MORSE_TABLE[i].ch;
  }
  return '?';
}

// Append a decoded character to the buffer (add at right end, discard oldest when full)
void addCWDecodedChar(char c) {
  if (c == '\0') return;
  if (cwDecodedLen < CW_DECODED_MAX) {
    cwDecodedBuf[cwDecodedLen++] = c;
    cwDecodedBuf[cwDecodedLen]   = '\0';
  } else {
    memmove(cwDecodedBuf, cwDecodedBuf + 1, CW_DECODED_MAX - 1);
    cwDecodedBuf[CW_DECODED_MAX - 1] = c;
    cwDecodedBuf[CW_DECODED_MAX]     = '\0';
  }
}

// Display CW decoded text in the bottom strip of the screen (y=54–63, 10 px)
// Newest character is always anchored to the right edge; older characters scroll left
// Font: 5x8 px fixed-width, 2 px gap → 7 px/char, maximum 18 characters in 128 px width
void displayCWText() {
  const int CHAR_W   = 5;
  const int CHAR_GAP = 2;
  const int STEP     = CHAR_W + CHAR_GAP;  // 7 px/char
  const int MAX_VIS  = 128 / STEP;         // 18 characters
  const int TEXT_Y   = 55;                 // Vertically centered in y=54–63 (1 px top margin)

  u8g2.setFont(u8g2_font_5x8_tr);
  int count  = (cwDecodedLen < MAX_VIS) ? cwDecodedLen : MAX_VIS; // Number of characters to display
  int start  = cwDecodedLen - count;        // Start index within the buffer
  int xStart = (MAX_VIS - count) * STEP;   // Right-align: offset left when fewer than MAX_VIS chars
  for (int i = 0; i < count; i++) {
    char buf[2] = { cwDecodedBuf[start + i], '\0' };
    u8g2.drawStr(xStart + i * STEP, TEXT_Y, buf);
  }
}

// CW decoder (event-driven architecture)
// Consumes mark/space duration events precisely measured by Core1 and decodes characters on Core0.
// Core0 pauses ~50 ms for FFT/rendering, so timing measurement is delegated to Core1's ring buffer.
void handleCWDecoder() {
  if (transmitting) { cwEvRd = cwEvWr; return; }  // Flush buffer during TX to discard stale events

  static float dotEst          = 60.0f;  // Estimated dot length (ms); initial value = 20 WPM
  static char  morse[10]       = "";     // Current received Morse elements (max 8 + null terminator)
  static int   morseLen        = 0;
  static bool  charDecoded     = false;  // Whether a character has already been confirmed in this silence
  static bool  wordAdded       = false;  // Whether a word space has already been added in this silence
  static unsigned long keyUpMs = 0;      // Timestamp of the most recent key-up event (ms)
  static bool  keyIsUp         = true;   // Whether the key is currently released

  unsigned long nowMs = millis();

  // ── Consume all events delivered from Core1 ──
  while (cwEvRd != cwEvWr) {
    CWEvent ev;
    ev.type  = cwEvBuf[cwEvRd].type;   // Copy volatile fields individually
    ev.durMs = cwEvBuf[cwEvRd].durMs;
    cwEvRd   = (cwEvRd + 1) % CW_EVBUF_SIZE;

    if (ev.type == 1) {
      // Mark ended (key-up confirmed): ev.durMs = duration of the preceding key-down
      uint16_t dur = ev.durMs;
      if (dur >= (uint16_t)(dotEst * 0.3f) && morseLen < 8) {
        if (dur < (uint16_t)(dotEst * 2.0f)) {
          // Dot: update dot length estimate by 15%
          morse[morseLen++] = '.';
          dotEst = dotEst * 0.85f + (float)dur * 0.15f;
        } else {
          // Dash: update dot length estimate using dash ÷ 3
          morse[morseLen++] = '-';
          dotEst = dotEst * 0.85f + ((float)dur / 3.0f) * 0.15f;
        }
        morse[morseLen] = '\0';
        dotEst = constrain(dotEst, 30.0f, 240.0f);  // Clamp to 5–40 WPM range
      }
      keyUpMs     = nowMs;
      keyIsUp     = true;
      charDecoded = false;
      wordAdded   = false;

    } else {
      // Space ended (key-down confirmed): ev.durMs = duration of the preceding silence
      // Character/word confirmation is left to the silence timeout below (prevents double processing)
      keyIsUp = false;
    }
  }

  // ── Character / word confirmation by silence timeout ──
  if (keyIsUp) {
    unsigned long silMs = nowMs - keyUpMs;

    // Inter-character space: silence ≥ dotEst × 2.5 → confirm one character
    if (!charDecoded && morseLen > 0
        && silMs >= (unsigned long)(dotEst * 2.5f)) {
      addCWDecodedChar(decodeMorse(morse));
      morseLen    = 0;
      morse[0]    = '\0';
      charDecoded = true;
    }

    // Inter-word space: silence ≥ dotEst × 6 → insert space character
    if (charDecoded && !wordAdded
        && silMs >= (unsigned long)(dotEst * 6.0f)) {
      addCWDecodedChar(' ');
      wordAdded = true;
    }
  }
}
