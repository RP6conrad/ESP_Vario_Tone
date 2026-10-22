/*
  Speed-Vario Tone Generator  (v2 proposal, based on Jan's Vario_tone.h)
  ----------------------------------------------------------------------
  Changes vs v1:
   1. Acceleration = least-squares slope of raw GPS speed over the last
      ACCEL_WINDOW_S (default 1.0 s), instead of S1 - S10. The 10 s average
      made the accel/decel switch ~3 s late after every speed turn.
   2. Hysteresis on the accel/decel decision, so it doesn't flicker.
   3. Beep timing runs from varioTick() in loop(), not from the GPS callback.
      In v1 the callback ran at 5 Hz, so beeps were quantised to 200 ms, and
      any beep period < 200 ms became a continuous tone (no beeps near the top
      of the speed range).
   4. Beep rate follows acceleration magnitude (as the v1 comment intended),
      pitch follows speed.
   5. Silent below lower_limit (configurable margin), not just below 3 m/s.

  Usage:
    - In the GPS message handler (every fix, 5/10 Hz), with RAW gps speed in m/s:
        varioNewSpeed(gps_speed_ms, config.vario_low_limit, config.vario_high_limit);
    - In loop(), as often as possible:
        varioTick();

  Hardware/API unchanged: ledcWriteTone(LEDC_CHANNEL, f) on GPIO21.
*/
#ifndef VARIO_TONE_H
#define VARIO_TONE_H
#include <Arduino.h>

// ---------- Config ----------
const int BUZZER_PIN      = 21;
const int LEDC_CHANNEL    = 0;
const int LEDC_RESOLUTION = 8;

// Acceleration estimate
const float ACCEL_WINDOW_S = 1.0;    // slope window; 0.6-1.5 s is a sensible range
const float ACCEL_ON       = 0.06;   // m/s^2 to enter accel/decel state (~0.12 kn/s)
const float ACCEL_OFF      = 0.03;   // m/s^2 to fall back to "steady" (hysteresis)
const float ACCEL_FULL     = 0.8;    // m/s^2 at which beep rate is fastest

// Pitch for speed (Hz). Decel tone sits 100 Hz lower, as in v1.
const int FREQ_MIN       = 500;
const int FREQ_MAX       = 2000;
const int DECEL_OFFSET   = 100;

// Behaviour outside the configured range
const float BELOW_LIMIT_SILENCE_MARGIN = 0.0;  // m/s below lower_limit before silence
const bool  STEADY_IS_SILENT = true;           // v1 behaviour: silent inside deadband

// Beeps (accelerating)
const unsigned long BEEP_PERIOD_SLOW_MS = 600;  // gentle accel
const unsigned long BEEP_PERIOD_FAST_MS = 150;  // hard accel
const unsigned long BEEP_ON_MS          = 70;   // audible on bone conduction / in wind

// ---------- State ----------
const int BUF_N = 32;                    // enough for 2 s at 10 Hz
float          spdBuf[BUF_N];
unsigned long  tBuf[BUF_N];
int  bufHead = 0, bufCount = 0;

int   varioState = 0;        // +1 accel, -1 decel, 0 steady/silent
int   toneFreq   = 0;        // Hz for current state
unsigned long beepPeriod = BEEP_PERIOD_SLOW_MS;
unsigned long lastBeepStart = 0;
bool  silent = true;
int   lastWrittenFreq = -1;

// ---------- Helpers ----------
float mapFloat(float x, float inMin, float inMax, float outMin, float outMax) {
  if (x < inMin) x = inMin;
  if (x > inMax) x = inMax;
  return outMin + (x - inMin) * (outMax - outMin) / (inMax - inMin);
}

void writeTone(int f) {               // avoid re-writing LEDC every tick
  if (f == lastWrittenFreq) return;
  ledcWriteTone(LEDC_CHANNEL, f);
  lastWrittenFreq = f;
}

// Least-squares slope (m/s^2) and mean speed over the last windowS seconds
bool slopeAndMean(float windowS, float &slope, float &mean) {
  if (bufCount < 3) return false;
  unsigned long tNow = tBuf[(bufHead - 1 + BUF_N) % BUF_N];
  float st = 0, sv = 0, stt = 0, stv = 0; int n = 0;
  for (int k = 0; k < bufCount; k++) {
    int i = (bufHead - 1 - k + BUF_N) % BUF_N;
    float t = -(float)(tNow - tBuf[i]) / 1000.0;   // seconds, <= 0
    if (t < -windowS) break;
    st += t; sv += spdBuf[i]; stt += t * t; stv += t * spdBuf[i]; n++;
  }
  if (n < 3) return false;
  float den = n * stt - st * st;
  if (den <= 0) return false;
  slope = (n * stv - st * sv) / den;
  mean  = sv / n;
  return true;
}

// ---------- Call on every GPS fix (raw speed, m/s) ----------
void varioNewSpeed(float speed, float lower_limit, float upper_limit) {
  if (upper_limit < lower_limit + 1) upper_limit = lower_limit + 1;

  spdBuf[bufHead] = speed; tBuf[bufHead] = millis();
  bufHead = (bufHead + 1) % BUF_N;
  if (bufCount < BUF_N) bufCount++;

  float a, v;
  if (!slopeAndMean(ACCEL_WINDOW_S, a, v)) { silent = true; return; }

  // Hysteresis: enter a state above ACCEL_ON, leave it below ACCEL_OFF
  if (varioState == 0) {
    if (a >  ACCEL_ON) varioState = +1;
    else if (a < -ACCEL_ON) varioState = -1;
  } else if (varioState == +1) {
    if (a < -ACCEL_ON) varioState = -1;
    else if (a < ACCEL_OFF) varioState = 0;
  } else {
    if (a > ACCEL_ON) varioState = +1;
    else if (a > -ACCEL_OFF) varioState = 0;
  }

  int f = (int)mapFloat(v, lower_limit, upper_limit, FREQ_MIN, FREQ_MAX);
  toneFreq   = (varioState == -1) ? f - DECEL_OFFSET : f;
  beepPeriod = (unsigned long)mapFloat(a, ACCEL_ON, ACCEL_FULL,
                                       BEEP_PERIOD_SLOW_MS, BEEP_PERIOD_FAST_MS);

  silent = (v < lower_limit - BELOW_LIMIT_SILENCE_MARGIN) ||
           (STEADY_IS_SILENT && varioState == 0);
}

// ---------- Call from loop() as often as possible ----------
void varioTick() {
  if (silent) { writeTone(0); return; }
  if (varioState == -1) { writeTone(toneFreq); return; }   // continuous decel tone
  if (varioState == 0)  { writeTone(toneFreq); return; }   // only if STEADY_IS_SILENT=false

  unsigned long now = millis();                            // accelerating: beeps
  if (now - lastBeepStart >= beepPeriod) lastBeepStart = now;
  writeTone((now - lastBeepStart < BEEP_ON_MS) ? toneFreq : 0);
}

#endif
