#include <Arduino.h>
#include <SPIFFS.h>

// =========================================================
//  ESP32 Temperature Station PID Controller + Data Logger
//  - DAC output (0..3.3V) drives your 0-3.3V -> 4-20mA module -> station input
//  - ADC reads station sensor voltage (0..3.3V)
//
//  Added:
//   1) Discrete-time PID (s-domain params implemented in sampled form)
//   2) Band-limited (filtered) derivative using N
//   3) Averaged ADC readings (moving average of last 10 samples, sampled every 0.5 s)
//   4) Anti-windup + output saturation handling
// =========================================================


// ==================== CONFIGURATION ====================

// Base sampling interval for ADC reads / logging (ms)
const unsigned long SAMPLE_INTERVAL_MS = 500;   // 0.5 s

// Moving average window length (10 readings, spaced by SAMPLE_INTERVAL_MS)
const int AVG_WINDOW = 10;

// PID update rate:
//  - 1  => update PID every 0.5 s using the moving-average measurement
//  - 10 => update PID every 5.0 s (often more appropriate for thermal systems)
const int PID_UPDATE_EVERY_N_SAMPLES = 1;

// ADC input pin (sensor voltage from station)
const int ADC_PIN = 34;

// DAC output pin (setpoint to station via your V->I module)
const int DAC_PIN = 25;

// Output saturation limits (DAC range)
const float U_MIN_V = 0.0f;
const float U_MAX_V = 3.3f;

// ---------------- PID Parameters (CONTINUOUS / s-domain) ----------------
//
// If your designed PID is in the common form:
//
//   C(s) = Kp * ( 1 + 1/(Ti*s) + (Td*s)/(1 + (Td/N)*s) )
//
// where:
//  - Kp: proportional gain
//  - Ti: integral time constant (seconds)
//  - Td: derivative time constant (seconds)
//  - N : derivative filter coefficient (dimensionless)
//
// Put your values here:
float Kp = 1.0f;
float Ti = 100.0f;     // seconds  (avoid Ti=0)
float Td = 0.0f;       // seconds  (Td=0 => PI controller)
float N  = 10.0f;      // typical 5..50

// NOTE: If you instead designed in parallel form Kp + Ki/s + Kd*s/(1 + s/N),
// you can set Ti = Kp/Ki and Td = Kd/Kp (when Kp>0).

// Reference (desired output) in *sensor-voltage units* (0..3.3 V).
// You can adjust at runtime via serial '+' / '-' commands.
float refSensorV = 0.35f;

// ---------------- Logging ----------------
const char* DATA_FILE = "/data.csv";
const size_t MAX_FILE_SIZE = 1000000;


// ==================== GLOBAL STATE ====================
unsigned long lastSampleTime = 0;
unsigned long sampleCount = 0;
bool loggingEnabled = true;
bool pidEnabled = false;

// Moving average buffer
int adcBuf[AVG_WINDOW] = {0};
int adcIdx = 0;
long adcSum = 0;
bool adcFilled = false;

// PID states
float iState = 0.0f;         // integrator contribution (already includes Kp/Ti)
float prevError = 0.0f;
float prevY = 0.0f;
float dFilt = 0.0f;          // filtered derivative of measurement (V/s)

// Current DAC output
float uOutV = 0.0f;


// ==================== HELPERS ====================
static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

float adcToVoltage(int adcValue) {
  // ESP32 ADC: 12-bit (0-4095), ~3.3V reference
  return (adcValue / 4095.0f) * 3.3f;
}

void setDacVoltage(float voltage) {
  voltage = clampf(voltage, U_MIN_V, U_MAX_V);
  uOutV = voltage;

  // ESP32 DAC: 8-bit (0-255) for ~0-3.3V
  int dacValue = (int)((voltage / 3.3f) * 255.0f + 0.5f);
  dacWrite(DAC_PIN, dacValue);
}

void resetPidStates(float currentY) {
  iState = 0.0f;
  prevError = 0.0f;
  prevY = currentY;
  dFilt = 0.0f;
}

// Moving average update: read ADC once, update buffer, return average voltage
float readSensorVoltageAveraged() {
  int raw = analogRead(ADC_PIN);

  // Update ring buffer sum
  if (!adcFilled) {
    // filling phase
    adcBuf[adcIdx] = raw;
    adcSum += raw;
    adcIdx++;
    if (adcIdx >= AVG_WINDOW) {
      adcIdx = 0;
      adcFilled = true;
    }
  } else {
    // steady phase
    adcSum -= adcBuf[adcIdx];
    adcBuf[adcIdx] = raw;
    adcSum += raw;
    adcIdx++;
    if (adcIdx >= AVG_WINDOW) adcIdx = 0;
  }

  int denom = adcFilled ? AVG_WINDOW : max(1, adcIdx);
  float avgAdc = (float)adcSum / (float)denom;
  return adcToVoltage((int)(avgAdc + 0.5f));
}


// ==================== SPIFFS / LOGGING ====================
void initSPIFFS() {
  if (!SPIFFS.begin(true)) {
    Serial.println("ERROR: SPIFFS mount failed!");
    return;
  }
  Serial.println("SPIFFS mounted successfully");

  if (!SPIFFS.exists(DATA_FILE)) {
    File file = SPIFFS.open(DATA_FILE, FILE_WRITE);
    if (file) {
      file.println("timestamp_ms,ref_v,u_v,sensor_v_avg,error,p_term,i_term,d_term,sat");
      file.close();
      Serial.println("Created new data file with header");
    }
  }
}

void clearDataFile() {
  File file = SPIFFS.open(DATA_FILE, FILE_WRITE);
  if (file) {
    file.println("timestamp_ms,ref_v,u_v,sensor_v_avg,error,p_term,i_term,d_term,sat");
    file.close();
    sampleCount = 0;
    Serial.println("Data file cleared");
  } else {
    Serial.println("ERROR: Could not clear file");
  }
}

void printFileContents() {
  Serial.println("\n========== FILE CONTENTS ==========");
  File file = SPIFFS.open(DATA_FILE, FILE_READ);
  if (file) {
    while (file.available()) {
      Serial.write(file.read());
    }
    file.close();
  } else {
    Serial.println("ERROR: Could not open file for reading");
  }
  Serial.println("====================================\n");
}

void printFileInfo() {
  Serial.println("\n---------- FILE INFO ----------");
  size_t totalBytes = SPIFFS.totalBytes();
  size_t usedBytes = SPIFFS.usedBytes();

  Serial.printf("SPIFFS Total: %u bytes\n", (unsigned)totalBytes);
  Serial.printf("SPIFFS Used:  %u bytes\n", (unsigned)usedBytes);
  Serial.printf("SPIFFS Free:  %u bytes\n", (unsigned)(totalBytes - usedBytes));

  File file = SPIFFS.open(DATA_FILE, FILE_READ);
  if (file) {
    Serial.printf("Data file size: %u bytes\n", (unsigned)file.size());

    size_t lines = 0;
    while (file.available()) {
      if (file.read() == '\n') lines++;
    }
    Serial.printf("Total samples: %u\n", (unsigned)(lines > 0 ? lines - 1 : 0));
    file.close();
  }
  Serial.println("-------------------------------\n");
}

bool logPidRow(unsigned long timestampMs,
               float refV, float uV, float yAvgV,
               float e, float pTerm, float iTerm, float dTerm,
               int satFlag) {
  // Check file size
  File f = SPIFFS.open(DATA_FILE, FILE_READ);
  if (f) {
    size_t sz = f.size();
    f.close();
    if (sz >= MAX_FILE_SIZE) {
      Serial.println("WARNING: Max file size reached. Stopping logging.");
      loggingEnabled = false;
      return false;
    }
  }

  f = SPIFFS.open(DATA_FILE, FILE_APPEND);
  if (!f) {
    Serial.println("ERROR: Could not open file for writing");
    return false;
  }

  f.printf("%lu,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f,%.5f,%d\n",
           timestampMs, refV, uV, yAvgV, e, pTerm, iTerm, dTerm, satFlag);
  f.close();
  return true;
}


// ==================== PID (DISCRETE IMPLEMENTATION) ====================
//
// Discrete realization using:
//  - Trapezoidal (Tustin) integration for I
//  - First-order low-pass filtered derivative on measurement for D
//  - Conditional integration anti-windup (clamp + freeze when pushing further into saturation)
//
// Notes:
//  - Using derivative on *measurement* avoids derivative kick from setpoint steps.
//  - This implements your *s-domain* PID at the chosen sample time Ts.
float pidStep(float rV, float yV, float Ts,
              float& pTerm, float& iTerm, float& dTerm,
              int& satFlag) {
  // Protect against invalid parameters
  float Ti_safe = (Ti <= 0.0f) ? 1e9f : Ti; // effectively no integral if Ti<=0
  float N_safe  = (N  <= 0.0f) ? 1e9f : N;  // effectively no D filter if N<=0

  // Convert to parallel gains
  float Ki = (Ti_safe >= 1e8f) ? 0.0f : (Kp / Ti_safe); // 1/s
  float Kd = Kp * Td;                                    // seconds*gain

  // Derivative filter time constant Tf = Td/N (standard band-limited derivative)
  float Tf = (Td <= 0.0f) ? 0.0f : (Td / N_safe);

  float e = rV - yV;

  // --- P ---
  pTerm = Kp * e;

  // --- D (on measurement) ---
  // raw derivative of measurement
  float dy = (yV - prevY);
  float deriv = dy / Ts; // V/s

  if (Td <= 0.0f || Kd == 0.0f) {
    dFilt = 0.0f;
    dTerm = 0.0f;
  } else {
    // low-pass filter on derivative: dFilt = alpha*dFilt + (1-alpha)*deriv
    // alpha = Tf/(Tf+Ts)  (backward-Euler / matched pole approx)
    float alpha = Tf / (Tf + Ts);
    alpha = clampf(alpha, 0.0f, 0.9999f);
    dFilt = alpha * dFilt + (1.0f - alpha) * deriv;

    // derivative on measurement -> subtract
    dTerm = -Kd * dFilt;
  }

  // --- I (candidate update using trapezoidal integration) ---
  float iCandidate = iState + Ki * (Ts * 0.5f) * (e + prevError);
  iTerm = iCandidate;

  // --- Unsaturated output ---
  float uUnsat = pTerm + iCandidate + dTerm;
  float uSat = clampf(uUnsat, U_MIN_V, U_MAX_V);

  satFlag = (uSat != uUnsat) ? 1 : 0;

  // --- Anti-windup (conditional integration) ---
  // If saturated AND error would drive further into saturation, freeze integrator.
  if (satFlag) {
    bool pushingHigh = (uUnsat > U_MAX_V) && (e > 0.0f);
    bool pushingLow  = (uUnsat < U_MIN_V) && (e < 0.0f);

    if (pushingHigh || pushingLow) {
      // Freeze integral (reject candidate)
      iCandidate = iState;
      iTerm = iCandidate;

      // Recompute with frozen I
      uUnsat = pTerm + iCandidate + dTerm;
      uSat = clampf(uUnsat, U_MIN_V, U_MAX_V);
      satFlag = (uSat != uUnsat) ? 1 : 0;
    } else {
      // Allow integration because it helps come out of saturation
      iState = iCandidate;
    }
  } else {
    iState = iCandidate;
  }

  prevError = e;
  prevY = yV;

  // Return saturated output (what we actually apply)
  return uSat;
}


// ==================== UI ====================
void printHelp() {
  Serial.println("\n----------------------------------------");
  Serial.println("Commands:");
  Serial.println("  'e' - Enable/Disable PID control");
  Serial.println("  'r' - Reset PID states + set DAC to 0V");
  Serial.println("  '+' - Increase reference (refSensorV) by 0.01 V");
  Serial.println("  '-' - Decrease reference (refSensorV) by 0.01 V");
  Serial.println("  's' - Toggle logging to SPIFFS");
  Serial.println("  'p' - Print file contents");
  Serial.println("  'c' - Clear data file");
  Serial.println("  'i' - Show file info");
  Serial.println("  'v' - Show current values");
  Serial.println("  'h' - Help");
  Serial.println("----------------------------------------");
}


// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  Serial.println("\n========================================");
  Serial.println("   ESP32 Temperature Station PID");
  Serial.println("========================================");

  initSPIFFS();

  // ADC setup
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  pinMode(ADC_PIN, INPUT);

  // Start with 0V output
  setDacVoltage(0.0f);

  // Prime the moving average + PID states with initial measurement
  for (int k = 0; k < AVG_WINDOW; k++) {
    delay(5);
    readSensorVoltageAveraged();
  }
  float y0 = readSensorVoltageAveraged();
  resetPidStates(y0);

  Serial.printf("\nHardware:\n");
  Serial.printf("  ADC: GPIO %d\n", ADC_PIN);
  Serial.printf("  DAC: GPIO %d\n", DAC_PIN);
  Serial.printf("  Sample interval: %lu ms\n", SAMPLE_INTERVAL_MS);
  Serial.printf("  Avg window: %d samples (%.1f s)\n",
                AVG_WINDOW, (AVG_WINDOW * SAMPLE_INTERVAL_MS) / 1000.0f);
  Serial.printf("  PID update every: %d samples (Ts_pid = %.3f s)\n",
                PID_UPDATE_EVERY_N_SAMPLES,
                (PID_UPDATE_EVERY_N_SAMPLES * SAMPLE_INTERVAL_MS) / 1000.0f);

  Serial.printf("\nPID (s-domain params):\n");
  Serial.printf("  Kp=%.6f, Ti=%.3f s, Td=%.3f s, N=%.3f\n", Kp, Ti, Td, N);
  Serial.printf("Reference output (sensor voltage): refSensorV=%.3f V\n", refSensorV);

  printHelp();
  printFileInfo();

  Serial.println("\n>>> PID is DISABLED. Press 'e' to enable closed-loop control. <<<\n");
}


// ==================== LOOP ====================
void loop() {
  // --- Serial commands ---
  if (Serial.available()) {
    char cmd = Serial.read();
    switch (cmd) {
      case 'e':
      case 'E': {
        pidEnabled = !pidEnabled;
        float yNow = readSensorVoltageAveraged();
        resetPidStates(yNow);
        Serial.printf("PID %s\n", pidEnabled ? "ENABLED" : "DISABLED");
        break;
      }
      case 'r':
      case 'R': {
        pidEnabled = false;
        setDacVoltage(0.0f);
        float yNow = readSensorVoltageAveraged();
        resetPidStates(yNow);
        Serial.println("RESET: PID disabled, DAC=0V, states cleared.");
        break;
      }
      case '+': {
        refSensorV += 0.01f;
        refSensorV = clampf(refSensorV, 0.0f, 3.3f);
        Serial.printf("refSensorV = %.3f V\n", refSensorV);
        break;
      }
      case '-': {
        refSensorV -= 0.01f;
        refSensorV = clampf(refSensorV, 0.0f, 3.3f);
        Serial.printf("refSensorV = %.3f V\n", refSensorV);
        break;
      }
      case 's':
      case 'S':
        loggingEnabled = !loggingEnabled;
        Serial.printf("Logging %s\n", loggingEnabled ? "ENABLED" : "DISABLED");
        break;

      case 'p':
      case 'P':
        printFileContents();
        break;

      case 'c':
      case 'C':
        clearDataFile();
        break;

      case 'i':
      case 'I':
        printFileInfo();
        break;

      case 'v':
      case 'V': {
        float yAvg = readSensorVoltageAveraged();
        Serial.printf("\nPID: %s | Logging: %s\n", pidEnabled ? "ON" : "OFF", loggingEnabled ? "ON" : "OFF");
        Serial.printf("refSensorV: %.3f V\n", refSensorV);
        Serial.printf("sensorAvgV: %.4f V\n", yAvg);
        Serial.printf("uOutV:      %.4f V\n\n", uOutV);
        break;
      }

      case 'h':
      case 'H':
      case '?':
        printHelp();
        break;
    }
  }

  // --- Timed sampling / control / logging ---
  unsigned long now = millis();
  if (now - lastSampleTime >= SAMPLE_INTERVAL_MS) {
    lastSampleTime = now;
    sampleCount++;

    // Measurement (moving average)
    float yAvgV = readSensorVoltageAveraged();

    // Update PID at configured rate
    if (pidEnabled && (sampleCount % PID_UPDATE_EVERY_N_SAMPLES == 0)) {
      float Ts = (PID_UPDATE_EVERY_N_SAMPLES * SAMPLE_INTERVAL_MS) / 1000.0f;

      float pTerm = 0.0f, iTerm = 0.0f, dTerm = 0.0f;
      int sat = 0;

      float uCmdV = pidStep(refSensorV, yAvgV, Ts, pTerm, iTerm, dTerm, sat);
      setDacVoltage(uCmdV);

      // Logging (only when PID updates, to avoid huge files)
      if (loggingEnabled) {
        logPidRow(now, refSensorV, uOutV, yAvgV,
                  (refSensorV - yAvgV), pTerm, iState, dTerm, sat);
      }

      // Serial print every few PID updates
      static unsigned long pidPrintCount = 0;
      pidPrintCount++;
      if (pidPrintCount % 5 == 0) {
        Serial.printf("t=%lu ms | r=%.3f y=%.3f | u=%.3f | P=%.3f I=%.3f D=%.3f %s\n",
                      now, refSensorV, yAvgV, uOutV, pTerm, iState, dTerm, sat ? "(SAT)" : "");
      }
    } else {
      // Optional: you can also log raw/avg measurement at base rate if you want
      // (kept off to reduce file size)
    }
  }
}
