#include <Arduino.h>
#include <SPIFFS.h>

// =========================================================
//  ESP32 Temperature Station PID Controller (Kick-Reduced 2DOF: I-PD)
//  Choice B IMPLEMENTED:
//    - ADC reads the DIVIDED voltage (Vadc)
//    - Internally (PID, ref, error, logs) we use the REAL station output voltage
//        Vstation = Vadc / DIV_RATIO
//      where DIV_RATIO = Vadc / Vstation = Rbottom / (Rtop + Rbottom)
//
//  P and D use ONLY measured output y (station voltage), I uses error e = r - y
// =========================================================


// ==================== CONFIGURATION ====================

// Base sampling interval for ADC reads / logging (ms)
const unsigned long SAMPLE_INTERVAL_MS = 500;   // 0.5 s

// Moving average window length (10 readings, spaced by SAMPLE_INTERVAL_MS)
const int AVG_WINDOW = 10;

// PID update rate:
const int PID_UPDATE_EVERY_N_SAMPLES = 1;

// ADC input pin (sensor voltage from station AFTER divider)
const int ADC_PIN = 34;

// DAC output pin (setpoint to station via your V->I module)
const int DAC_PIN = 25;

// Output saturation limits (DAC range)
const float U_MIN_V = 0.0f;
const float U_MAX_V = 3.3f;

// 2DOF setpoint weight for proportional part:
const float SETPOINT_WEIGHT_B = 0.0f;

// ---------------- Voltage Divider (MEASUREMENT PATH) ----------------
//
// Define the divider ratio:
//   Vadc = Vstation * DIV_RATIO
//
// If your divider is exactly 2/3, keep it.
// Otherwise compute it from resistors:
//   DIV_RATIO = Rbottom / (Rtop + Rbottom)
// where Rtop is from station output to ADC node,
// and Rbottom is from ADC node to GND.
//
const float DIV_RATIO = 2.0f / 3.0f;      // <-- CHANGE to your real divider ratio
const float V_STATION_MAX = 3.3f / DIV_RATIO;  // max station voltage measurable by ADC

// ---------------- PID Parameters (CONTINUOUS / s-domain) ----------------
float Kp = 7.240081f;
float Ki = 0.004960f;
float Ti = Kp / Ki;
float Td = 0.0f;               // seconds
float N  = 10.0f;              // typical 5..50

// Reference (desired output) in *REAL station-voltage units* (BEFORE divider).
// You can adjust at runtime via serial '+' / '-' commands.
float refStationV = 1.5f;

// ---------------- Logging ----------------
const char* DATA_FILE = "/data.csv";
const size_t MAX_FILE_SIZE = 1000000;


// ==================== GLOBAL STATE ====================
unsigned long lastSampleTime = 0;
unsigned long sampleCount = 0;
bool loggingEnabled = true;
bool pidEnabled = false;

// Moving average buffer (raw ADC counts)
int adcBuf[AVG_WINDOW] = {0};
int adcIdx = 0;
long adcSum = 0;
bool adcFilled = false;

// PID states
float iState = 0.0f;         // integrator contribution (in output volts)
float prevError = 0.0f;
float prevY = 0.0f;          // previous y in STATION volts
float dFilt = 0.0f;          // filtered derivative of measurement (V/s) in STATION volts

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

static inline float adcToStationVoltage(float vAdc) {
  // Convert from divided voltage to real station output voltage
  // Vstation = Vadc / DIV_RATIO
  return (DIV_RATIO <= 0.0f) ? vAdc : (vAdc / DIV_RATIO);
}

void setDacVoltage(float voltage) {
  voltage = clampf(voltage, U_MIN_V, U_MAX_V);
  uOutV = voltage;

  // ESP32 DAC: 8-bit (0-255) for ~0-3.3V
  int dacValue = (int)((voltage / 3.3f) * 255.0f + 0.5f);
  dacWrite(DAC_PIN, dacValue);
}

// Moving average update: read ADC once, update buffer, return average ADC-node voltage (Vadc)
float readAdcVoltageAveraged() {
  int raw = analogRead(ADC_PIN);

  if (!adcFilled) {
    adcBuf[adcIdx] = raw;
    adcSum += raw;
    adcIdx++;
    if (adcIdx >= AVG_WINDOW) {
      adcIdx = 0;
      adcFilled = true;
    }
  } else {
    adcSum -= adcBuf[adcIdx];
    adcBuf[adcIdx] = raw;
    adcSum += raw;
    adcIdx++;
    if (adcIdx >= AVG_WINDOW) adcIdx = 0;
  }

  int denom = adcFilled ? AVG_WINDOW : max(1, adcIdx);
  float avgAdc = (float)adcSum / (float)denom;
  return adcToVoltage((int)(avgAdc + 0.5f)); // Vadc
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
      // Log BOTH: ADC-node voltage and real station voltage
      file.println("timestamp_ms,ref_station_v,u_v,v_adc_avg,_avg,error_station,p_term,i_term,d_term,sat");
      file.close();
      Serial.println("Created new data file with header");
    }
  }
}

void clearDataFile() {
  File file = SPIFFS.open(DATA_FILE, FILE_WRITE);
  if (file) {
    file.println("timestamp_ms,ref_station_v,u_v,v_adc_avg,v_station_avg,error_station,p_term,i_term,d_term,sat");
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
               float refStation, float uV,
               float vAdcAvg, float vStationAvg,
               float eStation, float pTerm, float iTerm, float dTerm,
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

  f.printf("%lu,%.4f,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f,%.5f,%d\n",
           timestampMs, refStation, uV, vAdcAvg, vStationAvg, eStation, pTerm, iTerm, dTerm, satFlag);
  f.close();
  return true;
}


// ==================== PID (DISCRETE IMPLEMENTATION, I-PD) ====================
float pidStepNoKick(float rV, float yV, float Ts,
                    float& pTerm, float& iTerm, float& dTerm,
                    int& satFlag) {
  float Ti_safe = (Ti <= 0.0f) ? 1e9f : Ti;
  float N_safe  = (N  <= 0.0f) ? 1e9f : N;

  float Ki = (Ti_safe >= 1e8f) ? 0.0f : (Kp / Ti_safe); // 1/s
  float Kd = Kp * Td;

  float Tf = (Td <= 0.0f) ? 0.0f : (Td / N_safe);

  float e = rV - yV;

  // P on (b*r - y)
  pTerm = Kp * (SETPOINT_WEIGHT_B * rV - yV);

  // D on measurement
  float dy = (yV - prevY);
  float deriv = dy / Ts; // V/s

  if (Td <= 0.0f || Kd == 0.0f) {
    dFilt = 0.0f;
    dTerm = 0.0f;
  } else {
    float alpha = Tf / (Tf + Ts);
    alpha = clampf(alpha, 0.0f, 0.9999f);
    dFilt = alpha * dFilt + (1.0f - alpha) * deriv;
    dTerm = -Kd * dFilt;
  }

  // I candidate (trapezoidal)
  float iCandidate = iState + Ki * (Ts * 0.5f) * (e + prevError);
  iTerm = iCandidate;

  float uUnsat = pTerm + iCandidate + dTerm;
  float uSat = clampf(uUnsat, U_MIN_V, U_MAX_V);

  satFlag = (uSat != uUnsat) ? 1 : 0;

  // Anti-windup (conditional integration)
  if (satFlag) {
    bool pushingHigh = (uUnsat > U_MAX_V) && (e > 0.0f);
    bool pushingLow  = (uUnsat < U_MIN_V) && (e < 0.0f);

    if (pushingHigh || pushingLow) {
      iCandidate = iState;
      iTerm = iCandidate;

      uUnsat = pTerm + iCandidate + dTerm;
      uSat = clampf(uUnsat, U_MIN_V, U_MAX_V);
      satFlag = (uSat != uUnsat) ? 1 : 0;
    } else {
      iState = iCandidate;
    }
  } else {
    iState = iCandidate;
  }

  prevError = e;
  prevY = yV;

  return uSat;
}

// Bumpless enable: initialize states so enabling PID doesn't jump u
void initPidBumpless(float rV, float yV, float currentU) {
  dFilt = 0.0f;
  prevY = yV;

  prevError = (rV - yV);

  float p0 = Kp * (SETPOINT_WEIGHT_B * rV - yV);
  float d0 = 0.0f;

  float i0 = currentU - p0 - d0;

  float iMin = U_MIN_V - p0 - d0;
  float iMax = U_MAX_V - p0 - d0;
  iState = clampf(i0, iMin, iMax);
}


// ==================== UI ====================
void printHelp() {
  Serial.println("\n----------------------------------------");
  Serial.println("Commands:");
  Serial.println("  'e' - Enable/Disable PID control");
  Serial.println("  'r' - Reset PID states + set DAC to 0V");
  Serial.println("  '+' - Increase reference (refStationV) by 0.01 V (STATION volts)");
  Serial.println("  '-' - Decrease reference (refStationV) by 0.01 V (STATION volts)");
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
  Serial.println("   ESP32 Temperature Station PID (I-PD)");
  Serial.println("   Choice B: divider compensated (PID uses station volts)");
  Serial.println("========================================");

  initSPIFFS();

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  pinMode(ADC_PIN, INPUT);

  setDacVoltage(0.0f);

  // Prime the moving average
  for (int k = 0; k < AVG_WINDOW; k++) {
    delay(5);
    readAdcVoltageAveraged();
  }

  // Initialize states (PID still disabled) using STATION volts
  float vAdc0 = readAdcVoltageAveraged();
  float yStation0 = adcToStationVoltage(vAdc0);
  initPidBumpless(refStationV, yStation0, uOutV);

  Serial.printf("\nHardware:\n");
  Serial.printf("  ADC: GPIO %d (measures divided voltage)\n", ADC_PIN);
  Serial.printf("  DAC: GPIO %d\n", DAC_PIN);
  Serial.printf("  Sample interval: %lu ms\n", SAMPLE_INTERVAL_MS);
  Serial.printf("  Avg window: %d samples (%.1f s)\n",
                AVG_WINDOW, (AVG_WINDOW * SAMPLE_INTERVAL_MS) / 1000.0f);
  Serial.printf("  PID update every: %d samples (Ts_pid = %.3f s)\n",
                PID_UPDATE_EVERY_N_SAMPLES,
                (PID_UPDATE_EVERY_N_SAMPLES * SAMPLE_INTERVAL_MS) / 1000.0f);

  Serial.printf("\nDivider:\n");
  Serial.printf("  DIV_RATIO (Vadc/Vstation) = %.6f\n", DIV_RATIO);
  Serial.printf("  Max measurable station V  = %.3f V\n", V_STATION_MAX);

  Serial.printf("\nPID (s-domain params):\n");
  Serial.printf("  Kp=%.6f, Ti=%.3f s, Td=%.3f s, N=%.3f\n", Kp, Ti, Td, N);
  Serial.printf("  Proportional setpoint weight b = %.3f\n", SETPOINT_WEIGHT_B);
  Serial.printf("Reference output: refStationV=%.3f V (STATION volts)\n", refStationV);

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

        float vAdcNow = readAdcVoltageAveraged();
        float yStationNow = adcToStationVoltage(vAdcNow);

        if (pidEnabled) {
          initPidBumpless(refStationV, yStationNow, uOutV);
          Serial.println("PID ENABLED (bumpless, kick-reduced I-PD) [station volts]");
        } else {
          Serial.println("PID DISABLED");
        }
        break;
      }
      case 'r':
      case 'R': {
        pidEnabled = false;
        setDacVoltage(0.0f);

        float vAdcNow = readAdcVoltageAveraged();
        float yStationNow = adcToStationVoltage(vAdcNow);
        initPidBumpless(refStationV, yStationNow, uOutV);

        Serial.println("RESET: PID disabled, DAC=0V, states re-initialized.");
        break;
      }
      case '+': {
        refStationV += 0.01f;
        refStationV = clampf(refStationV, 0.0f, V_STATION_MAX);
        Serial.printf("refStationV = %.3f V (station)\n", refStationV);
        break;
      }
      case '-': {
        refStationV -= 0.01f;
        refStationV = clampf(refStationV, 0.0f, V_STATION_MAX);
        Serial.printf("refStationV = %.3f V (station)\n", refStationV);
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
        float vAdc = readAdcVoltageAveraged();
        float vStation = adcToStationVoltage(vAdc);

        Serial.printf("\nPID: %s | Logging: %s\n", pidEnabled ? "ON" : "OFF", loggingEnabled ? "ON" : "OFF");
        Serial.printf("refStationV: %.3f V (station)\n", refStationV);
        Serial.printf("vAdcAvg:     %.4f V (ADC node)\n", vAdc);
        Serial.printf("vStationAvg: %.4f V (station)\n", vStation);
        Serial.printf("uOutV:       %.4f V (DAC)\n\n", uOutV);
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

    // Measurement
    float vAdcAvg = readAdcVoltageAveraged();      // measured after divider
    float yStationV = adcToStationVoltage(vAdcAvg); // compensated measurement (real station volts)

    // Update PID at configured rate
    if (pidEnabled && (sampleCount % PID_UPDATE_EVERY_N_SAMPLES == 0)) {
      float Ts = (PID_UPDATE_EVERY_N_SAMPLES * SAMPLE_INTERVAL_MS) / 1000.0f;

      float pTerm = 0.0f, iTerm = 0.0f, dTerm = 0.0f;
      int sat = 0;

      // PID uses STATION volts for r and y
      float uCmdV = pidStepNoKick(refStationV, yStationV, Ts, pTerm, iTerm, dTerm, sat);
      setDacVoltage(uCmdV);

      // Logging (only when PID updates)
      if (loggingEnabled) {
        float eStation = (refStationV - yStationV);
        logPidRow(now, refStationV, uOutV, vAdcAvg, yStationV,
                  eStation, pTerm, iState, dTerm, sat);
      }

      // Serial print every few PID updates
      static unsigned long pidPrintCount = 0;
      pidPrintCount++;
      if (pidPrintCount % 5 == 0) {
        Serial.printf("t=%lu ms | r=%.3f(st) y=%.3f(st) [adc=%.3f] | u=%.3f | P=%.3f I=%.3f D=%.3f %s\n",
                      now, refStationV, yStationV, vAdcAvg, uOutV, pTerm, iState, dTerm, sat ? "(SAT)" : "");
      }
    }
  }
}
