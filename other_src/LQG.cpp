#include <Arduino.h>
#include <SPIFFS.h>

// =========================================================
//  ESP32 Temperature Station Controller (LQG-Servo + SPIFFS Logging)
//  ---------------------------------------------------------
//  This is the "full" version matching your previous I-PD project style:
//    - Same sampling scheme (0.5s), moving average buffer (last 10 samples)
//    - Same SPIFFS logging workflow (data.csv + commands to print/clear/info)
//    - Same pins (ADC=GPIO34, DAC=GPIO25) by default
//
//  Controller implemented: discrete-time LQG servo (Kalman estimator + LQI)
//  Parameters are taken from your MATLAB export (lqg_params.mat).
//
//  IMPORTANT ABOUT UNITS:
//    Your identified model is in TEMPERATURE units (ΔT / Δu), so the LQG
//    tracks a temperature reference Tref (°C). The ADC voltage is converted
//    to temperature using the same formula as MATLAB.
//
//  Measurement path:
//    ADC reads divided sensor voltage Vadc (≈0.67..3.3V)
//    Temperature conversion (match MATLAB):
//      T = TEMP_A * ( G_div * (Vadc + V_SENSOR_OFFSET) ) + TEMP_B
//
//  Actuator path:
//    We compute station input command u_station (1..5V), then map to DAC (0..3.3V)
//    assuming your amplifier mapping is linear:
//      u_station = 1 + (4/3.3)*u_dac
//      u_dac     = (u_station - 1) / (4/3.3)
//
//  Deviation-model control around a bias:
//    dT_meas = T_meas - T_baseC
//    xhat[k+1] = A*xhat + B*du + L*(dT_meas - (C*xhat + D*du))
//    du_unsat  = -Kx*xhat - Ki*xi
//    u_station_unsat = U_station_bias + du_unsat
//    u_station_cmd = sat(u_station_unsat, 1..5)
//    du_cmd = u_station_cmd - U_station_bias
//    xi[k+1] = xi[k] + Ts*( e + (du_cmd - du_unsat)/Taw )   (anti-windup)
//
//  Serial commands (same style as your I-PD):
//    'e' : Enable/disable LQG control (bumpless init)
//    'r' : Reset states, disable control, output = bias
//    '+' : Increase Tref by +0.5 °C
//    '-' : Decrease Tref by -0.5 °C
//    's' : Toggle logging to SPIFFS
//    'p' : Print file contents
//    'c' : Clear data file
//    'i' : File info
//    'v' : Print current values
//    'h' : Help
// =========================================================


// ==================== CONFIGURATION ====================

// Base sampling interval for ADC reads / logging (ms)
const unsigned long SAMPLE_INTERVAL_MS = 500;   // from MATLAB Ts

// Moving average window length (last 10 samples, spaced by SAMPLE_INTERVAL_MS)
const int AVG_WINDOW = 10;

// LQG update rate:
const int LQG_UPDATE_EVERY_N_SAMPLES = 1;

// ADC input pin (sensor voltage AFTER divider)
const int ADC_PIN = 34;

// DAC output pin (setpoint output)
const int DAC_PIN = 25;

// DAC output voltage limits (ESP32 side)
const float U_DAC_MIN_V = 0.0f;
const float U_DAC_MAX_V = 3.3f;

// ---------------- Station actuator limits (after amplifier) ----------------
const float U_STATION_MIN_V = 1.0f;
const float U_STATION_MAX_V = 5.0f;

// Amplifier mapping (assumed linear):
//   u_station = U_STATION_MIN + ( (U_STATION_MAX-U_STATION_MIN) / 3.3 ) * u_dac
const float GAIN_DAC2STATION = (U_STATION_MAX_V - U_STATION_MIN_V) / (U_DAC_MAX_V - U_DAC_MIN_V); // 4/3.3
const float OFFS_DAC2STATION = U_STATION_MIN_V;

// Operating point bias (station input volts) from MATLAB export
float U_station_bias = 1.05f;

// ---------------- Measurement clamp (ADC node volts) ----------------
const float V_ADC_MIN = 0.67f;
const float V_ADC_MAX = 3.30f;

// ---------------- Temperature conversion constants (match MATLAB) ----------------
// T = TEMP_A * ( G_div * (Vadc + V_SENSOR_OFFSET) ) + TEMP_B
const float TEMP_A = 36.0f;
const float TEMP_B = -24.315f;
const float G_div  = 1.5f;
const float V_SENSOR_OFFSET = 0.15f;

// (Optional) identification offset used in MATLAB (not needed directly in deviation form)
const float U_OFFSET_V = 0.1f;

// ---------------- LQG parameters (from MATLAB export) ----------------
// nx = 1
const float A = 0.9996456815f;
const float B = 0.06248892689f;
const float C = 0.09839974458f;
const float D = 0.0f;

const float Kx = 1.491179127f;
const float Ki = -1.358196596f;
const float L  = 0.04885791904f;

// ---------------- Practical knobs ----------------
const float ERROR_DEADBAND_C = 0.05f;
const float Taw = 60.0f; // anti-windup back-calculation time constant (seconds)

// Optional station-command rate limit (helps TRIAC / reduces jerk)
const bool  USE_RATE_LIMIT = true;
const float RATE_LIMIT_V_PER_S = 0.30f; // station V/s

// Reference adjust step
const float TREF_STEP_C = 75.58f;

// ---------------- Logging ----------------
const char* DATA_FILE = "/data.csv";
const size_t MAX_FILE_SIZE = 1000000;


// ==================== GLOBAL STATE ====================
unsigned long lastSampleTime = 0;
unsigned long sampleCount = 0;
bool loggingEnabled = true;
bool lqgEnabled = false;

// Moving average buffer (raw ADC counts)
int adcBuf[AVG_WINDOW] = {0};
int adcIdx = 0;
long adcSum = 0;
bool adcFilled = false;

// Controller states
float xhat = 0.0f;
float xi   = 0.0f;
float du   = 0.0f;  // deviation command in station volts

// Baseline temperature (renamed to avoid ESP32 core symbol conflict with T0)
float T_baseC = 25.3434f;

// Reference temperature (C)
float TrefC = 25.3434f;

// Current DAC output voltage (ESP32 side)
float uDacV = 0.0f;

// Rate limit per control step
float duRateMaxPerStep = 0.0f;


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
  float vAdc = adcToVoltage((int)(avgAdc + 0.5f));

  // clamp to plausible sensor range
  vAdc = clampf(vAdc, V_ADC_MIN, V_ADC_MAX);
  return vAdc;
}

float adcToTempC(float vAdc) {
  return TEMP_A * (G_div * (vAdc + V_SENSOR_OFFSET)) + TEMP_B;
}

// Useful to log: the "station-equivalent" voltage before divider/offset used in MATLAB
float adcToStationEquivalentV(float vAdc) {
  return G_div * (vAdc + V_SENSOR_OFFSET);
}

void setDacVoltage(float voltage) {
  voltage = clampf(voltage, U_DAC_MIN_V, U_DAC_MAX_V);
  uDacV = voltage;

  // ESP32 DAC: 8-bit (0-255) for ~0-3.3V
  int dacValue = (int)((voltage / 3.3f) * 255.0f + 0.5f);
  dacWrite(DAC_PIN, dacValue);
}

float stationToDacV(float uStation) {
  // u_station = OFFS + GAIN*u_dac  => u_dac = (u_station - OFFS)/GAIN
  return (uStation - OFFS_DAC2STATION) / GAIN_DAC2STATION;
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
      file.println("timestamp_ms,Tref_C,Tmeas_C,That_C,Vadc_avg,Vstation_eq,u_station,u_dac,du,xhat,xi,sat");
      file.close();
      Serial.println("Created new data file with header");
    }
  }
}

void clearDataFile() {
  File file = SPIFFS.open(DATA_FILE, FILE_WRITE);
  if (file) {
    file.println("timestamp_ms,Tref_C,Tmeas_C,That_C,Vadc_avg,Vstation_eq,u_station,u_dac,du,xhat,xi,sat");
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

bool logLqgRow(unsigned long timestampMs,
               float Tref, float Tmeas, float That,
               float vAdcAvg, float vStationEq,
               float uStation, float uDac,
               float duCmd, float xhatVal, float xiVal,
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

  f.printf("%lu,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.4f,%.6f,%.6f,%.6f,%d\n",
           timestampMs, Tref, Tmeas, That, vAdcAvg, vStationEq, uStation, uDac, duCmd, xhatVal, xiVal, satFlag);
  f.close();
  return true;
}


// ==================== LQG CONTROL ====================
void initLqgBumpless(float TmeasNow) {
  // Reset deviation model around current temperature
  T_baseC = TmeasNow;
  TrefC = TmeasNow;

  xhat = 0.0f;
  xi   = 0.0f;
  du   = 0.0f;

  // Output bias
  float uStation = clampf(U_station_bias, U_STATION_MIN_V, U_STATION_MAX_V);
  float uDac = stationToDacV(uStation);
  setDacVoltage(uDac);
}

float lqgStep(float Tref, float Tmeas, float TsCtrl,
              float& That,
              float& uStationCmd,
              int& satFlag) {
  // deviation measurement
  float dT_meas = Tmeas - T_baseC;

  // observer
  float yhat = C * xhat + D * du;
  xhat = A * xhat + B * du + L * (dT_meas - yhat);

  // estimate
  float dT_hat = C * xhat + D * du;
  That = T_baseC + dT_hat;

  // error (integrate on estimate)
  float e = Tref - That;
  if (fabsf(e) < ERROR_DEADBAND_C) e = 0.0f;

  // control (unsat deviation)
  float du_unsat = -Kx * xhat - Ki * xi;

  // absolute station command and saturation
  float u_unsat = U_station_bias + du_unsat;
  float u_sat = clampf(u_unsat, U_STATION_MIN_V, U_STATION_MAX_V);
  satFlag = (u_sat != u_unsat) ? 1 : 0;

  float du_sat = u_sat - U_station_bias;

  // rate limit (in deviation volts)
  float du_cmd = du_sat;
  if (USE_RATE_LIMIT) {
    float ddu = du_cmd - du;
    ddu = clampf(ddu, -duRateMaxPerStep, duRateMaxPerStep);
    du_cmd = du + ddu;
    // recompute u from rate-limited du
    u_sat = clampf(U_station_bias + du_cmd, U_STATION_MIN_V, U_STATION_MAX_V);
  }

  // anti-windup back-calculation
  xi += TsCtrl * (e + (du_cmd - du_unsat) / Taw);

  // commit
  du = du_cmd;
  uStationCmd = u_sat;
  return du_cmd;
}


// ==================== UI ====================
void printHelp() {
  Serial.println("\n----------------------------------------");
  Serial.println("Commands:");
  Serial.println("  'e' - Enable/Disable LQG control (bumpless)");
  Serial.println("  'r' - Reset (disable LQG, output=bias, reset states)");
  Serial.println("  '+' - Increase Tref by 0.5 C");
  Serial.println("  '-' - Decrease Tref by 0.5 C");
  Serial.println("  's' - Toggle logging to SPIFFS");
  Serial.println("  'p' - Print file contents");
  Serial.println("  'c' - Clear data file");
  Serial.println("  'i' - Show file info");
  Serial.println("  'v' - Show current values");
  Serial.println("  'h' - Help");
  Serial.println("----------------------------------------");
}

void printCurrentValues() {
  float vAdc = readAdcVoltageAveraged();
  float tMeas = adcToTempC(vAdc);
  float vStEq = adcToStationEquivalentV(vAdc);

  Serial.printf("\nLQG: %s | Logging: %s\n", lqgEnabled ? "ON" : "OFF", loggingEnabled ? "ON" : "OFF");
  Serial.printf("TrefC:        %.3f C\n", TrefC);
  Serial.printf("Tmeas:        %.3f C\n", tMeas);
  Serial.printf("T_baseC:      %.3f C (baseline)\n", T_baseC);
  Serial.printf("VadcAvg:      %.4f V\n", vAdc);
  Serial.printf("VstationEq:   %.4f V\n", vStEq);
  Serial.printf("Bias u_station=%.4f V (station)\n", U_station_bias);
  Serial.printf("uDacV:        %.4f V (DAC)\n", uDacV);
  Serial.printf("xhat:         %.6f\n", xhat);
  Serial.printf("xi:           %.6f\n", xi);
  Serial.printf("du:           %.6f (station V deviation)\n\n", du);
}


// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  Serial.println("\n========================================");
  Serial.println("   ESP32 Temperature Station LQG (LQG-Servo)");
  Serial.println("   SPIFFS logging compatible with your I-PD project");
  Serial.println("========================================");

  initSPIFFS();

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  pinMode(ADC_PIN, INPUT);

  // Start output at bias
  float uStation0 = clampf(U_station_bias, U_STATION_MIN_V, U_STATION_MAX_V);
  setDacVoltage(stationToDacV(uStation0));

  // Prime the moving average
  for (int k = 0; k < AVG_WINDOW; k++) {
    delay(5);
    readAdcVoltageAveraged();
  }

  // Rate limit per step based on control period
  float TsCtrl = (LQG_UPDATE_EVERY_N_SAMPLES * SAMPLE_INTERVAL_MS) / 1000.0f;
  duRateMaxPerStep = RATE_LIMIT_V_PER_S * TsCtrl;

  // Bumpless init around current measurement
  float vAdc0 = readAdcVoltageAveraged();
  float tMeas0 = adcToTempC(vAdc0);
  initLqgBumpless(tMeas0);

  Serial.printf("\nHardware:\n");
  Serial.printf("  ADC: GPIO %d (measures divided voltage)\n", ADC_PIN);
  Serial.printf("  DAC: GPIO %d (ESP32 DAC)\n", DAC_PIN);
  Serial.printf("  Sample interval: %lu ms\n", SAMPLE_INTERVAL_MS);
  Serial.printf("  Avg window: %d samples (%.1f s)\n",
                AVG_WINDOW, (AVG_WINDOW * SAMPLE_INTERVAL_MS) / 1000.0f);
  Serial.printf("  LQG update every: %d samples (Ts_ctrl = %.3f s)\n",
                LQG_UPDATE_EVERY_N_SAMPLES, TsCtrl);

  Serial.printf("\nLQG params (nx=1):\n");
  Serial.printf("  A=%.9f B=%.9f C=%.9f D=%.9f\n", A, B, C, D);
  Serial.printf("  Kx=%.6f Ki=%.6f L=%.6f\n", Kx, Ki, L);
  Serial.printf("  Bias u_station=%.4f V (limits %.1f..%.1f V)\n", U_station_bias, U_STATION_MIN_V, U_STATION_MAX_V);

  Serial.printf("\nTemp conversion:\n");
  Serial.printf("  T = %.3f*(%.3f*(Vadc + %.3f)) + %.3f\n", TEMP_A, G_div, V_SENSOR_OFFSET, TEMP_B);
  Serial.printf("  Initial T_baseC=%.3f C\n", T_baseC);

  printHelp();
  printFileInfo();

  Serial.println("\n>>> LQG is DISABLED. Press 'e' to enable closed-loop control. <<<\n");
}


// ==================== LOOP ====================
void loop() {
  // --- Serial commands ---
  if (Serial.available()) {
    char cmd = Serial.read();
    switch (cmd) {
      case 'e':
      case 'E': {
        lqgEnabled = !lqgEnabled;

        float vAdcNow = readAdcVoltageAveraged();
        float tMeasNow = adcToTempC(vAdcNow);

        if (lqgEnabled) {
          initLqgBumpless(tMeasNow);
          Serial.println("LQG ENABLED (bumpless: T_baseC=Tmeas, xhat/xi/du reset, output=bias)");
        } else {
          Serial.println("LQG DISABLED (output holds last DAC value)");
        }
        break;
      }
      case 'r':
      case 'R': {
        lqgEnabled = false;

        float vAdcNow = readAdcVoltageAveraged();
        float tMeasNow = adcToTempC(vAdcNow);
        initLqgBumpless(tMeasNow);

        Serial.println("RESET: LQG disabled, output=bias, states reset.");
        break;
      }
      case '+': {
        TrefC += TREF_STEP_C;
        Serial.printf("TrefC = %.2f C\n", TrefC);
        break;
      }
      case '-': {
        TrefC -= TREF_STEP_C;
        Serial.printf("TrefC = %.2f C\n", TrefC);
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
      case 'V':
        printCurrentValues();
        break;

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
    float vAdcAvg = readAdcVoltageAveraged();
    float tMeas = adcToTempC(vAdcAvg);
    float vStationEq = adcToStationEquivalentV(vAdcAvg);

    // Update LQG at configured rate
    if (lqgEnabled && (sampleCount % LQG_UPDATE_EVERY_N_SAMPLES == 0)) {
      float TsCtrl = (LQG_UPDATE_EVERY_N_SAMPLES * SAMPLE_INTERVAL_MS) / 1000.0f;

      float That = tMeas;
      float uStationCmd = U_station_bias;
      int sat = 0;

      float duCmd = lqgStep(TrefC, tMeas, TsCtrl, That, uStationCmd, sat);

      // Output
      float uDac = stationToDacV(uStationCmd);
      setDacVoltage(uDac);

      // Logging (only when LQG updates)
      if (loggingEnabled) {
        logLqgRow(now, TrefC, tMeas, That,
                  vAdcAvg, vStationEq,
                  uStationCmd, uDac,
                  duCmd, xhat, xi, sat);
      }

      // Serial print every few updates
      static unsigned long lqgPrintCount = 0;
      lqgPrintCount++;
      if (lqgPrintCount % 5 == 0) {
        Serial.printf("t=%lu ms | Tref=%.2f T=%.2f That=%.2f | Vadc=%.3f | u_st=%.3f u_dac=%.3f | xhat=%.4f xi=%.4f %s\n",
                      now, TrefC, tMeas, That, vAdcAvg, uStationCmd, uDac, xhat, xi, sat ? "(SAT)" : "");
      }
    }
  }
}
