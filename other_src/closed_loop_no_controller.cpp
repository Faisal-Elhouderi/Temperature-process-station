#include <Arduino.h>
#include <SPIFFS.h>

// =========================================================
//  ESP32 Temperature Station: UNITY CLOSED LOOP (C = 1)
//  Kp = 1, Ki = 0, Kd = 0
//
//  Control law (using ESP32-scaled volts):
//    e = r - y
//    u = clamp(u_bias + e, 0..3.3)
//
//  Notes:
//  - r and y are in 0..3.3 V (ADC-scaled sensor voltage).
//  - u is 0..3.3 V (DAC output).
//  - u_bias is optional feedforward to place operating point.
// =========================================================

// ---------------- Sampling ----------------
const unsigned long SAMPLE_INTERVAL_MS = 500;   // 0.5 s
const int AVG_WINDOW = 10;
const int UPDATE_EVERY_N_SAMPLES = 1;           // update control every N samples

// ---------------- Pins ----------------
const int ADC_PIN = 34;
const int DAC_PIN = 25;

// ---------------- Output limits ----------------
const float U_MIN_V = 0.0f;
const float U_MAX_V = 3.3f;

// ---------------- Closed-loop settings ----------------
float refSensorV = 3.0f;   // reference in scaled sensor volts (0..3.3)
float uBiasV     = 0.00f;   // optional feedforward bias (0..3.3). Set to 0 for pure u=e.

// ---------------- Logging ----------------
const char* DATA_FILE = "/data.csv";
const size_t MAX_FILE_SIZE = 1000000;
bool loggingEnabled = true;

// ---------------- State ----------------
unsigned long lastSampleTime = 0;
unsigned long sampleCount = 0;
bool loopEnabled = false;

int adcBuf[AVG_WINDOW] = {0};
int adcIdx = 0;
long adcSum = 0;
bool adcFilled = false;

float uOutV = 0.0f;

// ---------------- Helpers ----------------
static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

float adcToVoltage(int adcValue) {
  return (adcValue / 4095.0f) * 3.3f;
}

void setDacVoltage(float voltage) {
  voltage = clampf(voltage, U_MIN_V, U_MAX_V);
  uOutV = voltage;

  int dacValue = (int)((voltage / 3.3f) * 255.0f + 0.5f);
  dacWrite(DAC_PIN, dacValue);
}

float readSensorVoltageAveraged() {
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
  return adcToVoltage((int)(avgAdc + 0.5f));
}

// ---------------- SPIFFS / Logging ----------------
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
    while (file.available()) Serial.write(file.read());
    file.close();
  } else {
    Serial.println("ERROR: Could not open file for reading");
  }
  Serial.println("====================================\n");
}

bool logRow(unsigned long timestampMs,
            float refV, float uV, float yAvgV,
            float e, float pTerm, float iTerm, float dTerm,
            int satFlag) {
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

void printHelp() {
  Serial.println("\n----------------------------------------");
  Serial.println("Commands:");
  Serial.println("  'e' - Enable/Disable unity closed-loop");
  Serial.println("  'r' - Reset (loop off + DAC=0V)");
  Serial.println("  '+' - Increase refSensorV by 0.01 V");
  Serial.println("  '-' - Decrease refSensorV by 0.01 V");
  Serial.println("  ']' - Increase uBiasV by 0.01 V");
  Serial.println("  '[' - Decrease uBiasV by 0.01 V");
  Serial.println("  's' - Toggle logging to SPIFFS");
  Serial.println("  'p' - Print file contents");
  Serial.println("  'c' - Clear data file");
  Serial.println("  'v' - Show current values");
  Serial.println("  'h' - Help");
  Serial.println("----------------------------------------");
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  Serial.println("\n========================================");
  Serial.println("   ESP32 Unity Closed-Loop (Kp=1)");
  Serial.println("========================================");

  initSPIFFS();

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  pinMode(ADC_PIN, INPUT);

  setDacVoltage(0.0f);

  // Prime the moving average
  for (int k = 0; k < AVG_WINDOW; k++) {
    delay(5);
    readSensorVoltageAveraged();
  }

  Serial.printf("ADC: GPIO %d | DAC: GPIO %d\n", ADC_PIN, DAC_PIN);
  Serial.printf("Sample interval: %lu ms | Avg window: %d\n", SAMPLE_INTERVAL_MS, AVG_WINDOW);
  Serial.printf("Update every: %d samples\n", UPDATE_EVERY_N_SAMPLES);
  Serial.printf("refSensorV=%.3f V | uBiasV=%.3f V\n", refSensorV, uBiasV);

  printHelp();
  Serial.println("\n>>> Loop is DISABLED. Press 'e' to enable unity closed-loop. <<<\n");
}

void loop() {
  // --- Serial commands ---
  if (Serial.available()) {
    char cmd = Serial.read();
    switch (cmd) {
      case 'e':
      case 'E':
        loopEnabled = !loopEnabled;
        Serial.printf("Unity closed-loop %s\n", loopEnabled ? "ENABLED" : "DISABLED");
        break;

      case 'r':
      case 'R':
        loopEnabled = false;
        setDacVoltage(0.0f);
        Serial.println("RESET: loop disabled, DAC=0V.");
        break;

      case '+':
        refSensorV = clampf(refSensorV + 0.01f, 0.0f, 3.3f);
        Serial.printf("refSensorV = %.3f V\n", refSensorV);
        break;

      case '-':
        refSensorV = clampf(refSensorV - 0.01f, 0.0f, 3.3f);
        Serial.printf("refSensorV = %.3f V\n", refSensorV);
        break;

      case ']':
        uBiasV = clampf(uBiasV + 0.01f, 0.0f, 3.3f);
        Serial.printf("uBiasV = %.3f V\n", uBiasV);
        break;

      case '[':
        uBiasV = clampf(uBiasV - 0.01f, 0.0f, 3.3f);
        Serial.printf("uBiasV = %.3f V\n", uBiasV);
        break;

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

      case 'v':
      case 'V': {
        float yAvg = readSensorVoltageAveraged();
        Serial.printf("\nLoop: %s | Logging: %s\n", loopEnabled ? "ON" : "OFF", loggingEnabled ? "ON" : "OFF");
        Serial.printf("refSensorV: %.3f V\n", refSensorV);
        Serial.printf("sensorAvgV: %.4f V\n", yAvg);
        Serial.printf("uBiasV:     %.4f V\n", uBiasV);
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

    float yAvgV = readSensorVoltageAveraged();

    if (loopEnabled && (sampleCount % UPDATE_EVERY_N_SAMPLES == 0)) {
      // Unity controller: u = uBias + (r - y)
      float e = refSensorV - yAvgV;

      float uUnsat = uBiasV + e;           // Kp = 1
      float uSat   = clampf(uUnsat, U_MIN_V, U_MAX_V);
      int sat      = (uSat != uUnsat) ? 1 : 0;

      setDacVoltage(uSat);

      // "p_term" is just e (since Kp=1). i_term=d_term=0.
      if (loggingEnabled) {
        logRow(now, refSensorV, uOutV, yAvgV, e, e, 0.0f, 0.0f, sat);
      }

      static unsigned long printCount = 0;
      printCount++;
      if (printCount % 5 == 0) {
        Serial.printf("t=%lu ms | r=%.3f y=%.3f | e=%.3f | u=%.3f %s\n",
                      now, refSensorV, yAvgV, e, uOutV, sat ? "(SAT)" : "");
      }
    }
  }
}
