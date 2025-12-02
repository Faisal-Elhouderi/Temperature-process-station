#include <Arduino.h>
#include <SPIFFS.h>

// ==================== CONFIGURATION ====================
// Sampling rate in milliseconds (change this value as needed)
// 100ms = 10 samples/sec, 50ms = 20 samples/sec, 10ms = 100 samples/sec
const unsigned long SAMPLING_INTERVAL_MS = 100;

// ADC input pin (use ADC1 pins: GPIO 32-39 recommended)
const int ADC_PIN = 34;  // Analog signal input

// Data file path
const char* DATA_FILE = "/data.csv";

// Maximum file size in bytes (to prevent filling up flash)
// SPIFFS on ESP32 is typically ~1.5MB, setting limit to 1MB
const size_t MAX_FILE_SIZE = 1000000;

// ==================== GLOBAL VARIABLES ====================
unsigned long lastSampleTime = 0;
unsigned long sampleCount = 0;
bool loggingEnabled = true;

// ==================== FUNCTION DECLARATIONS ====================
void initSPIFFS();
void logData(float voltage);
float adcToVoltage(int adcValue);
void printFileContents();
void clearDataFile();
void printFileInfo();

// ==================== SETUP ====================s



void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  
  Serial.println("\n========================================");
  Serial.println("   ESP32 Analog Data Logger");
  Serial.println("========================================");
  
  // Initialize SPIFFS
  initSPIFFS();
  
  // Configure ADC
  analogReadResolution(12);  // 12-bit resolution (0-4095)
  analogSetAttenuation(ADC_11db);  // Full range: 0-3.3V
  
  // Configure ADC pin as input
  pinMode(ADC_PIN, INPUT);
  
  Serial.printf("Signal Input: GPIO %d\n", ADC_PIN);
  Serial.printf("Sampling interval: %lu ms\n", SAMPLING_INTERVAL_MS);
  Serial.println("----------------------------------------");
  Serial.println("Commands via Serial:");
  Serial.println("  'p' - Print file contents");
  Serial.println("  'c' - Clear data file");
  Serial.println("  'i' - Print file info");
  Serial.println("  's' - Stop/Start logging");
  Serial.println("----------------------------------------");
  Serial.println("Logging started...\n");
  
  // Print file info at startup
  printFileInfo();
}

// ==================== MAIN LOOP ====================
void loop() {
  // Handle serial commands
  if (Serial.available()) {
    char cmd = Serial.read();
    switch (cmd) {
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
      case 's':
      case 'S':
        loggingEnabled = !loggingEnabled;
        Serial.printf("Logging %s\n", loggingEnabled ? "ENABLED" : "DISABLED");
        break;
    }
  }
  
  // Sample at configured interval
  unsigned long currentTime = millis();
  if (loggingEnabled && (currentTime - lastSampleTime >= SAMPLING_INTERVAL_MS)) {
    lastSampleTime = currentTime;
    
    // Read analog value
    int rawValue = analogRead(ADC_PIN);
    
    // Convert to voltage
    float voltage = adcToVoltage(rawValue);
    
    // Log to file
    logData(voltage);
    
    // Print to serial (every 10 samples to reduce serial traffic)
    sampleCount++;
    if (sampleCount % 10 == 0) {
      Serial.printf("[%lu] Voltage: %.3fV\n", sampleCount, voltage);
    }
  }
}

// ==================== FUNCTION DEFINITIONS ====================

void initSPIFFS() {
  if (!SPIFFS.begin(true)) {
    Serial.println("ERROR: SPIFFS mount failed!");
    return;
  }
  Serial.println("SPIFFS mounted successfully");
  
  // Check if data file exists, if not create with header
  if (!SPIFFS.exists(DATA_FILE)) {
    File file = SPIFFS.open(DATA_FILE, FILE_WRITE);
    if (file) {
      file.println("timestamp_ms,voltage");
      file.close();
      Serial.println("Created new data file with header");
    }
  }
}

float adcToVoltage(int adcValue) {
  // ESP32 ADC: 12-bit (0-4095), 3.3V reference
  return (adcValue / 4095.0) * 3.3;
}

void logData(float voltage) {
  // Check file size before writing
  File file = SPIFFS.open(DATA_FILE, FILE_READ);
  if (file) {
    size_t fileSize = file.size();
    file.close();
    
    if (fileSize >= MAX_FILE_SIZE) {
      Serial.println("WARNING: Max file size reached. Stopping logging.");
      Serial.println("Use 'c' command to clear the file.");
      loggingEnabled = false;
      return;
    }
  }
  
  // Append data to file
  file = SPIFFS.open(DATA_FILE, FILE_APPEND);
  if (file) {
    file.printf("%lu,%.4f\n", millis(), voltage);
    file.close();
  } else {
    Serial.println("ERROR: Could not open file for writing");
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

void clearDataFile() {
  File file = SPIFFS.open(DATA_FILE, FILE_WRITE);
  if (file) {
    file.println("timestamp_ms,voltage");
    file.close();
    sampleCount = 0;
    Serial.println("Data file cleared");
  } else {
    Serial.println("ERROR: Could not clear file");
  }
}

void printFileInfo() {
  Serial.println("\n---------- FILE INFO ----------");
  
  size_t totalBytes = SPIFFS.totalBytes();
  size_t usedBytes = SPIFFS.usedBytes();
  
  Serial.printf("SPIFFS Total: %u bytes\n", totalBytes);
  Serial.printf("SPIFFS Used:  %u bytes\n", usedBytes);
  Serial.printf("SPIFFS Free:  %u bytes\n", totalBytes - usedBytes);
  
  File file = SPIFFS.open(DATA_FILE, FILE_READ);
  if (file) {
    Serial.printf("Data file size: %u bytes\n", file.size());
    
    // Count lines (samples)
    size_t lines = 0;
    while (file.available()) {
      if (file.read() == '\n') lines++;
    }
    Serial.printf("Total samples: %u\n", lines > 0 ? lines - 1 : 0);  // -1 for header
    file.close();
  }
  Serial.println("-------------------------------\n");
}
