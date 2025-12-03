#include <Arduino.h>
#include <SPIFFS.h>

// ==================== CONFIGURATION ====================
// Sampling rate in milliseconds
const unsigned long SAMPLING_INTERVAL_MS = 500;

// ADC input pin (sensor voltage from station)
const int ADC_PIN = 34;  // Analog signal input from sensor

// DAC output pin (setpoint to TRIAC DRIVE via 0-3.3V to 4-20mA module)
const int DAC_PIN = 25;  // DAC output (GPIO 25 or 26)

// Setpoint configuration
// Your module converts: 0V → 4mA, 3.3V → 20mA
// Assuming station: 4mA → 0%, 20mA → 100% of temperature range
const float SETPOINT_VOLTAGE = 1.5;  // Setpoint in volts (0 - 3.3V)
                                      // Adjust this to set your desired temperature

// Timing
const unsigned long INITIAL_WAIT_MS = 3000;  // Wait before step (to capture baseline)

// Data file path
const char* DATA_FILE = "/data.csv";

// Maximum file size in bytes
const size_t MAX_FILE_SIZE = 1000000;

// ==================== GLOBAL VARIABLES ====================
unsigned long lastSampleTime = 0;
unsigned long stepStartTime = 0;
unsigned long sampleCount = 0;
bool loggingEnabled = false;  // Start disabled, wait for step command
bool stepApplied = false;
float currentSetpoint = 0.0;  // Current DAC output voltage

// ==================== FUNCTION DECLARATIONS ====================
void initSPIFFS();
void logData(unsigned long timestamp, float setpoint, float sensorVoltage);
float adcToVoltage(int adcValue);
void setSetpointVoltage(float voltage);
void applyStep();
void printFileContents();
void clearDataFile();
void printFileInfo();
void printHelp();

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  
  Serial.println("\n========================================");
  Serial.println("   ESP32 Temperature Station Logger");
  Serial.println("   With Setpoint Control");
  Serial.println("========================================");
  
  // Initialize SPIFFS
  initSPIFFS();
  
  // Configure ADC (input from sensor)
  analogReadResolution(12);  // 12-bit resolution (0-4095)
  analogSetAttenuation(ADC_11db);  // Full range: 0-3.3V
  pinMode(ADC_PIN, INPUT);
  
  // Initialize DAC output to 0V (4mA = minimum setpoint)
  setSetpointVoltage(0.0);
  
  Serial.printf("\nHardware Configuration:\n");
  Serial.printf("  Sensor Input:    GPIO %d (ADC)\n", ADC_PIN);
  Serial.printf("  Setpoint Output: GPIO %d (DAC)\n", DAC_PIN);
  Serial.printf("  Sampling Rate:   %lu ms\n", SAMPLING_INTERVAL_MS);
  Serial.printf("  Step Setpoint:   %.2f V\n", SETPOINT_VOLTAGE);
  
  printHelp();
  
  // Print file info at startup
  printFileInfo();
  
  Serial.println("\n>>> Setpoint at 0V. Press 'g' to start step response test <<<\n");
}

// ==================== MAIN LOOP ====================
void loop() {
  // Handle serial commands
  if (Serial.available()) {
    char cmd = Serial.read();
    switch (cmd) {
      case 'g':  // GO - Start step response test
      case 'G':
        if (!stepApplied) {
          clearDataFile();  // Clear old data
          loggingEnabled = true;
          stepStartTime = millis();
          Serial.println("\n>>> LOGGING STARTED - Recording baseline... <<<");
          Serial.printf(">>> Step will be applied in %lu ms <<<\n\n", INITIAL_WAIT_MS);
        } else {
          Serial.println("Step already applied. Press 'r' to reset first.");
        }
        break;
        
      case 'r':  // RESET - Reset to initial state
      case 'R':
        setSetpointVoltage(0.0);
        stepApplied = false;
        loggingEnabled = false;
        sampleCount = 0;
        Serial.println("\n>>> RESET: Setpoint back to 0V. Press 'g' to start new test <<<\n");
        break;
        
      case 'p':  // PRINT file contents
      case 'P':
        loggingEnabled = false;  // Pause logging while printing
        printFileContents();
        break;
        
      case 'c':  // CLEAR data file
      case 'C':
        clearDataFile();
        break;
        
      case 'i':  // INFO - file info
      case 'I':
        printFileInfo();
        break;
        
      case 's':  // STOP/START logging
      case 'S':
        loggingEnabled = !loggingEnabled;
        Serial.printf("Logging %s\n", loggingEnabled ? "ENABLED" : "DISABLED");
        break;
        
      case 'v':  // Show current VALUES
      case 'V':
        Serial.printf("\nCurrent Setpoint: %.2f V\n", currentSetpoint);
        Serial.printf("Current Sensor:   %.3f V\n", adcToVoltage(analogRead(ADC_PIN)));
        Serial.printf("Step Applied:     %s\n", stepApplied ? "YES" : "NO");
        Serial.printf("Logging:          %s\n", loggingEnabled ? "ON" : "OFF");
        Serial.printf("Samples:          %lu\n\n", sampleCount);
        break;
        
      case 'h':  // HELP
      case 'H':
      case '?':
        printHelp();
        break;
        
      case '+':  // Manually increase setpoint
        currentSetpoint += 0.1;
        if (currentSetpoint > 3.3) currentSetpoint = 3.3;
        setSetpointVoltage(currentSetpoint);
        Serial.printf("Setpoint: %.2f V\n", currentSetpoint);
        break;
        
      case '-':  // Manually decrease setpoint
        currentSetpoint -= 0.1;
        if (currentSetpoint < 0) currentSetpoint = 0;
        setSetpointVoltage(currentSetpoint);
        Serial.printf("Setpoint: %.2f V\n", currentSetpoint);
        break;
    }
  }
  
  // Apply step after initial wait period
  if (loggingEnabled && !stepApplied && (millis() - stepStartTime >= INITIAL_WAIT_MS)) {
    applyStep();
  }
  
  // Sample at configured interval
  unsigned long currentTime = millis();
  if (loggingEnabled && (currentTime - lastSampleTime >= SAMPLING_INTERVAL_MS)) {
    lastSampleTime = currentTime;
    
    // Calculate timestamp relative to step start
    unsigned long relativeTime = currentTime - stepStartTime;
    
    // Read sensor voltage
    int rawValue = analogRead(ADC_PIN);
    float sensorVoltage = adcToVoltage(rawValue);
    
    // Log to file (timestamp, setpoint, sensor reading)
    logData(relativeTime, currentSetpoint, sensorVoltage);
    
    // Print to serial (every 10 samples)
    sampleCount++;
    if (sampleCount % 10 == 0) {
      Serial.printf("[%lu] t=%lu ms, Setpoint=%.2fV, Sensor=%.3fV\n", 
                    sampleCount, relativeTime, currentSetpoint, sensorVoltage);
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
  
  // Create file with header if it doesn't exist
  if (!SPIFFS.exists(DATA_FILE)) {
    File file = SPIFFS.open(DATA_FILE, FILE_WRITE);
    if (file) {
      file.println("timestamp_ms,setpoint_v,sensor_v");
      file.close();
      Serial.println("Created new data file with header");
    }
  }
}

float adcToVoltage(int adcValue) {
  // ESP32 ADC: 12-bit (0-4095), 3.3V reference
  return (adcValue / 4095.0) * 3.3;
}

void setSetpointVoltage(float voltage) {
  // Clamp to valid range
  if (voltage < 0) voltage = 0;
  if (voltage > 3.3) voltage = 3.3;
  
  currentSetpoint = voltage;
  
  // ESP32 DAC: 8-bit (0-255) for 0-3.3V
  int dacValue = (int)((voltage / 3.3) * 255);
  dacWrite(DAC_PIN, dacValue);
}

void applyStep() {
  Serial.println("\n========================================");
  Serial.println(">>> STEP APPLIED! <<<");
  Serial.printf(">>> Setpoint changed: 0V → %.2fV <<<\n", SETPOINT_VOLTAGE);
  Serial.println("========================================\n");
  
  setSetpointVoltage(SETPOINT_VOLTAGE);
  stepApplied = true;
}

void logData(unsigned long timestamp, float setpoint, float sensorVoltage) {
  // Check file size before writing
  File file = SPIFFS.open(DATA_FILE, FILE_READ);
  if (file) {
    size_t fileSize = file.size();
    file.close();
    
    if (fileSize >= MAX_FILE_SIZE) {
      Serial.println("WARNING: Max file size reached. Stopping logging.");
      loggingEnabled = false;
      return;
    }
  }
  
  // Append data to file
  file = SPIFFS.open(DATA_FILE, FILE_APPEND);
  if (file) {
    file.printf("%lu,%.4f,%.4f\n", timestamp, setpoint, sensorVoltage);
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
    file.println("timestamp_ms,setpoint_v,sensor_v");
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
    Serial.printf("Total samples: %u\n", lines > 0 ? lines - 1 : 0);
    file.close();
  }
  Serial.println("-------------------------------\n");
}

void printHelp() {
  Serial.println("\n----------------------------------------");
  Serial.println("Commands:");
  Serial.println("  'g' - GO: Start step response test");
  Serial.println("  'r' - RESET: Set setpoint to 0V");
  Serial.println("  's' - STOP/START logging");
  Serial.println("  'p' - PRINT file contents");
  Serial.println("  'c' - CLEAR data file");
  Serial.println("  'i' - Show file INFO");
  Serial.println("  'v' - Show current VALUES");
  Serial.println("  '+' - Increase setpoint by 0.1V");
  Serial.println("  '-' - Decrease setpoint by 0.1V");
  Serial.println("  'h' - Show this HELP");
  Serial.println("----------------------------------------");
}
