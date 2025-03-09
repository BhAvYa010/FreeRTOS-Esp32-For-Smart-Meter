#include <Wire.h>
#define BLYNK_PRINT Serial
#include "EmonLib.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/timer.h>

EnergyMonitor emon;

// Calibration Constants
#define vCalibration 106.8    // Adjust based on your voltage sensor
#define currCalibration 0.52  // Adjust based on your current sensor

// Hardware Configuration
const int ledPin = 2;          // Built-in LED
const int voltageFaultPin = 25; // GPIO for over-voltage hardware interrupt
const int currentFaultPin = 26; // GPIO for over-current hardware interrupt

// Shared Data Protection
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile float currentVrms = 0; // Protected by timerMux
volatile float currentIrms = 0; // Protected by timerMux
volatile bool voltageFault = false;
volatile bool currentFault = false;

// Energy Tracking
float kWh = 0;
unsigned long lastmillis = 0;
SemaphoreHandle_t energyMutex;

// Blynk Credentials
char auth[] = "your_auth_token";
char ssid[] = "your_SSID";
char pass[] = "your_password";

// Thresholds
const float HIGH_VOLTAGE = 230.0; // 230V upper limit
const float MAX_CURRENT = 10.0;   // 10A current limit

// FreeRTOS Task Handles
TaskHandle_t sensorTaskHandle, blynkTaskHandle;

// Timer Configuration (100ms interval)
#define TIMER_DIVIDER 80          // 1MHz timer (80MHz/80)
#define TIMER_INTERVAL_MS 100     
hw_timer_t *timer = NULL;

//======================================================================
// Timer ISR: Checks for threshold violations every 100ms
//======================================================================
void IRAM_ATTR onTimer() {
  // Atomically capture sensor values
  portENTER_CRITICAL_ISR(&timerMux);
  float v = currentVrms;
  float i = currentIrms;
  portEXIT_CRITICAL_ISR(&timerMux);

  // Check thresholds
  if(v > HIGH_VOLTAGE) voltageFault = true;
  if(i > MAX_CURRENT) currentFault = true;
}

//======================================================================
// Hardware Interrupt Handlers (External Fault Signals)
//======================================================================
void IRAM_ATTR handleVoltageFaultISR() {
  voltageFault = true;
  digitalWrite(ledPin, HIGH); // Immediate visual feedback
}

void IRAM_ATTR handleCurrentFaultISR() {
  currentFault = true;
  digitalWrite(ledPin, HIGH);
}

//======================================================================
// Sensor Task: Measures and Updates Energy Data
//======================================================================
void sensorTask(void *pvParam) {
  for(;;) {
    // 1. Measure voltage/current (20 AC cycles, 2000ms timeout)
    emon.calcVI(20, 2000); 

    // 2. Atomically update shared sensor values
    portENTER_CRITICAL(&timerMux);
    currentVrms = emon.Vrms;
    currentIrms = emon.Irms;
    portEXIT_CRITICAL(&timerMux);

    // 3. Update energy (protected by mutex)
    if(xSemaphoreTake(energyMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      kWh += emon.apparentPower * (millis() - lastmillis) / 3600000000.0; // kWh calculation
      lastmillis = millis();
      xSemaphoreGive(energyMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(1000)); // 1s cycle
  }
}

//======================================================================
// Blynk Task: Non-Blocking IoT Communication
//======================================================================
void blynkTask(void *pvParam) {
  Blynk.config(auth); // Non-blocking configuration
  
  for(;;) {
    if(!Blynk.connected()) {
      Blynk.connect(3000); // 3s timeout
    }

    if(Blynk.connected()) {
      // Atomically read values
      portENTER_CRITICAL(&timerMux);
      float v = currentVrms;
      float i = currentIrms;
      portEXIT_CRITICAL(&timerMux);

      // Get energy data
      if(xSemaphoreTake(energyMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        Blynk.virtualWrite(V0, v);
        Blynk.virtualWrite(V1, i);
        Blynk.virtualWrite(V2, kWh);
        xSemaphoreGive(energyMutex);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5000)); // 5s update
  }
}

//======================================================================
// Fault Handler: Processes Active Faults
//======================================================================
void checkFaults() {
  if(voltageFault || currentFault) {
    // Atomic read for accurate diagnostics
    portENTER_CRITICAL(&timerMux);
    float v = currentVrms;
    float i = currentIrms;
    portEXIT_CRITICAL(&timerMux);

    Serial.printf("[FAULT] Voltage: %.1fV, Current: %.2fA\n", v, i);
    
    // Reset flags only if values are now safe
    if(v <= HIGH_VOLTAGE && i <= MAX_CURRENT) {
      voltageFault = currentFault = false;
      digitalWrite(ledPin, LOW);
    }
  }
}

//======================================================================
// Setup & Main Loop
//======================================================================
void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  
  // 1. Sensor Initialization
  emon.voltage(35, vCalibration, 1.7); // Phase calibration for 230V systems
  emon.current(34, currCalibration);

  // 2. Hardware Interrupt Configuration
  pinMode(voltageFaultPin, INPUT_PULLDOWN);
  pinMode(currentFaultPin, INPUT_PULLDOWN);
  attachInterrupt(voltageFaultPin, handleVoltageFaultISR, RISING);
  attachInterrupt(currentFaultPin, handleCurrentFaultISR, RISING);

  // 3. Timer Interrupt Setup
  timer = timerBegin(0, TIMER_DIVIDER, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, TIMER_INTERVAL_MS * 1000, true);
  timerAlarmEnable(timer);

  // 4. FreeRTOS Initialization
  energyMutex = xSemaphoreCreateMutex();
  xTaskCreate(sensorTask, "Sensor Task", 4096, NULL, 2, &sensorTaskHandle);
  xTaskCreate(blynkTask, "Blynk Task", 8192, NULL, 1, &blynkTaskHandle);
}

void loop() {
  checkFaults();
  vTaskDelay(pdMS_TO_TICKS(100)); // Reduce CPU usage
}
