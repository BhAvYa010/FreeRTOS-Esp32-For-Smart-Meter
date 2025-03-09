#include <Wire.h>
#define BLYNK_PRINT Serial
#include "EmonLib.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

EnergyMonitor emon;
#define vCalibration 106.8
#define currCalibration 0.52

// Fixed missing declarations
const int ledPin = 2;               // ESP32 built-in LED
volatile bool voltageFault = false; // Atomic flag for ISRs
volatile bool currentFault = false; 
float kWh = 0;
unsigned long lastmillis = 0;

// Blynk credentials (replace with yours)
char auth[] = "your_auth_token";
char ssid[] = "your_SSID";
char pass[] = "your_password";

// Thresholds
const float HIGH_VOLTAGE = 230.0;
const float MAX_CURRENT = 10.0;

// FreeRTOS handles & mutex
TaskHandle_t sensorTaskHandle, blynkTaskHandle;
SemaphoreHandle_t energyMutex;

void IRAM_ATTR handleVoltageFault() {
  voltageFault = true;  // Atomic write
}

void IRAM_ATTR handleCurrentFault() {
  currentFault = true;
}

void sensorTask(void *pvParam) {  // Combined sensor task
  for(;;) {
    // Take measurement
    emon.calcVI(20, 2000);  // 20 crossings, 2000ms timeout
    
    // Get exclusive access to energy data
    if(xSemaphoreTake(energyMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      kWh += emon.apparentPower * (millis() - lastmillis) / 3600000000.0;
      lastmillis = millis();
      xSemaphoreGive(energyMutex);
    }

    // Check faults
    if(emon.Vrms > HIGH_VOLTAGE) {
      handleVoltageFault();
      digitalWrite(ledPin, HIGH);
    }
    if(emon.Irms > MAX_CURRENT) {
      handleCurrentFault();
      digitalWrite(ledPin, HIGH);
    }
    
    vTaskDelay(pdMS_TO_TICKS(1000));  // 1s delay
  }
}

void blynkTask(void *pvParam) {
  Blynk.begin(auth, ssid, pass);  // Blocking connection
  
  for(;;) {
    if(Blynk.connected()) {
      if(xSemaphoreTake(energyMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        Blynk.virtualWrite(V0, emon.Vrms);
        Blynk.virtualWrite(V1, emon.Irms); 
        Blynk.virtualWrite(V2, kWh);
        xSemaphoreGive(energyMutex);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5000));  // 5s update
  }
}

void checkFaults() {  // Central fault handler
  if(voltageFault || currentFault) {
    Serial.printf("FAULT! V:%.1fV I:%.3fA\n", emon.Vrms, emon.Irms);
    voltageFault = currentFault = false;
    digitalWrite(ledPin, LOW);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  
  // Initialize energy monitor
  emon.voltage(35, vCalibration, 1.7);  // Voltage pin
  emon.current(34, currCalibration);    // Current pin

  // Create mutex before tasks
  energyMutex = xSemaphoreCreateMutex();
  
  // Create tasks with safe stack sizes
  xTaskCreate(sensorTask, "Sensors", 4096, NULL, 2, &sensorTaskHandle);
  xTaskCreate(blynkTask, "Blynk", 8192, NULL, 1, &blynkTaskHandle);
}

void loop() {
  checkFaults();  // Handle faults in main loop
}
