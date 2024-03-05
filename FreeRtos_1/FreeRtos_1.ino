#include <Wire.h>

#define BLYNK_PRINT Serial
#include "EmonLib.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
EnergyMonitor emon;
#define vCalibration 106.8
#define currCalibration 0.52
BlynkTimer timer;

char auth[] = "DUDgpXIBc8JMH3-K3RHAQs7-Vq0U6ITs";
char ssid[] = "DESKTOP-D6BKAJV 1575";
char pass[] = "2fR129>5";

float kWh = 0;
unsigned long lastmillis = millis();

TaskHandle_t currentTaskHandle = NULL;
TaskHandle_t voltageTaskHandle = NULL;
TaskHandle_t powerCalculationTaskHandle = NULL;
TaskHandle_t blynkTaskHandle = NULL;

void handleVoltageFault() {
  // Handle high voltage fault
  Serial.println("ALERT: High voltage detected!");
  // Additional actions if needed
}

void handleCurrentFault() {
  // Handle high current fault
  Serial.println("ALERT: High current detected!");
  // Additional actions if needed
}

void currentTask(void *parameter) {
  for (;;) {
    emon.calcVI(20, 2000);
    Serial.print("\tIrms: ");
    Serial.print(emon.Irms, 4);
    Serial.print("A");

    // Notify the power calculation task
    xTaskNotify(powerCalculationTaskHandle, 1, eNoAction);
    
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay for 1 second
  }
}

void voltageTask(void *parameter) {
  for (;;) {
    // Read voltage sensor value here
    emon.calcVI(20, 2000);
    Serial.print("Vrms: ");
    Serial.print(emon.Vrms, 2);
    Serial.print("V");

    // Notify the power calculation task
    xTaskNotify(powerCalculationTaskHandle, 1, eNoAction);
    
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay for 1 second
  }
}

void powerCalculationTask(void *parameter) {
  for (;;) {
    // Wait for notifications from current and voltage tasks
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Calculate power here
    Serial.print("\tkWh: ");
    kWh = kWh + emon.apparentPower * (millis() - lastmillis) / 3600000000.0;
    Serial.print(kWh, 4);
    Serial.println("kWh");
    lastmillis = millis();

    vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay for 1 second
  }
}

void blynkTask(void *parameter) {
  for (;;) {
    // Send data to Blynk here
    Blynk.virtualWrite(V1, emon.Vrms);
    Blynk.virtualWrite(V0, emon.Irms);
    Blynk.virtualWrite(V3, kWh);
    vTaskDelay(5000 / portTICK_PERIOD_MS); // Delay for 5 seconds
  }
}

void setup() {
  Serial.begin(9600);
  emon.voltage(35, vCalibration, 1.7); // Voltage: input pin, calibration, phase_shift
  emon.current(34, currCalibration);   // Current: input pin, calibration.
  Blynk.begin(auth, ssid, pass);

  pinMode(ledPin, OUTPUT);

  // Attach interrupt handlers
  attachInterrupt(digitalPinToInterrupt(34), handleCurrentFault, RISING);
  attachInterrupt(digitalPinToInterrupt(35), handleVoltageFault, RISING);

  xTaskCreate(currentTask, "CurrentTask", 10000, NULL, 4, &currentTaskHandle);
  xTaskCreate(voltageTask, "VoltageTask", 10000, NULL, 3, &voltageTaskHandle);
  xTaskCreate(powerCalculationTask, "PowerCalculationTask", 10000, NULL, 2, &powerCalculationTaskHandle);
  xTaskCreate(blynkTask, "BlynkTask", 10000, NULL, 1, &blynkTaskHandle);
}

void loop() {
  Blynk.run();
  timer.run();
  checkAndResetFault();  // Check and reset faults in the loop
}
