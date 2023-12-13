#include <Wire.h>
#include <vector>

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
volatile bool faultDetected = false;
int ledPin = LED_BUILTIN; 
const float HIGH_VOLTAGE_THRESHOLD = 230.0; 
const float MAX_CURRENT_THRESHOLD = 10.0; 

std::vector<float> voltageBuffer;
std::vector<float> currentBuffer;

void handleFault() {
  // Simulate sending an alert
  Serial.println("ALERT: High voltage detected!");
  digitalWrite(ledPin, HIGH); // Turn on LED for visual indication
}

void IRAM_ATTR handleCurrentFault() {
  faultDetected = true;
}

void IRAM_ATTR handleVoltageFault() {
  faultDetected = true;
}

float calculateMovingAverage(std::vector<float>& buffer) {
  float sum = 0;
  for (float value : buffer) {
    sum += value;
  }
  return sum / buffer.size();
}

void checkAndResetFault() {
  // Check if a fault was previously detected
  if (faultDetected) {
    // Check if voltage and current are within normal ranges
    if (emon.Vrms <= HIGH_VOLTAGE_THRESHOLD && emon.Irms <= MAX_CURRENT_THRESHOLD) {
      // Voltage and current are normal, reset the fault
      faultDetected = false;
      digitalWrite(ledPin, LOW);  // Turn off LED
    }
  }
}

void currentTask(void *parameter) {
  for (;;) {
    emon.calcVI(20, 2000);
    Serial.print("\tIrms: ");
    Serial.print(emon.Irms, 4);
    Serial.print("A");

    currentBuffer.push_back(emon.Irms);
    if (currentBuffer.size() > 5) {
      currentBuffer.erase(currentBuffer.begin());
    }

    // Notify the power calculation task
    xTaskNotifyFromISR(powerCalculationTaskHandle, 0, eNoAction, NULL);
    
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

    voltageBuffer.push_back(emon.Vrms);
    if (voltageBuffer.size() > 5) {
      voltageBuffer.erase(voltageBuffer.begin());
    }

    // Check for high voltage condition
    if (emon.Vrms > HIGH_VOLTAGE_THRESHOLD) {
      faultDetected = true;
    } else if (faultDetected) {
      faultDetected = false;
      // Voltage has returned to normal, turn off LED
      digitalWrite(ledPin, LOW);
    }

    // Notify the power calculation task
    xTaskNotifyFromISR(powerCalculationTaskHandle, 0, eNoAction, NULL);
    
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay for 1 second
  }
}

void powerCalculationTask(void *parameter) {
  for (;;) {
    // Wait for notifications from current and voltage tasks
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Check for faults
    if (faultDetected) {
      handleFault();
    }

    // Calculate power here
    Serial.print("\tkWh: ");
    kWh = kWh + emon.apparentPower * (millis() - lastmillis) / 3600000000.0;
    Serial.print(kWh, 4);
    Serial.println("kWh");
    lastmillis = millis();

    // Calculate moving average for the last 5 seconds
    float avgVoltage = calculateMovingAverage(voltageBuffer);
    float avgCurrent = calculateMovingAverage(currentBuffer);

    // Send data to Blynk here (send the moving averages)
    Blynk.virtualWrite(V1, avgVoltage);
    Blynk.virtualWrite(V0, avgCurrent);
    Blynk.virtualWrite(V3, kWh);

    // Check and reset faults
    checkAndResetFault();

    vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay for 1 second
  }
}

void blynkTask(void *parameter) {
  for (;;) {
    // Send data to Blynk here
    Blynk.virtualWrite(V1, calculateMovingAverage(voltageBuffer));
    Blynk.virtualWrite(V0, calculateMovingAverage(currentBuffer));
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
