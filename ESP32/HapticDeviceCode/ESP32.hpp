#pragma once

#include <Wire.h>

#include "Sensor.hpp"
#include "VibrationActuator.hpp"
#include "BLEManager.hpp"

const int I2C_SCL = 0;  // GPIO0
const int I2C_SDA = 1;  // GPIO1

const int H_SLEEP_PIN = 4;  // GPIO4
const int H_PH_PIN = 5;     // GPIO5
const int H_EN_PIN = 6;     // GPIO6

const int FREQ_HZ = 235;    // Actuator frequency in Hz

class ESP32 {
    public:
        ESP32() : sensor{&Wire}, actuator{H_SLEEP_PIN, H_PH_PIN, H_EN_PIN, FREQ_HZ}, ble{} {};

        void setup() {

            Serial.begin(9600);
            Wire.begin(I2C_SDA, I2C_SCL);
            Wire.setClock(100000);

            sensor.setup();
            actuator.setup();
            ble.setup();

            Serial.println("ESP32 Setup Finished!");
            delay(5000);

            // Start multitasks
            xTaskCreate(taskSensorWrapper, "Sensor", 4096, this, 1, NULL);
            xTaskCreate(taskActuatorWrapper, "Actuator", 2048, this, 2, NULL);
        }
        
        void taskSensor() {
            float lastBLETime = 0;
            while (true) {
                // Get current time in seconds
                float currentTime = millis() / 1000.0;
                // Read absolute pressure in Pa
                float pressurePa = sensor.readPressure();
                // Read temperature in deg
                float temperature = sensor.readTemperature();

                sensor.computeMeasuredVoltage();
                float measuredPWM = sensor.readMeasuredPWM();
                float currentPWM = actuator.readCurrentPWM();

                // Format message as "time;pressure;temperature;measuredPWM;"
                char message[32];
                snprintf(message, sizeof(message), "%.3f;%.2f;%.2f;%.2f;", currentTime, pressurePa, temperature, measuredPWM);
                
                // Print message on serial monitor & Send message on BLE every 10 ms
                if ((currentTime - lastBLETime) >= 0.01) {
                    Serial.println(message);
                    ble.sendMessage(message);
                    lastBLETime = currentTime;
                }
                
                // Delay 5 ms
                vTaskDelay(pdMS_TO_TICKS(5));
            }
        }
        
        void taskActuator() {
            while (true) {
                // Read user input from BLE
                int targetPWM = ble.readTargetPWM();
                actuator.storeTargetPWM(targetPWM);
                // Compute PWM by PID controller
                actuator.compute(sensor.readMeasuredPWM());
                // Set the PWM
                actuator.setPWM();
                // Delay 10 ms
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
        
        // Static wrapper functions
        static void taskSensorWrapper(void* pvParams) {
            ESP32* self = static_cast<ESP32*>(pvParams);
            self->taskSensor();
        }
        
        static void taskActuatorWrapper(void* pvParams) {
            ESP32* self = static_cast<ESP32*>(pvParams);
            self->taskActuator();
        }
        
    private:
        Sensor sensor;
        VibrationActuator actuator;
        BLEManager ble;
};
