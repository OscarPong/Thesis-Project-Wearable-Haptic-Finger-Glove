#pragma once

#include <esp_timer.h>

class VibrationActuator {
    public:
        VibrationActuator() = default;
        VibrationActuator(const int& sleep_pin, const int& PH_pin, const int& EN_pin, const int& frequency) {
            // Initialize actuator
            sleepPin = sleep_pin;
            PHPin = PH_pin;
            ENPin = EN_pin;
            halfPeriod = 1000000 / (2 * frequency);  // in microseconds
            kp = 1.6;
            ki = 0.9;
            kd = 0.7;
        }

        void setup() {
            // Initialize actuator
            pinMode(sleepPin, OUTPUT);
            pinMode(PHPin, OUTPUT);
            pinMode(ENPin, OUTPUT);

            // Turn on the actuator
            digitalWrite(sleepPin, HIGH);
            digitalWrite(PHPin, LOW);
            analogWrite(ENPin, 0);

            // Create timer and start toggling PHPin at freq Hz
            const esp_timer_create_args_t timer_args = {
                .callback = &VibrationActuator::timerCallbackTrampoline,
                .arg = this,
                .dispatch_method = ESP_TIMER_TASK,
                .name = "vib_toggle"
            };
            esp_timer_create(&timer_args, &timer);
            esp_timer_start_periodic(timer, halfPeriod);
        }

        void storeTargetPWM(int PWM) {
            if (PWM < 0) PWM = 0;
            if (PWM > 255) PWM = 255;
            targetPWM = PWM;
        }

        void compute(float input) {

            // Comment out to switch on PID control
            // currentPWM = targetPWM;
            // return;

            currentTime = micros();
            dt = static_cast<float>(currentTime - previousTime) / 1e6;
            previousTime = currentTime;

            // PID CONTROLLER
            error = targetPWM - input;
            integral = integral + error * dt;
            derivative = (error - previousError) / dt;
            float output = kp * error + ki * integral + kd * derivative;

            previousError = error;

            if (output < 0) output = 0;
            if (output > 255) output = 255;
            if (targetPWM == 0) {
                currentPWM = 0;
            } else {
                currentPWM = output;
            }
        }

        void setPWM() {
            if (currentPWM != previousPWM) {
                analogWrite(ENPin, currentPWM);
                previousPWM = currentPWM;
            }
        }

        float readCurrentPWM() {
            return currentPWM;  
        }

        // Static trampoline to call class method from ISR
        static void timerCallbackTrampoline(void* arg) {
            static_cast<VibrationActuator*>(arg)->togglePH();
        }

        void togglePH() {
            ph_state = !ph_state;
            digitalWrite(PHPin, ph_state);
        }

    private:
        int sleepPin, PHPin, ENPin;
        uint32_t halfPeriod;
        esp_timer_handle_t timer = nullptr;;
        bool ph_state = false;

        int targetPWM, currentPWM, previousPWM = 0;
        uint32_t currentTime, previousTime = micros();
        float dt;
        float kp, ki, kd;
        float error, previousError, derivative, integral;
};