#pragma once

#include <LPS22HHSensor.h>

class Sensor {
    public:
        Sensor() = default;
        Sensor(TwoWire* wireAddress) : lps22hh{wireAddress} {}

        void setup() {
            lps22hh.begin();
            lps22hh.Enable();
            lps22hh.SetOutputDataRate(200.0f);
            Serial.println("LPS22HH Sensor Initialized!");
        }

        float readPressure() {
            // Read absolute pressure
            lps22hh.GetPressure(&pressure);
            float pressurePa = pressure * 100;  // Convert into Pa

            if (abs(pressurePa - previousPressure == 0.0)) return pressurePa;
            // Store deltaP into buffer
            deltaP = abs(pressurePa - previousPressure);
            previousPressure = pressurePa;
            previousdeltaP = deltaP;

            if (deltaP > 300.0) return pressurePa;
            deltaPBuffer[deltaPBufferIndex] = deltaP;
            deltaPBufferIndex = (deltaPBufferIndex + 1) % BUFFER_SIZE;
            
            return pressurePa;
        }

        float readTemperature() {
            lps22hh.GetTemperature(&temperature);
            return temperature; // In degrees Celsius
        }

        float computeAverageDeltaP() {

            // Copy deltaP to a temporary array and sort descending
            float temp[BUFFER_SIZE];
            memcpy(temp, deltaPBuffer, sizeof(float) * BUFFER_SIZE);

            // Sort in descending order
            for (int i = 0; i < BUFFER_SIZE - 1; i++) {
                for (int j = 0; j < BUFFER_SIZE - i - 1; j++) {
                    if (temp[j] < temp[j + 1]) {
                        float t = temp[j];
                        temp[j] = temp[j + 1];
                        temp[j + 1] = t;
                    }
                }
            }

            // Get top 10
            const int topN = 10;
            float top10[topN];
            for (int i = 0; i < topN; i++) {
                top10[i] = temp[i];
            }

            // Filter outliers
            float sum = 0.0;
            int count = 0;
            for (int i = 2; i < topN-2; i++) {
                sum += top10[i];
                count++;
            }

            // Return mean of filtered values
            if (count == 0) return 0.0;
            return sum / count;
        }

        float butterworthFilter(float input) {
            // Shift old samples
            x[2] = x[1];
            x[1] = x[0];
            x[0] = input;

            y[2] = y[1];
            y[1] = y[0];

            // Apply difference equation
            y[0] = b[0]*x[0] + b[1]*x[1] + b[2]*x[2] - a[1]*y[1] - a[2]*y[2];

            return y[0];
        }

        float computeMeasuredVoltage() {
            float averageDeltaP = computeAverageDeltaP();
            float filteredDeltaP = butterworthFilter(averageDeltaP);

            // Calculate Peak-to-Peak voltage from Pressure: // y = -0.249281*x^2 + 10.043143*x + 9.408743
            float a = -0.249281;
            float b = 10.043143;
            float c = 9.408743 - filteredDeltaP;

            float discriminant = b * b - 4 * a * c;

            if (discriminant < 0) return -999;

            float x1 = (-b + sqrt(discriminant)) / (2 * a);
            float x2 = (-b - sqrt(discriminant)) / (2 * a);

            // Return the solution in [-1, 11] V
            if (x1 >= -1 && x1 <= 11) measuredVoltage = x1;
            if (x2 >= -1 && x2 <= 11) measuredVoltage = x2;

            return measuredVoltage;
        }

        int readMeasuredPWM() {
            return int(255 * (measuredVoltage/10));
        }

    private:
        LPS22HHSensor lps22hh;
        float pressure, previousPressure;
        float temperature;

        static const int BUFFER_SIZE = 50;
        float deltaPBuffer[BUFFER_SIZE];
        int deltaPBufferIndex= 0;
        float deltaP, previousdeltaP;
        float measuredVoltage, previousMeasuredVoltage;

        // Butterworth coefficients from MATLAB
        const float b[3] = {0.0000212, 0.00004239, 0.0000212};  // numerator
        const float a[3] = {1.0, -1.9869, 0.987};    // denominator
        // Filter state buffers
        float x[3] = {0};  // input history
        float y[3] = {0};  // output history
};