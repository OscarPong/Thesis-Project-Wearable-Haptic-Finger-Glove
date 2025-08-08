#pragma once

class ActuatorCallback : public BLECharacteristicCallbacks {
    public:
        ActuatorCallback() = default;

        void onWrite(BLECharacteristic *chr) override {
            uint8_t PWMPercentage = chr->getData()[0];     // 0-100
            targetPWM = static_cast<int>(255 * PWMPercentage / 100);
            // Serial.println(PWMPercentage);
        }

        int readTargetPWM() {
            return targetPWM;
        }

    private:
        int targetPWM = 0;
};