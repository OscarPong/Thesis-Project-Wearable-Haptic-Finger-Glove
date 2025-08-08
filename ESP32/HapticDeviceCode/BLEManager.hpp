#pragma once

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#include "ActuatorCallback.hpp"

const char* SERVICE_UUID = "12345678-1234-1234-1234-123456789abc";            // My desired Service UUID
const char* S_CHARACTERISTIC_UUID = "87654321-4321-4321-4321-abcdefabcdef";   // My Sensor Characteristic UUID
const char* A_CHARACTERISTIC_UUID = "87654321-4321-4321-4321-abcdefabcd01";   // My Actuator Characteristic UUID


class BLEManager {
    public:
        BLEManager() = default;

        void setup() {
            // Initialize BLE
            BLEDevice::init("ESP32_C3_BLE");  // Name of your ESP32 device
            pServer = BLEDevice::createServer();

            // Print the BLE MAC address to the serial monitor
            Serial.print("BLE Device Address: ");
            Serial.println(BLEDevice::getAddress().toString().c_str());

            // Create BLE service
            pService = pServer->createService(SERVICE_UUID);

            // Create BLE characteristic for reading pressure data
            pSensorChar = pService->createCharacteristic(
                S_CHARACTERISTIC_UUID,
                BLECharacteristic::PROPERTY_READ |
                BLECharacteristic::PROPERTY_NOTIFY  // Enables notification to the client
            );

            // Create BLE characteristic for receiving actuator commands
            pActuatorChar = pService->createCharacteristic(
                A_CHARACTERISTIC_UUID,
                BLECharacteristic::PROPERTY_WRITE |
                BLECharacteristic::PROPERTY_WRITE_NR    // Write without response
            );

            // Set Actuator Commands Callback
            pActuatorChar->setCallbacks(&actuatorCallback);

            // Start the service
            pService->start();

            // Start advertising
            BLEAdvertising *pAdvertising = pServer->getAdvertising();
            pAdvertising->start();
            Serial.println("BLE Server Started. Waiting for client...");
        }

        void sendMessage(const char* message) {
            // Send message over BLE (notify the connected client)
            pSensorChar->setValue((uint8_t*)message, strlen(message));  // Update the characteristic value
            pSensorChar->notify();  // Notify client with the latest value
        }
        
        int readTargetPWM() {
            return actuatorCallback.readTargetPWM();
        }

    private:
        BLEServer* pServer;
        BLEService* pService;
        BLECharacteristic* pSensorChar;
        BLECharacteristic* pActuatorChar;
        ActuatorCallback actuatorCallback;
};