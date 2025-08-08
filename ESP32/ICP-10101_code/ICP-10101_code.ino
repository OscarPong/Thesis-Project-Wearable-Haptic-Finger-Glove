#include <Wire.h>
#include <ICP101xx.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#define I2C_SCL 0  // GPIO0
#define I2C_SDA 1  // GPIO1

ICP101xx icp10101(Wire);

#define SERVICE_UUID        "12345678-1234-1234-1234-123456789abc"  // My desired UUID
#define CHARACTERISTIC_UUID "87654321-4321-4321-4321-abcdefabcdef"  // My desired UUID

BLECharacteristic *pCharacteristic;
BLEServer *pServer;

void setup() {

    Serial.begin(9600);
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(100000);

    // Initialize the sensor
    icp10101.begin();
    icp10101.start();
    Serial.println("ICP-10101 Sensor Initialized!");

    // Initialize BLE
    BLEDevice::init("ESP32_C3_BLE");  // Name of your ESP32 device
    pServer = BLEDevice::createServer();

    // Print the BLE MAC address to the serial monitor
    Serial.print("BLE Device Address: ");
    Serial.println(BLEDevice::getAddress().toString().c_str());

    // Create BLE service
    BLEService *pService = pServer->createService(SERVICE_UUID);

    // Create BLE characteristic for pressure data
    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_NOTIFY  // Enables notification to the client
    );

    // Start the service
    pService->start();

    // Start advertising
    BLEAdvertising *pAdvertising = pServer->getAdvertising();
    pAdvertising->start();
    Serial.println("BLE Server Started. Waiting for client...");

    delay(5000);

}

void loop() {

    // Time step 10 ms
    delay(10);

    // Read absolute pressure in Pa
    float pressure, temperature;
    int status = icp10101.getData(pressure, temperature);

    // Get current time in seconds
    float currentTime = millis() / 1000.0;

    // Format message as "time;pressure"
    char message[32];
    // snprintf(message, sizeof(message), "%.3f;%.2f", currentTime, pressure);
    snprintf(message, sizeof(message), "%.2f", pressure);

    // Print message on serial monitor
    Serial.println(message);

    // Send message over BLE (notify the connected client)
    pCharacteristic->setValue((uint8_t*)message, strlen(message));  // Update the characteristic value
    pCharacteristic->notify();  // Notify client with the latest value

}