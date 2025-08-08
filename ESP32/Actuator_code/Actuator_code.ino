#include <Wire.h>
#include <LPS22HHSensor.h>
#include <driver/timer.h>

#define I2C_SCL 0  // GPIO0
#define I2C_SDA 1  // GPIO1

#define H_SLEEP_PIN  4  // GPIO4
#define H_PH_PIN  5     // GPIO5
#define H_EN_PIN  6     // GPIO6

#define FREQ_HZ 235
#define HALF_PERIOD_US (1000000 / (FREQ_HZ * 2))  // = 2127 µs

volatile bool ph_state = false;

void IRAM_ATTR togglePH(void* arg) {
    ph_state = !ph_state;
    digitalWrite(H_PH_PIN, ph_state);
}

esp_timer_handle_t ph_timer;

void setup() {

    Serial.begin(9600);
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(100000);

    // Initialize actuator
    pinMode(H_SLEEP_PIN, OUTPUT);
    pinMode(H_PH_PIN, OUTPUT);
    pinMode(H_EN_PIN, OUTPUT);

    digitalWrite(H_SLEEP_PIN, HIGH);    // Enable H-bridge
    digitalWrite(H_PH_PIN, LOW);        // Start LOW
    analogWrite(H_EN_PIN, 0);           // No PWM yet

    // Create periodic timer
    const esp_timer_create_args_t timer_args = {
        .callback = &togglePH,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "ph_toggle_timer"
    };

    esp_timer_create(&timer_args, &ph_timer);
    esp_timer_start_periodic(ph_timer, HALF_PERIOD_US);  // Call every ~2127 µs (235 Hz)
    
}


void loop() {
    
    Serial.print("Time: ");
    Serial.print(millis());
    Serial.println(" ms");

}