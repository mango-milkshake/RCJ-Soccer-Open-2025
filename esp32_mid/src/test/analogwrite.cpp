#include <Arduino.h>
#define PIN 37
#define VAL 127

void setup(){
    Serial.begin(115200);
    pinMode(PIN, OUTPUT);
    analogWriteFrequency(25000);
}

void loop(){
    analogWrite(PIN, VAL);
    delay(1);
}

// #include <Arduino.h>

// // Define the PWM properties
// #define LEDC_CHANNEL 0      // Use LEDC channel 0
// #define LEDC_TIMER 0        // Use LEDC timer 0
// #define LEDC_BASE_FREQ 5000 // 5 kHz frequency
// #define LEDC_RESOLUTION 8   // 8-bit resolution (0-255)
// #define PIN 37
// #define VAL 127

// void setup() {
//     // Configure LEDC timer
//     ledcSetup(LEDC_CHANNEL, LEDC_BASE_FREQ, LEDC_RESOLUTION);

//     // Attach the pin to the LEDC channel
//     ledcAttachPin(PIN, LEDC_CHANNEL);

//     // Set the duty cycle
//     ledcWrite(LEDC_CHANNEL, VAL);
// }

// void loop() {
//     // No need to continuously write in loop if the value is constant
//     // If you want to change the duty cycle, you would do it here:
//     // ledcWrite(LEDC_CHANNEL, new_value);
// }
