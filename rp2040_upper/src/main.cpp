#include <Arduino.h>
#include <Wire.h>
#include <UARTComms.h>
#include <Adafruit_NeoPixel.h>

#define DEBUGGING
#ifdef DEBUGGING
#define DEBUG(x) Serial.println(String(#x) + String(": ") + String(x) + String('\r')); 
#else
#define DEBUG(x) 123;
#endif

#define PICO_LED 16
#define PICO_LED_BRIGHTNESS 50
Adafruit_NeoPixel pico_led(1, PICO_LED, NEO_GRB + NEO_KHZ800);

// I2C Comms with ESP
#define ESP_SDA_PIN 4
#define ESP_SCL_PIN 5
#define ESP_SEND_DATA_LEN 5
#define ESP_RCV_DATA_LEN 8
#define I2C_ADDR 0x09
volatile byte espSendBuffer[ESP_SEND_DATA_LEN];
byte lastBuffer[ESP_SEND_DATA_LEN];
byte espRcvBuffer[ESP_RCV_DATA_LEN];
volatile bool data_ready = false;
byte firstbyte = 5;

// UART Comms with top camera
#define TOP_CAM_TX_PIN 8
#define TOP_CAM_RX_PIN 9
#define TOP_CAM_DATA_LEN 5
byte topCamBuffer[TOP_CAM_DATA_LEN];
UARTComms topCamUART(TOP_CAM_TX_PIN, TOP_CAM_RX_PIN, topCamBuffer, TOP_CAM_DATA_LEN, Serial2);
float top_ball_angle, top_ball_dist;
bool topNoBall = false;
float topLastSeenBall = 0;

// UART Comms with front camera
#define FRONT_CAM_TX_PIN 12
#define FRONT_CAM_RX_PIN 13
#define FRONT_CAM_DATA_LEN 9
byte frontCamBuffer[FRONT_CAM_DATA_LEN];
UARTComms frontCamUART(FRONT_CAM_TX_PIN, FRONT_CAM_RX_PIN, frontCamBuffer, FRONT_CAM_DATA_LEN, Serial1);
float front_ball_angle, front_ball_dist;
bool frontNoBall = false;
float fronLastSeenBall = 0;

// Variables
float self_x = 0, self_y = 0, self_heading = 0;
int rounded_ball_angle = 0, rounded_ball_dist = 0;

void send(){
    if(data_ready) Wire.write((const uint8_t*)espSendBuffer, ESP_SEND_DATA_LEN);
    else Wire.write(lastBuffer, ESP_SEND_DATA_LEN);
}

void receive(int num_bytes){
    if(num_bytes != ESP_RCV_DATA_LEN){
        Serial.print("Received bad data length");
        return;
    }
    else Serial.print("Received: ");
    for (int i=0; i<ESP_RCV_DATA_LEN; i++){
        if(Wire.available()){
            espRcvBuffer[i] = Wire.read();
        }
    }
    // for (auto i : espRcvBuffer){
    //     Serial.print(i);
    //     Serial.print(" ");
    // }
    // Serial.println();
    if(espRcvBuffer[0]!=firstbyte) {
        Serial.print("Received bad data");
        return;
    }
    self_x = (float)(espRcvBuffer[1] + (espRcvBuffer[2]<<8)) / 128;
    self_y = (float)(espRcvBuffer[3] + (espRcvBuffer[4]<<8)) / 128;
    self_heading = (float)(espRcvBuffer[6] + (espRcvBuffer[7]<<8)) / 128;
    if(espRcvBuffer[5]==0) self_heading *= -1;
    DEBUG(self_x);
    DEBUG(self_y);
    DEBUG(self_heading);
}

void getTopCamData(){
    bool status = topCamUART.uartRead((byte)5);
    if(status){
        top_ball_angle = (float)(topCamBuffer[1] + (topCamBuffer[2]<<8)) / 128;
        top_ball_dist = (float)(topCamBuffer[3] + (topCamBuffer[4]<<8)) / 128;

        if(top_ball_angle==0 && top_ball_dist==0) {
            topNoBall = true;
        }
        else {
            topNoBall = false;
            topLastSeenBall = millis();
        }
        // DEBUG(top_ball_angle);
        // DEBUG(top_ball_dist);
    }
}

void setup(){
    Serial.begin(115200);
    topCamUART.init();
    frontCamUART.init();

    Wire.setSDA(ESP_SDA_PIN);
    Wire.setSCL(ESP_SCL_PIN);
    Wire.begin(I2C_ADDR);
    Wire.onRequest(send);
    Wire.onReceive(receive);

    lastBuffer[0] = 5;
    for (int i=1; i<ESP_SEND_DATA_LEN; i++) lastBuffer[i] = 0;
    espSendBuffer[0] = 5;

    pico_led.begin();
    pico_led.setBrightness(PICO_LED_BRIGHTNESS);
    pico_led.setPixelColor(0, pico_led.Color(15, 15, 0));
    pico_led.show();
}

void loop(){
    pico_led.setPixelColor(0, pico_led.Color(15, 0, 15));
    pico_led.show();
    getTopCamData();

    data_ready = false;
    espSendBuffer[0] = 5;
    frontNoBall = true;
    if(!frontNoBall){
        rounded_ball_angle = floor(front_ball_angle * 128);
        rounded_ball_dist = floor(front_ball_dist * 128);
    }
    else{
        rounded_ball_angle = floor(top_ball_angle * 128);
        rounded_ball_dist = floor(top_ball_dist * 128);
    }
    espSendBuffer[0] = 5;
    espSendBuffer[1] = rounded_ball_angle & 0xFF;
    espSendBuffer[2] = (rounded_ball_angle >> 8) & 0xFF;
    espSendBuffer[3] = rounded_ball_dist & 0xFF;
    espSendBuffer[4] = (rounded_ball_dist >> 8) & 0xFF;
    data_ready = true;

    memcpy(&lastBuffer, (const uint8_t*) espSendBuffer, ESP_SEND_DATA_LEN);
}
