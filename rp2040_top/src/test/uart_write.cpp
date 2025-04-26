#include <Arduino.h>

#define TX_PIN 0
#define RX_PIN 1
#define SERIAL_SIZE 128
#define DATA_LEN 30

void setup(){
    Serial.begin(115200);
    // while(!Serial.available()) ;
    // while(Serial.available()) Serial.read();
    Serial.println("started");

    Serial1.setRX(RX_PIN);
    Serial1.setTX(TX_PIN);
    Serial1.begin(115200);
    Serial.print("finished Serial setup");
}

void loop(){
    // Serial.print("running");
    for (int i=1; i<=DATA_LEN; i++){
        Serial1.write(i);
    }
    delay(3);
}
