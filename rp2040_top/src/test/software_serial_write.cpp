#include <Arduino.h>
#include <SoftwareSerial.h>

#define TX_PIN 2
#define RX_PIN 3
#define SERIAL_SIZE 128
#define DATA_LEN 30

SoftwareSerial swSerial(RX_PIN, TX_PIN);

void setup(){
    Serial.begin(115200);
    // while(!Serial.available()) ;
    // while(Serial.available()) Serial.read();
    Serial.println("started");

    swSerial.begin(115200);
    Serial.print("finished Serial setup");
}

void loop(){
    Serial.print("running");
    for (int i=1; i<=DATA_LEN; i++){
        swSerial.write(i);
    }
    delay(3);
}
