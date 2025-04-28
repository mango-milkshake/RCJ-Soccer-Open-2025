#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>

#define NO_SERIAL

#define TX_PIN 8
#define RX_PIN 9
#define SERIAL_SIZE 128
#define DATA_LEN 30

byte buffer[DATA_LEN];
int counter = 0, total = 0;

AsyncWebServer server(80);

const char* ssid = "heeheehaahaaheeheehaahaa"; // WiFi SSID
const char* password = "lipofire"; // WiFi Password

void initWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi ..");
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print('.');
        delay(1000);
    }
    Serial.println(WiFi.localIP());
}

#define PRINT_DELAY 50
bool started = false;
int lastPrintTime = millis();

void setup(){
    Serial.begin(115200);
    // while(!Serial.available()) ;
    // while(Serial.available()) Serial.read();
    Serial.println("started");

    // WiFi.softAP(ssid, password);
    // // Once connected, print IP
    // Serial.print("IP Address: ");
    // Serial.println(WiFi.softAPIP());
    initWiFi();

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        // request->send(200, "text/plain", "Webserial interface at http://" + WiFi.softAPIP().toString() + "/webserial");
        request->send(200, "text/plain", "Webserial interface at http://" + WiFi.localIP().toString() + "/webserial");
    });

    // WebSerial is accessible at "<IP Address>/webserial" in browser
    WebSerial.begin(&server);

    /* Attach Message Callback */
    WebSerial.onMessage([&](uint8_t *data, size_t len) {
        Serial.printf("Received %u bytes from WebSerial: ", len);
        Serial.write(data, len);
        Serial.println();
        WebSerial.println("Received Data...");
        String d = "";
            for(size_t i=0; i < len; i++){
            d += char(data[i]);
        }
        WebSerial.println(d);
    });

    server.begin();

    Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
}

void loop(){
    if(!started){
        Serial.println("main loop started");
        started = true;
    }
    #ifdef NO_SERIAL
    if(millis()-lastPrintTime >= PRINT_DELAY){
        WebSerial.println("hi");
        lastPrintTime = millis();
    }
    #else
    // float loopStartTime = micros();
    if(Serial1.available()>=DATA_LEN){
        while(Serial1.available()>=DATA_LEN && Serial1.peek()!=1) {
            Serial.println("first byte not 1");
            Serial1.read();
        }
        // if(Serial1.available()>=DATA_LEN && Serial1.peek()==1){
        // float startTime = micros();
        int len = Serial1.readBytes(buffer, DATA_LEN);
        // float endTime = micros();
        // Serial.printf("Time: %f \n", endTime - startTime);
        if(len!=DATA_LEN || buffer[0]!=1){
            Serial.print("Received bad data: length: ");
            Serial.print(len);
            Serial.print(", data: ");
            for (auto i : buffer) {
                Serial.print(i);
                Serial.print(" ");
            }
        }
        else{
            total++;
            int sum = 0;
            for (auto i : buffer){
                sum += i;
            }
            if(sum!=465) counter++;
            if(millis()-lastPrintTime > PRINT_DELAY){
                for (auto i : buffer){
                    WebSerial.print(String(i)+" ");
                }
                WebSerial.println();
                WebSerial.printf("Errors: %d out of %d\n", counter, total);
                WebSerial.printf("Error rate: %f percent\n", (float)counter*100/total); 
                lastPrintTime = millis();  
            }
        } 
        // }
    }
    // else{
    //     Serial.println("No data received");
    // }
    // float loopEndTime = micros();
    // Serial.printf("Loop Time: %f \n", loopEndTime - loopStartTime);
    #endif

    WebSerial.loop();
}
