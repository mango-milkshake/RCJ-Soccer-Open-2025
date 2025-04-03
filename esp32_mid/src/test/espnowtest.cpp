#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>

uint8_t broadcastAddress[6] = {0,0,0,0,0,0}; 
uint8_t own_mac_address[6];
typedef struct struct_message {
    int isPresent;
} struct_message;
struct_message espnowData;
struct_message espnowDataRecv;


void readMacAddress(){ //read own mac address and set broadcast address to other bot

    esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, own_mac_address);
    if (ret == ESP_OK) {
    // Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x\n",
    //               own_mac_address[0], own_mac_address[1], own_mac_address[2],
    //               own_mac_address[3], own_mac_address[4], own_mac_address[5]);
    // } 
    // else{
    //     Serial.println("Failed to read MAC address");
    // }
    const uint8_t MAC_1[6] = {0x3c, 0x84, 0x27, 0x26, 0x03, 0x14};
    const uint8_t MAC_2[6] = {0x3c, 0x84, 0x27, 0x26, 0x02, 0x48};
    if (memcmp(own_mac_address, MAC_1, 6) == 0){
        memcpy(broadcastAddress, MAC_2, 6);
    }
    else if (memcmp(own_mac_address, MAC_2, 6) == 0){
        memcpy(broadcastAddress, MAC_1, 6);
    }
    }
}

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status){  
    Serial.print("\r\nLast Packet Send Status:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void onDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len){ //interpret received data here
    memcpy(&espnowDataRecv, incomingData, sizeof(espnowDataRecv));
    Serial.println(espnowDataRecv.isPresent);
}

void set_up_esp_now(){

    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    //callback functions for sending and receiving
    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataRecv);
    
    // Register peer
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    
    // Add peer        
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
    }
}

void sendData(){ //send data here
    //Define what values to send
    espnowData.isPresent = 2;

    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&espnowData, sizeof(espnowData));

    if (result == ESP_OK) {
        Serial.println("✅ ESP-NOW: Data sent successfully.");
    } else {
        Serial.print("❌ ESP-NOW: Send failed, error code: ");
        Serial.println(result);
    }
}

void setup(){
    Serial.begin(115200);
    readMacAddress();
    set_up_esp_now();
}

void loop(){
    sendData();
    Serial.println(espnowDataRecv.isPresent);
}