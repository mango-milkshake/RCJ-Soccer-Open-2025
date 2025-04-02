/*
Common errors:
Could not connect to COM, port does not exist: Stop monitoring before uploading code
Fatal error: serial data not received: unplug, hold down boot, plug in
Serial not updating: hold down reset, hold and release boot, wait 1 second, release reset
*/

#include <esp_now.h>
#include <WiFi.h>

//put RECEIVER mac address here
uint8_t broadcastAddress[] = {0x34, 0x85, 0x18, 0xbc, 0xe0, 0x60};

// Must match the receiver structure!!
typedef struct struct_message {
  //sends 3 ints
  int a;
  int b;
  int c;
} struct_message;

// Create a struct_message called myData
struct_message myData;

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
 
void setup() {
    delay(5000);
  // Init Serial Monitor
  Serial.begin(115200);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Transmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}
 
void loop() {
  //Serial.println("aa");
  // Set values to send

  //cap at 50 cuz the led was hurting my eyes
  myData.a = random(1,50);
  myData.b = random(1,50);
  myData.c = random(1,50);


  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
    
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  delay(400);
}