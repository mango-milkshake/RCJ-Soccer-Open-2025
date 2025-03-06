/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-web-server-sent-events-sse/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// Replace with your network credentials
const char* ssid = "heeheehaahaaheeheehaahaa";
const char* password = "lipo_fire";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Create an Event Source on /events
AsyncEventSource events("/events");

// Timer variables
unsigned long lastTime = 0;  
unsigned long timerDelay = 30000;

#define TX_PIN 10
#define RX_PIN 11

byte buffer[9];
float ball_angle, ball_dist, goal_angle, goal_dist;

void getSensorReadings(){
  if(Serial1.available()>=9){
        while(Serial1.peek()!=1) {
            Serial.println("first byte not 1");
            Serial1.read();
        }
        int len = Serial1.readBytes(buffer, 9);
        if(len!=9 || buffer[0]!=1){
            Serial.print("Received bad data: length: ");
            Serial.print(len);
            Serial.print(", data: ");
            for (auto i : buffer) {
                Serial.print(i);
                Serial.print(" ");
            }
        }
        else{
            ball_angle = (float)(buffer[1] + (buffer[2]<<8)) / 128;
            ball_dist = (float)(buffer[3] + (buffer[4]<<8)) / 128;
            goal_angle = (float)buffer[5] + (buffer[6]<<8) / 128;
            goal_dist = (float)buffer[7] + (buffer[8]<<8) / 128;
            Serial.print(ball_angle, 3);
            Serial.print("\t");
            Serial.print(ball_dist, 3);
            Serial.print("\t");
            Serial.print(goal_angle, 3);
            Serial.print("\t");
            Serial.print(goal_dist, 3);
        }
        Serial.println();
    }
}

// Initialize WiFi
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

String processor(const String& var){
  getSensorReadings();
  if(var == "BALL ANGLE"){
    return String(ball_angle);
  }
  else if(var == "BALL DIST"){
    return String(ball_dist);
  }
  else if(var == "GOAL ANGLE"){
    return String(goal_angle);
  }
  else if(var == "GOAL DIST"){
    return String(goal_dist);
  }
  return String();
}

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>bot data</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <link rel="stylesheet" href="https://use.fontawesome.com/releases/v5.7.2/css/all.css" integrity="sha384-fnmOCqbTlWIlj8LyTjo7mOUStjsKC4pOpQbqyi7RrhN7udi9RwhKkMHpvLbHG9Sr" crossorigin="anonymous">
  <link rel="icon" href="data:,">
  <style>
    html {font-family: Arial; display: inline-block; text-align: center;}
    p { font-size: 1.2rem;}
    body {  margin: 0;}
    .topnav { overflow: hidden; background-color: #50B8B4; color: white; font-size: 1rem; }
    .content { padding: 20px; }
    .card { background-color: white; box-shadow: 2px 2px 12px 1px rgba(140,140,140,.5); }
    .cards { max-width: 800px; margin: 0 auto; display: grid; grid-gap: 2rem; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); }
    .reading { font-size: 1.4rem; }
  </style>
</head>
<body>
  <div class="topnav">
    <h1>DATA PANEL</h1>
  </div>
  <div class="content">
    <div class="cards">
      <div class="card">
        <p> BALL ANGLE</p><p><span class="reading"><span id="ball_angle">%BALL ANGLE%</span> </span></p>
      </div>
      <div class="card">
        <p> BALL DIST</p><p><span class="reading"><span id="ball_dist">%BALL DIST%</span> </span></p>
      </div>
      <div class="card">
        <p> GOAL ANGLE</p><p><span class="reading"><span id="goal_angle">%GOAL ANGLE%</span> </span></p>
      </div>
      <div class="card">
        <p> GOAL DIST</p><p><span class="reading"><span id="goal_dist">%GOAL DIST%</span> </span></p>
      </div>
    </div>
  </div>
<script>
if (!!window.EventSource) {
 var source = new EventSource('/events');
 
 source.addEventListener('open', function(e) {
  console.log("Events Connected");
 }, false);
 source.addEventListener('error', function(e) {
  if (e.target.readyState != EventSource.OPEN) {
    console.log("Events Disconnected");
  }
 }, false);
 
 source.addEventListener('message', function(e) {
  console.log("message", e.data);
 }, false);
 
 source.addEventListener('ball_angle', function(e) {
  console.log("ball_angle", e.data);
  document.getElementById("ball_angle").innerHTML = e.data;
 }, false);
 
 source.addEventListener('ball_dist', function(e) {
  console.log("ball_dist", e.data);
  document.getElementById("ball_dist").innerHTML = e.data;
 }, false);
 
 source.addEventListener('goal_angle', function(e) {
  console.log("goal_angle", e.data);
  document.getElementById("goal_angle").innerHTML = e.data;
 }, false);

 source.addEventListener('goal_dist', function(e) {
  console.log("goal_dist", e.data);
  document.getElementById("goal_dist").innerHTML = e.data;
 }, false);
 
 function resetPosition(element){
  var xhr = new XMLHttpRequest();
  xhr.open("GET", "/"+element.id, true);
  console.log(element.id);
  xhr.send();
}
}
</script>
</body>
</html>)rawliteral";

void setup() {
  Serial.begin(115200);
  Serial.print("started");
  Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  initWiFi();


  // Handle Web Server
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html, processor);
  });

  // Handle Web Server Events
  events.onConnect([](AsyncEventSourceClient *client){
    if(client->lastId()){
      Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }
    // send event with message "hello!", id current millis
    // and set reconnect delay to 1 second
    client->send("hello!", NULL, millis(), 10000);
  });
  server.addHandler(&events);
  server.begin();
}

void loop() {
  if(Serial1.available()>=9){
    getSensorReadings();

    // Send Events to the Web Client with the Sensor Readings
        events.send("ping",NULL,millis());
    events.send(String(ball_angle).c_str(),"ball_angle",millis());
    events.send(String(ball_dist).c_str(),"ball_dist",millis());
    events.send(String(goal_angle).c_str(),"goal_angle",millis());
    events.send(String(goal_dist).c_str(),"goal_dist",millis());
    
    lastTime = millis();
  }
}