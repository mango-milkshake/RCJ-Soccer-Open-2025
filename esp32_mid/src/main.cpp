#include <Arduino.h>
#include <Wire.h>
#include <PID.h>
#include <CommonUtils.h>
#include <Adafruit_NeoPixel.h>

#define LED_PIN 18
#define LED_COUNT 10
#define LED_BRIGHTNESS 50
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

#define DEBUG(x) Serial.println(String(#x) + String(": ") + String(x) + String('\r')); 
#define TURN_OFF_SW 40
bool turnOff = false;

#define SDA_PIN 8
#define SCL_PIN 9
#define I2C_RCV_DATA_LEN 32
#define I2C_SEND_DATA_LEN 9
#define I2C_RCV_PICO_ADDR 0x08
#define I2C_SEND_PICO_ADDR 0x09

#define PICO_TX_PIN 16
#define PICO_RX_PIN 17
#define PICO_SERIAL_DATA_LEN 10

#define CAM_TX_PIN 10
#define CAM_RX_PIN 11
#define CAM_SERIAL_DATA_LEN 5

HardwareSerial Seriall0(0);
HardwareSerial Seriall1(1);
HardwareSerial Seriall2(2);

byte uartBufferPico[PICO_SERIAL_DATA_LEN];
byte uartBufferCam[CAM_SERIAL_DATA_LEN];

byte rcvBuffer[I2C_RCV_DATA_LEN+1], sendBuffer[I2C_SEND_DATA_LEN];
byte zeroBuffer[I2C_SEND_DATA_LEN];
SemaphoreHandle_t i2cMutex, coordMutex, ballMutex;

PID pid_rotate(2, 0, 0, 5000);
// PID pid_speed(5, 0, 0, 5000);
PID pid_x(10, 0, 0, 5000);
PID pid_y(10, 0, 0, 5000);

#define FIELD_WIDTH 1.82 // 0.91
#define FIELD_HEIGHT 2.43 // 1.21
float cur_x, cur_y, cur_lidar_heading, cur_imu_heading;
float cur_ball_x = 0, cur_ball_y = 0;

// core 0 handles main game logic and writing motor control info to rp2040
void core0Task(void *pvParameters){
    float self_x = 0, self_y = 0, self_lidar_heading = 0, self_imu_heading = 0;
    float speed_xdir, speed_ydir, angle, rotation;
    float target_x = 0, target_y = 0;
    zeroBuffer[0] = 0;
    for (int i=1; i<I2C_SEND_DATA_LEN; i++) zeroBuffer[i] = 0;
    while(1){
        // Serial.print("Core0");
        if(digitalRead(TURN_OFF_SW)==HIGH) turnOff = true;
        else turnOff = false;
        if(xSemaphoreTake(coordMutex, 0)){
            self_x = cur_x;
            self_y = cur_y;
            self_lidar_heading = cur_lidar_heading;
            self_imu_heading = cur_imu_heading;
            xSemaphoreGive(coordMutex);
            // Serial.println("Updated coordinates");
            // DEBUG(self_x);
            // DEBUG(self_y);
        }

        if(xSemaphoreTake(ballMutex, 0)){
            if(cur_ball_x==0 && cur_ball_y==0){
                target_x = 0.80;
                target_y = 0.91;
            }
            else{
                target_x = self_x + cur_ball_x;
                target_y = self_y + cur_ball_y - 8.5;
            }
            xSemaphoreGive(ballMutex);
            // DEBUG(target_x);
            // DEBUG(target_y);
        }

        float x_dist = target_x - self_x, y_dist = target_y - self_y;
        speed_xdir = constrain(pid_x.compute(0, x_dist), -1, 1);
        speed_ydir = constrain(pid_y.compute(0, y_dist), -1, 1);
        // float distance = sqrt(x_dist * x_dist + y_dist * y_dist);

        if(x_dist==0) {
            if(y_dist>=0) angle = 90;
            else angle = 270;
         }
         else if(y_dist==0){
             if(x_dist>=0) angle = 0;
             else angle = 180;
         }
         else{
             angle = DEG(atanf(abs(y_dist / x_dist)));
             if(y_dist>0 && x_dist<0) angle = 180 - angle;
             else if(y_dist<0 && x_dist<0) angle = 180 + angle;
             else if(y_dist<0 && x_dist>0) angle = 360 - angle;
         }

        angle = 90 - angle;
        if(angle<0) angle += 360;
        if(angle>=360) angle -= 360;
        rotation = constrain(pid_rotate.compute(0, RAD(self_imu_heading)), -1, 1);
        // speed = constrain(pid_speed.compute(0, distance), -1, 1);
        // DEBUG(rotation);
        // DEBUG(speed); 
        // DEBUG(angle);

        angle = angle - self_imu_heading;
        if(angle<0) angle += 360;
        if(angle>=360) angle -= 360;

        float calc_angle = 90 - angle;
        if(calc_angle<0) calc_angle += 360;
        if(calc_angle>=360) calc_angle -= 360;

        // DEBUG(speed_xdir);
        // DEBUG(speed_ydir);

        // DEBUG(speedX);
        // DEBUG(speedY);
        
        uint8_t rotation_sign, speed_x_sign, speed_y_sign;
        if(copysign(1, rotation)==1) rotation_sign = 1;
        else rotation_sign = 0;
        if(copysign(1, speed_xdir)==1) speed_x_sign = 1;
        else speed_x_sign = 0;
        if(copysign(1, speed_ydir)==1) speed_y_sign = 1;
        else speed_y_sign = 0;
        uint8_t rounded_rotation = floor(abs(rotation) * 255);
        uint8_t rounded_speed_x = floor(abs(speed_xdir) * 255);
        uint8_t rounded_speed_y = floor(abs(speed_ydir) * 255);
        int rounded_angle = floor(angle * 128);

        sendBuffer[0] = 5;
        sendBuffer[1] = speed_x_sign;
        sendBuffer[2] = rounded_speed_x;
        sendBuffer[3] = speed_y_sign;
        sendBuffer[4] = rounded_speed_y;
        sendBuffer[5] = rotation_sign;
        sendBuffer[6] = rounded_rotation;
        sendBuffer[7] = (rounded_angle & 0xFF);
        sendBuffer[8] = ((rounded_angle >> 8) & 0xFF);

        if(xSemaphoreTake(i2cMutex, portMAX_DELAY)){
            Wire.beginTransmission(I2C_SEND_PICO_ADDR);
            if(turnOff) {
                // Serial.println("Bot off");
                Wire.write(zeroBuffer, I2C_SEND_DATA_LEN);
            }
            else Wire.write(sendBuffer, I2C_SEND_DATA_LEN);
            Wire.endTransmission();
            xSemaphoreGive(i2cMutex);
            // Serial.println("Data sent");
        }
    }
}

// core 1 handles receiving data and processing to get final self and ball coordinates
void core1Task(void *pvParameters){
    float coord_x = 0, coord_y = 0, lidar_heading = 0, imu_heading = 0;
    float ball_angle = 0, ball_dist = 0, ball_x = 0, ball_y = 0;
    bool no_ball = false;
    int loopcount = 0;
    while(1){
        // Serial.print("Core1");
        // Serial.println(loopcount);
        // loopcount++;
        if(Seriall1.available()>=CAM_SERIAL_DATA_LEN){
            int counter = 0;
            while(Seriall1.peek()!=1) {
                // Serial.println("Cam first byte not 1");
                Seriall1.read();
                counter++;
                // Serial.println(counter);
                // if(counter>=10) break;
            }
            int len = Seriall1.readBytes(uartBufferCam, CAM_SERIAL_DATA_LEN);
            // while(Seriall1.available()) Seriall1.read();
            if(len!=CAM_SERIAL_DATA_LEN || uartBufferCam[0]!=1){
                Serial.print("Received bad data: length: ");
                Serial.print(len);
                Serial.print(", data: ");
                for (auto i : uartBufferCam) {
                    Serial.print(i);
                    Serial.print(" ");
                }
            }
            else{
                ball_angle = (float)(uartBufferCam[1] + (uartBufferCam[2]<<8)) / 128;
                ball_dist = (float)(uartBufferCam[3] + (uartBufferCam[4]<<8)) / 128;
                if(ball_angle==0 && ball_dist==0) no_ball = true;
                else no_ball = false;
                DEBUG(ball_angle);
                DEBUG(ball_dist);

                float relative_angle = 90 - (ball_angle + imu_heading);
                ball_x = (ball_dist * cosf(relative_angle)) / 100;
                ball_y = (ball_dist * sinf(relative_angle)) / 100;

                if(no_ball){
                    strip.setPixelColor(0, strip.Color(0, 0, 15));
                    strip.show();
                }
                else{
                    strip.setPixelColor(0, strip.Color(0, 15, 0));
                    strip.show();
                }

                if(xSemaphoreTake(ballMutex, portMAX_DELAY)){
                    if(no_ball){
                        cur_ball_x = 0;
                        cur_ball_y = 0;
                    }
                    else{
                        cur_ball_x = ball_x;
                        cur_ball_y = ball_y;
                    }
                    xSemaphoreGive(ballMutex);
                    // DEBUG(ball_x);
                    // DEBUG(ball_y);
                    // Serial.println("Ball data updated");
                }
                // for (auto i : uartBufferCam){
                //     Serial.print(i);
                //     Serial.print(" ");
                // }
            }
            // Serial.println();
        }
        // else{
        //     Serial.println("No data received");
        // }
        
        if(Seriall2.available()>=PICO_SERIAL_DATA_LEN){
            int counter = 0;
            while(Seriall2.peek()!=1) {
                Serial.println("Pico first byte not 1");
                Seriall2.read();
                counter++;
                // if(counter>=PICO_SERIAL_DATA_LEN) break;
            }
            int len = Seriall2.readBytes(uartBufferPico, PICO_SERIAL_DATA_LEN);
            if(len!=PICO_SERIAL_DATA_LEN || uartBufferPico[0]!=1){
                Serial.print("Received bad data: length: ");
                Serial.print(len);
                Serial.print(", data: ");
                for (auto i : uartBufferPico) {
                    Serial.print(i);
                    Serial.print(" ");
                }
            }
            else{
                coord_x = (float)(uartBufferPico[1] + (uartBufferPico[2]<<8)) / 128;
                coord_y = (float)(uartBufferPico[3] + (uartBufferPico[4]<<8)) / 128;
                lidar_heading = (float)(uartBufferPico[5] + (uartBufferPico[6]<<8)) / 128;

                imu_heading = (float)(uartBufferPico[8] + (uartBufferPico[9]<<8)) / 128;
                if(uartBufferPico[7]==0) imu_heading *= -1;

                // DEBUG(coord_x);
                // DEBUG(coord_y);
                // DEBUG(lidar_heading);
                // DEBUG(imu_heading);
                // Serial.print(coord_x, 3);
                // Serial.print("\t");
                // Serial.print(coord_y, 3);
                // Serial.print("\t");
                // Serial.print(lidar_heading, 3);
                // Serial.print("\t");
                // Serial.print(imu_heading, 3);

                if(xSemaphoreTake(coordMutex, portMAX_DELAY)){
                    cur_x = coord_x;
                    cur_y = coord_y;
                    cur_lidar_heading = lidar_heading;
                    cur_imu_heading = imu_heading;
                    xSemaphoreGive(coordMutex);
                    // Serial.println("Coordinate data updated");
                }
                // for (auto i : uartBufferPico){
                //     Serial.print(i);
                //     Serial.print(" ");
                // }
            }
            // Serial.println();
        }
        // else{
        //     Serial.println("No data received");
        // }
    }
}

// core 1 handles reading data from cameras and top and bottom plates, and sensor fusion
void setup(){
    Serial.begin(115200);

    Seriall1.begin(115200, SERIAL_8N1, CAM_RX_PIN, CAM_TX_PIN);
    Seriall2.begin(115200, SERIAL_8N1, PICO_RX_PIN, PICO_TX_PIN);

    pinMode(TURN_OFF_SW, INPUT);

    i2cMutex = xSemaphoreCreateMutex(); 
    coordMutex = xSemaphoreCreateMutex();
    ballMutex = xSemaphoreCreateMutex();
    Wire.begin(SDA_PIN, SCL_PIN, 100000);
    // Serial.println("finished Wire setup");

    // while(!Serial.available()) ;
    // while(Serial.available()) Serial.read();
    // Serial.println("started");

    strip.begin();
    strip.setBrightness(LED_BRIGHTNESS);
    strip.show();

    xTaskCreatePinnedToCore(core0Task, "Read Data", 16384, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(core1Task, "Send Data", 16384, NULL, 1, NULL, 1);
}

void loop(){
    // Serial.println("running");
}
