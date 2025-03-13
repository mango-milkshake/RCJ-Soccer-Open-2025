#include <Arduino.h>
#include <Wire.h>
#include <PID.h>
#include <CommonUtils.h>






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
#define SERIAL_DATA_LEN 10

//first git commit

// HardwareSerial Seriall0(0);
HardwareSerial Seriall1(1);
HardwareSerial Seriall2(2);

byte uartBuffer[SERIAL_DATA_LEN];

byte rcvBuffer[I2C_RCV_DATA_LEN+1], sendBuffer[I2C_SEND_DATA_LEN];
byte zeroBuffer[I2C_SEND_DATA_LEN];
SemaphoreHandle_t i2cMutex, coordMutex;

PID pid_rotate(0.8, 0, 0, 5000);
// PID pid_speed(5, 0, 0, 5000);
PID pid_x(5, 0, 0, 5000);
PID pid_y(5, 0, 0, 5000);

#define FIELD_WIDTH 1.82 // 0.91
#define FIELD_HEIGHT 2.43 // 1.21
float cur_x, cur_y, cur_lidar_heading, cur_imu_heading;
float target_x = FIELD_WIDTH/2, target_y = FIELD_HEIGHT/2;

// only core 0
float self_x = 0, self_y = 0, self_lidar_heading = 0, self_imu_heading = 0;
float speed_xdir, speed_ydir, angle, rotation;

// core 0 handles main game logic and writing motor control info to rp2040
void core0Task(void *pvParameters){
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
    while(1){
        // Serial.print("Core1");
        if(Seriall2.available()>=SERIAL_DATA_LEN){
            while(Seriall2.peek()!=1) {
                Serial.println("first byte not 1");
                Seriall2.read();
            }
            int len = Seriall2.readBytes(uartBuffer, SERIAL_DATA_LEN);
            if(len!=SERIAL_DATA_LEN || uartBuffer[0]!=1){
                Serial.print("Received bad data: length: ");
                Serial.print(len);
                Serial.print(", data: ");
                for (auto i : uartBuffer) {
                    Serial.print(i);
                    Serial.print(" ");
                }
            }
            else{
                float coord_x = (float)(uartBuffer[1] + (uartBuffer[2]<<8)) / 128;
                float coord_y = (float)(uartBuffer[3] + (uartBuffer[4]<<8)) / 128;
                float lidar_heading = (float)(uartBuffer[5] + (uartBuffer[6]<<8)) / 128;

                float imu_heading = (float)(uartBuffer[8] + (uartBuffer[9]<<8)) / 128;
                if(uartBuffer[7]==0) imu_heading *= -1;

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
                // for (auto i : uartBuffer){
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

    Seriall2.begin(115200, SERIAL_8N1, PICO_RX_PIN, PICO_TX_PIN);

    pinMode(TURN_OFF_SW, INPUT);

    i2cMutex = xSemaphoreCreateMutex(); 
    coordMutex = xSemaphoreCreateMutex();
    Wire.begin(SDA_PIN, SCL_PIN, 100000);
    // Serial.println("finished Wire setup");

    // while(!Serial.available()) ;
    // while(Serial.available()) Serial.read();
    // Serial.println("started");

    xTaskCreatePinnedToCore(core0Task, "Read Data", 16384, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(core1Task, "Send Data", 16384, NULL, 1, NULL, 1);
}

void loop(){
    // Serial.println("running");
}
