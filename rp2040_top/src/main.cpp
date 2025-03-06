#include <Lidar.h>
#include <vector>
#include <Adafruit_NeoPixel.h>
#include <RotatingCalipers.h>
#include <CommonUtils.h>

#define PICO_LED 16
#define PICO_LED_BRIGHTNESS 50
Adafruit_NeoPixel pico_led(1, PICO_LED, NEO_GRB + NEO_KHZ800);

#define STRIP_LED 28
#define STRIP_COUNT 24
#define STRIP_BRIGHTNESS 3
Adafruit_NeoPixel strip(STRIP_COUNT, STRIP_LED, NEO_GRB + NEO_KHZ800);

#define NUM_LIDARS 24
#define NUM_EACH_BUS 12
#define BUS0_SW 27
#define BUS1_SW 26
float distRaw[NUM_LIDARS];
std::vector<Lidar> lidar;
uint8_t scl[2] = {9, 11}, sda[2] = {8, 10};
float angle[NUM_LIDARS];
float calib[NUM_LIDARS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

#define NUM_POINTS 24
Point coords[NUM_LIDARS], hull[NUM_LIDARS];
float prev_heading;

#define TX_PIN 0
#define RX_PIN 1

void setup(){
    Serial.begin(115200);

    for (uint8_t i=0; i<NUM_LIDARS; i++){
        angle[i] = (90 - i*15);
        if(angle[i]<0) angle[i] += 360;
        if(angle[i]>=360) angle[i] -= 360;
        uint8_t bus;
        if(i%2==0) bus = 0;
        else bus = 1;
        lidar.emplace_back(scl[bus], sda[bus], i+1, angle[i], calib[i]+DIST_FROM_CENTRE);
    }

    for (int i=0; i<2; i++){
        Analog_IIC_Init(scl[i], sda[i]);
    }

    Serial1.setRX(RX_PIN);
    Serial1.setTX(TX_PIN);
    Serial1.begin(115200);

    strip.begin();
    strip.setBrightness(STRIP_BRIGHTNESS);
    strip.setPixelColor(0, pico_led.Color(15, 15, 0));
    strip.show();

    pico_led.begin();
    pico_led.setBrightness(PICO_LED_BRIGHTNESS);
    pico_led.setPixelColor(0, pico_led.Color(15, 15, 0));
    pico_led.show();
}

void loop(){
    pico_led.setPixelColor(0, pico_led.Color(15, 0, 0));
    pico_led.show();

    for (int i=0; i<NUM_LIDARS; i++){
        // distRaw[i] = lidar[i].readRaw();
        coords[i] = lidar[i].readCoords();
        if(lidar[i].buffer.dis<=0.10) strip.setPixelColor(i, strip.Color(15, 0, 0));
    }
    strip.show();

    int hullSize = convexHull(coords, NUM_POINTS, hull);
    MinAreaRect rect = findMinAreaRect(hull, hullSize);

    // Serial.print("dimensions");
    // Serial.print(rect.width);
    // Serial.print(" ");
    // Serial.print(rect.height);
    // Serial.print("\t");

    // Serial.print("area: ");
    // Serial.println(rect.area);

    float heading = 0, basicAngle = 0;

    if(rect.vector_width.x==0) {
       if(rect.vector_width.y>=0) heading = 90;
       else heading = 270;
       basicAngle = 90;
    }
    else if(rect.vector_width.y==0){
        if(rect.vector_width.x>=0) heading = 0;
        else heading = 180;
        basicAngle = 0;
    }
    else{
        heading = DEG(atanf(abs(rect.vector_width.y / rect.vector_width.x)));
        basicAngle = heading;
        if(rect.vector_width.y>0 && rect.vector_width.x<0) heading = 180 - heading;
        else if(rect.vector_width.y<0 && rect.vector_width.x<0) heading = 180 + heading;
        else if(rect.vector_width.y<0 && rect.vector_width.x>0) heading = 360 - heading;
    }

    float other_heading = heading + (180 ? heading<180 : -180);
    float diff1 = abs(prev_heading - heading), diff2 = abs(prev_heading - other_heading);
    float final_heading = heading ? diff1 <= diff2 : other_heading;
    int uart_heading = floor(final_heading * 128);
    prev_heading = final_heading;
    
    Serial.print("heading: ");
    Serial.print(final_heading);
    Serial.println();

    Point cur_coords = getCoords(rect, basicAngle);
    Serial.print("coordinates: ");
    Serial.print("{");
    Serial.print(cur_coords.x);
    Serial.print(", ");
    Serial.print(cur_coords.y);
    Serial.print("}, ");
    Serial.println();

    int rounded_coord_x = floor(cur_coords.x * 128);
    int rounded_coord_y = floor(cur_coords.y * 128);

    Serial1.write(1);
    Serial1.write(rounded_coord_x & 0xFF);
    Serial1.write((rounded_coord_x >> 8) & 0xFF);
    Serial1.write(rounded_coord_y & 0xFF);
    Serial1.write((rounded_coord_y >> 8) & 0xFF);
    Serial1.write(uart_heading & 0xFF);
    Serial1.write((uart_heading >> 8) & 0xFF);
    
}
