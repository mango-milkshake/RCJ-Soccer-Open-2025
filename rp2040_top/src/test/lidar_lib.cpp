#include <Lidar.h>
#include <chrono>
#include <vector>
#include <Adafruit_NeoPixel.h>
#include <RotatingCalipers.h>
#include <CommonUtils.h>

#define DEBUGGING

#ifndef DEBUGGING
#define Serial.print(x) 123;
#define Serial.println(x) 123;
#endif

#define READ_RAW_LIDARS
// #define READ_COORDS
// #define READ_RECT

#define led_pin 16
#define led_count 1
#define brightness 50
Adafruit_NeoPixel strip(led_count, led_pin, NEO_GRB + NEO_KHZ800);

#define NUM_LIDARS 24
#define NUM_EACH_BUS 12
#define LIDAR_DIST_FROM_CENTRE 0.065f
float distRaw[NUM_LIDARS];
std::vector<Lidar> lidar;
uint8_t scl[2] = {9, 11}, sda[2] = {8, 10};
float angle[NUM_LIDARS];
float calib[NUM_LIDARS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

#define NUM_POINTS 24
Point coords[NUM_LIDARS], hull[NUM_LIDARS];
double fps = 0;

void printPoint(Point p){
    Serial.print("{");
    Serial.print(p.x);
    Serial.print(", ");
    Serial.print(p.y);
    Serial.print("}\t");
}

void setup(){
    Serial.begin(115200);

    for (uint8_t i=0; i<NUM_LIDARS; i++){
        angle[i] = (90 - i*15);
        if(angle[i]<0) angle[i] += 360;
        if(angle[i]>=360) angle[i] -= 360;
        uint8_t bus;
        if(i%2==0) bus = 0;
        else bus = 1;
        lidar.emplace_back(scl[bus], sda[bus], i+1, angle[i], calib[i]+LIDAR_DIST_FROM_CENTRE);
    }

    for (int i=0; i<2; i++){
        Analog_IIC_Init(scl[i], sda[i]);
    }

    strip.begin();
    strip.setBrightness(brightness);
    strip.setPixelColor(0, strip.Color(0, 15, 0));
    strip.show();
}

void loop(){
    strip.setPixelColor(0, strip.Color(15, 15, 0));
    strip.show();

    #ifdef READ_COORDS
    Serial.print("{");
    #endif

    for (int i=0; i<NUM_LIDARS; i++){
        // auto start_time = std::chrono::steady_clock::now();
        // if(i%2!=1) continue;
        #ifdef READ_RAW_LIDARS
        distRaw[i] = lidar[i].readRaw();
        #endif
        
        #ifdef READ_COORDS
        coords[i] = lidar[i].readCoords();
        #endif

        // auto cur_time = std::chrono::steady_clock::now();
        // std::chrono::nanoseconds diff = cur_time - start_time;
        // fps = 0.9 * fps + 0.1 * (1000000000 / diff.count());

        // Serial.print("Read Sensor ");
        Serial.print(i+1);
        // Serial.print(" - time: ");
        // Serial.println(fps);

        #ifdef READ_RAW_LIDARS
        Serial.print(" dist: ");
        Serial.print(" ");
        Serial.print(distRaw[i]);
        Serial.print("\t");
        #endif

        #ifdef READ_COORDS
        Serial.print("{");
        Serial.print(coords[i].x);
        Serial.print(", ");
        Serial.print(coords[i].y);
        Serial.print("}, ");
        #endif
    }

    #ifdef READ_RAW_LIDARS
    Serial.println();
    #endif

    #ifdef READ_COORDS
    Serial.println('}');
    #endif 

    Serial.println();

    int hullSize = convexHull(coords, NUM_POINTS, hull);
    MinAreaRect rect = findMinAreaRect(hull, hullSize);

    #ifdef READ_RECT
    Serial.print("dimensions");
    Serial.print(rect.width);
    Serial.print(" ");
    Serial.print(rect.height);
    Serial.print("\t");

    // Serial.print(rect.vector_width.x);
    // Serial.print(" ");
    // Serial.print(rect.vector_width.y);
    // Serial.print(" ");

    // Serial.print(rect.vector_width.x);
    // Serial.print(" ");
    // Serial.print(rect.vector_width.y);
    // Serial.print("\t");

    printPoint(rect.bottom_left);
    printPoint(rect.bottom_right);
    printPoint(rect.top_left);
    printPoint(rect.top_right);
    Serial.println();

    Serial.print("area: ");
    Serial.println(rect.area);
    #endif

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

    if(heading>=180) heading -= 180;
    
    #ifdef READ_RECT
    Serial.print("heading: ");
    Serial.print(heading);
    Serial.println();

    // Point cur_coords = getCoords(rect, basicAngle);
    // Serial.print("coordinates: ");
    // Serial.print("{");
    // Serial.print(cur_coords.x);
    // Serial.print(", ");
    // Serial.print(cur_coords.y);
    // Serial.print("}, ");
    // Serial.println();
    #endif
    
    delay(100);
}
