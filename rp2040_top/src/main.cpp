#include <Lidar.h>
#include <vector>
#include <Adafruit_NeoPixel.h>
#include <RotatingCalipers.h>
#include <CommonUtils.h>
#include <IMU.h>

#define DEBUG(x) Serial.print(#x); Serial.print(": "); Serial.println(x);

// #define PRINT_LIDARS
// #define PRINT_RECT
#define PRINT_COORDS
#define PRINT_DUMMY_POINTS
#define PRINT_HEADING
#define PRINT_IMU

#define FIELD_WIDTH 1.82f
#define FIELD_HEIGHT 2.43f
#define FIELD_AREA 4.4226f
#define FIELD_MIN_THRESH 0.8
#define FIELD_MAX_THRESH 1.2

#define PICO_LED 16
#define PICO_LED_BRIGHTNESS 50
Adafruit_NeoPixel pico_led(1, PICO_LED, NEO_GRB + NEO_KHZ800);

#define STRIP_LED 28
#define STRIP_COUNT 24
#define STRIP_BRIGHTNESS 100
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

#define NUM_POINTS 28
Point coords[NUM_POINTS], hull[NUM_POINTS];
float prev_heading = 0.0f, temp_heading = 0.0f;
Point prev_coords = {0.0f, 0.0f};
Corners dummyPoints;

#define LIDAR_ANGLE_WEIGHT 0.03f
#define IMU_ANGLE_WEIGHT 0.97f
// weights should add up to 1.0

#define TX_PIN 0
#define RX_PIN 1

#define CS0_PIN 5
#define MISO0_PIN 4 // RX
#define MOSI0_PIN 3 // TX
#define SCK0_PIN 2

#define CS1_PIN 13
#define MISO1_PIN 12 // RX
#define MOSI1_PIN 15 // TX
#define SCK1_PIN 14

#define TARE_BUTTON 29
IMU imu0(MOSI0_PIN, MISO0_PIN, SCK0_PIN, CS0_PIN, SPI);
IMU imu1(MOSI1_PIN, MISO1_PIN, SCK1_PIN, CS1_PIN, SPI1);

spin_lock_t *imuLock;
float imu0_heading = 0.0f, prev_imu0_heading, imu1_heading, prev_imu1_heading; // access only on core 0
float shared_imu0 = 0.0f, shared_imu1 = 0.0f; // use mutex for accessing on both cores

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
        lidar.emplace_back(scl[bus], sda[bus], i+1, angle[i], calib[i]+DIST_FROM_CENTRE);
    }

    for (int i=0; i<2; i++){
        Analog_IIC_Init(scl[i], sda[i]);
    }

    Serial1.setRX(RX_PIN);
    Serial1.setTX(TX_PIN);
    Serial1.begin(115200);

    imuLock = spin_lock_instance(0);

    strip.begin();
    strip.setBrightness(STRIP_BRIGHTNESS);
    strip.setPixelColor(0, pico_led.Color(15, 15, 0));
    strip.show();

    pico_led.begin();
    pico_led.setBrightness(PICO_LED_BRIGHTNESS);
    pico_led.setPixelColor(0, pico_led.Color(15, 15, 0));
    pico_led.show();
}

void setup1(){
    pinMode(TARE_BUTTON, INPUT);
    imu0.init();
    imu1.init();
}

void loop(){
    pico_led.setPixelColor(0, pico_led.Color(15, 0, 0));
    pico_led.show();

    if(digitalRead(TARE_BUTTON)==HIGH){
        prev_heading = 0.0f;
        temp_heading = 0.0f;
    }

    if (!is_spin_locked(imuLock)) {  
        uint32_t irq_state = spin_lock_blocking(imuLock);
        imu0_heading = shared_imu0;
        imu1_heading = shared_imu1;
        spin_unlock(imuLock, irq_state);

        #ifdef PRINT_IMU
        Serial.print("IMU heading: ");
        Serial.print(imu0_heading);
        Serial.print("\t");
        Serial.println(imu1_heading);
        #endif
    }

    float angle_diff = (imu0_heading - prev_imu0_heading + imu1_heading - prev_imu1_heading)/2;
    temp_heading = angle_diff + prev_heading;

    #ifdef PRINT_LIDARS
    Serial.print("Lidar coordinates: {");
    #endif

    for (int i=0; i<NUM_LIDARS; i++){
        // distRaw[i] = lidar[i].readRaw();
        coords[i] = lidar[i].readCoords();
        if(lidar[i].buffer.dis > MAX_LIDAR_DIST) strip.setPixelColor(i, strip.Color(15, 0, 15));
        else if(lidar[i].buffer.dis<=0.10) strip.setPixelColor(i, strip.Color(15, 0, 0));
        else strip.setPixelColor(i, strip.Color(0, 15, 0));

        #ifdef PRINT_LIDARS
        Serial.print("{");
        Serial.print(coords[i].x);
        Serial.print(", ");
        Serial.print(coords[i].y);
        Serial.print("}, ");
        #endif
    }
    strip.show();

    #ifdef PRINT_LIDARS
    Serial.println("}");
    #endif

    if(prev_imu0_heading != 0 && prev_imu1_heading != 0 && prev_coords.x != 0 && prev_coords.y != 0){
        dummyPoints = getCorners(temp_heading, prev_coords);
    }

    int hullSize = convexHull(coords, NUM_POINTS, hull);
    MinAreaRect rect = findMinAreaRect(hull, hullSize);

    if((rect.area < FIELD_MIN_THRESH * FIELD_AREA)){
        coords[24] = dummyPoints.bl;
        coords[25] = dummyPoints.br;
        coords[26] = dummyPoints.tl;
        coords[27] = dummyPoints.tr;

        #ifdef PRINT_DUMMY_POINTS
        printPoint(dummyPoints.bl);
        printPoint(dummyPoints.br);
        printPoint(dummyPoints.tl);
        printPoint(dummyPoints.tr);
        Serial.println();
        Serial.println();
        #endif
        int hullSize = convexHull(coords, NUM_POINTS, hull);
        rect = findMinAreaRect(hull, hullSize);
    }
    else{
        coords[24] = {0.0f, 0.0f};
        coords[25] = {0.0f, 0.0f};
        coords[26] = {0.0f, 0.0f};
        coords[27] = {0.0f, 0.0f};
    }

    #ifdef PRINT_RECT
    Serial.print("Corners (might have been swapped): "); // BL BR TL TR
    printPoint(rect.bottom_left);
    printPoint(rect.bottom_right);
    printPoint(rect.top_left);
    printPoint(rect.top_right);
    Serial.println();

    Serial.print("dimensions");
    Serial.print(rect.width);
    Serial.print(" ");
    Serial.print(rect.height);
    Serial.print("\t");

    Serial.print("area: ");
    Serial.println(rect.area);
    #endif

    bool flip = false;
    float rotate_rect_angle = 0;
    if(rect.vector_width.x==0) {
        if(rect.vector_width.y>=0) rotate_rect_angle = PI/2;
        else rotate_rect_angle = -PI/2;
    }
    else rotate_rect_angle = atanf(rect.vector_width.y / rect.vector_width.x); // in radians
    Point unscaled_coords = rotatePoint(rect.bottom_left, rotate_rect_angle);

    #ifdef PRINT_COORDS
    Serial.print("unscaled raw coords: ");
    printPoint(unscaled_coords);
    Serial.println();
    #endif

    if(unscaled_coords.x < 0 && unscaled_coords.y < 0) {
        // Serial.println("both coord negatives - flip");
        flip = true;
        unscaled_coords.x = -unscaled_coords.x;
        unscaled_coords.y = -unscaled_coords.y;
    }
    else if(unscaled_coords.x >=0 && unscaled_coords.y >=0) flip = false;
    else Serial.println("weird coords obtained");
    Point cur_coords = scaleCoord(rect, unscaled_coords);

    #ifdef PRINT_COORDS
    Serial.print("rotate rect angle: ");
    Serial.println(DEG(rotate_rect_angle));
    #endif

    float heading = DEG(rotate_rect_angle) + (flip ? 180 : 0);
    if(heading < 0) heading += 360;
    if(heading >= 360) heading -= 360;

    float other_heading = heading + (heading<180 ? 180 : -180);
    float diff1 = abs(temp_heading - heading), diff2 = abs(temp_heading - other_heading);
    if(diff1>180) diff1 = 360 - diff1;
    if(diff2>180) diff2 = 360 - diff2;
    float lidar_heading = heading;
    bool swapped = diff1 <= diff2 ? false : true;
    if(swapped){
        lidar_heading = other_heading;
        cur_coords.x = FIELD_WIDTH - cur_coords.x;
        cur_coords.y = FIELD_HEIGHT - cur_coords.y;
    }
    LIM_ANGLE_180(lidar_heading);

    if(rect.area > FIELD_MAX_THRESH * FIELD_AREA){
        Serial.println("bad area obtained");
        cur_coords = prev_coords;
        // while(1) ;
    }
    
    #ifdef PRINT_COORDS
    Serial.print("lidar heading: ");
    Serial.print(lidar_heading);
    Serial.println();

    Serial.print("coordinates: ");
    Serial.print("{");
    Serial.print(abs(cur_coords.x));
    Serial.print(", ");
    Serial.print(abs(cur_coords.y));
    Serial.println("}");
    #endif

    float weightedX = sinf(RAD(lidar_heading)) * LIDAR_ANGLE_WEIGHT + sinf(RAD(temp_heading)) * IMU_ANGLE_WEIGHT;
    float weightedY = cosf(RAD(lidar_heading)) * LIDAR_ANGLE_WEIGHT + cosf(RAD(temp_heading)) * IMU_ANGLE_WEIGHT;
    float final_heading = DEG(atan2(weightedX, weightedY));
    LIM_ANGLE_180(final_heading);

    #ifdef PRINT_HEADING
    Serial.print("temp heading: ");
    Serial.println(temp_heading);
    Serial.print("final angle: ");
    Serial.print(final_heading);
    Serial.println();
    #endif

    prev_coords = cur_coords;
    prev_heading = final_heading;
    prev_imu0_heading = imu0_heading;
    prev_imu1_heading = imu1_heading;

    int rounded_coord_x = floor(cur_coords.x * 128);
    int rounded_coord_y = floor(cur_coords.y * 128);
    int uart_heading = floor(final_heading * 128);
    // int rounded_imu_heading = floor(abs(imu_heading) * 128);

    Serial1.write(1);
    Serial1.write(rounded_coord_x & 0xFF);
    Serial1.write((rounded_coord_x >> 8) & 0xFF);
    Serial1.write(rounded_coord_y & 0xFF);
    Serial1.write((rounded_coord_y >> 8) & 0xFF);
    Serial1.write(uart_heading & 0xFF);
    Serial1.write((uart_heading >> 8) & 0xFF);

    // if(copysign(1, imu_heading)==1) Serial1.write(1);
    // else Serial1.write((uint8_t)0);
    // Serial1.write(rounded_imu_heading & 0xFF);
    // Serial1.write((rounded_imu_heading >> 8) & 0xFF);

    Serial.println();
}

void loop1(){
    // if(digitalRead(TARE_BUTTON)==HIGH){
    //     imu0.tareAll();
    //     imu1.tareAll();
    // }
    imu0.updateAllData();
    imu1.updateAllData();

    if (!is_spin_locked(imuLock)) {  
        uint32_t irq_state = spin_lock_blocking(imuLock);
        shared_imu0 = imu0.yaw;
        shared_imu1 = imu1.yaw;
        spin_unlock(imuLock, irq_state);
    }

    if(abs(imu0.roll)>=10 || abs(imu0.pitch)>=10 || abs(imu1.roll)>=10 || abs(imu1.pitch)>=10){
        pico_led.setPixelColor(0, pico_led.Color(0, 15, 0));
        pico_led.show();
    }
}
