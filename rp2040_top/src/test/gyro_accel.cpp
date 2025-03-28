// #include <Arduino.h>
// #include <IMU.h>

// // #define SECOND_BUS

// #ifdef SECOND_BUS
// #define SPI SPI1
// #define CS_PIN 13
// #define MISO_PIN 12 // RX
// #define MOSI_PIN 15 // TX
// #define SCK_PIN 14
// #else
// #define CS_PIN 5
// #define MISO_PIN 4 // RX
// #define MOSI_PIN 3 // TX
// #define SCK_PIN 2
// #endif

// IMU imu(MOSI_PIN, MISO_PIN, SCK_PIN, CS_PIN, SPI);

// void setup(){
//     Serial.begin(115200);
//     while(!Serial.available()) ;
//     while(Serial.available()) Serial.read();
//     Serial.println("started");
//     imu.init();
// }
// float ax, ay, az; 

// void loop(){
//     ax = imu.readAccelX() / 835.07f;
//     ay = imu.readAccelY() / 835.07f;
//     az = imu.readAccelZ() / 835.07f;

//     Serial.print("Accel: ");
//     Serial.print(ax);
//     Serial.print(" ");
//     Serial.print(ay);
//     Serial.print(" ");
//     Serial.print(az);
//     Serial.print(" ");
//     Serial.println();
// }
// #include <Arduino.h>
// #include <IMU.h>

// // #define SECOND_BUS
// #ifdef SECOND_BUS
// #define SPI SPI1
// #define CS_PIN 13
// #define MISO_PIN 12 // RX
// #define MOSI_PIN 15 // TX
// #define SCK_PIN 14
// #else
// #define CS_PIN 5
// #define MISO_PIN 4 // RX
// #define MOSI_PIN 3 // TX
// #define SCK_PIN 2
// #endif

// IMU imu(MOSI_PIN, MISO_PIN, SCK_PIN, CS_PIN, SPI);


// float vx = 0.0, vy = 0.0, vz = 0.0;
// float xPos = 0.0, yPos = 0.0, zPos = 0.0;

// // For timing
// unsigned long prevMicros = 0;

// void setup() {
//   Serial.begin(115200);

//   while (!Serial.available());
//   while (Serial.available()) Serial.read();

//   Serial.println("started");
//   imu.init();
//   prevMicros = micros(); 
// }

// void loop() {

//   unsigned long now = micros();
//   float dt = (now - prevMicros) / 1e6; 
//   prevMicros = now;


//   float ax = imu.readAccelX() / 835.07f; 
//   float ay = imu.readAccelY() / 835.07f ;
//   float az = imu.readAccelZ() / 835.07f - 9.81;



//   vx += ax * dt;
//   vy += ay * dt;
//   vz += az * dt;


//   xPos += vx * dt;
//   yPos += vy * dt;
//   zPos += vz * dt;

// //   // Print results
// //   Serial.print("Accel (m/s^2): ");
// //   Serial.print(ax); Serial.print(", ");
// //   Serial.print(ay); Serial.print(", ");
// //   Serial.println(az);

// //   Serial.print("Vel   (m/s):   ");
// //   Serial.print(vx); Serial.print(", ");
// //   Serial.print(vy); Serial.print(", ");
// //   Serial.println(vz);

//   Serial.print("Pos   (m):     ");
//   Serial.print(xPos); Serial.print(", ");
//   Serial.print(yPos); Serial.print(", ");
//   Serial.println(zPos);
//   Serial.println();

// }
#include <Arduino.h>
#include <IMU.h>

// #define SECOND_BUS
#ifdef SECOND_BUS
#define SPI SPI1
#define CS_PIN 13
#define MISO_PIN 12 // RX
#define MOSI_PIN 15 // TX
#define SCK_PIN 14
#else
#define CS_PIN 5
#define MISO_PIN 4 // RX
#define MOSI_PIN 3 // TX
#define SCK_PIN 2
#endif

IMU imu(MOSI_PIN, MISO_PIN, SCK_PIN, CS_PIN, SPI);

// Duration (in seconds) for taring:
const float TARE_DURATION = 10.0;

// For computing dt:
unsigned long prevMicros = 0;

// Velocity (m/s) and Position (m):
float vx = 0.0f, vy = 0.0f, vz = 0.0f;
float xPos = 0.0f, yPos = 0.0f, zPos = 0.0f;

// For storing the final offset (in g):
float offsetX = 0.0f, offsetY = 0.0f, offsetZ = 0.0f;

// For accumulating readings during tare
float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;
unsigned long sampleCount = 0;

// State to mark when tare is finished
bool tareComplete = false;
unsigned long tareStartMillis = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial.available());
  while (Serial.available()) Serial.read();
  Serial.println("started");
  
  imu.init();
  prevMicros = micros();
  
  // Mark the start time of the taring phase
  tareStartMillis = millis();
}

void loop() {
  // 1) Calculate elapsed time in seconds for integration
  unsigned long now = micros();
  float dt = (now - prevMicros) / 1e6;
  prevMicros = now;

  // 2) Read raw acceleration and convert to g (based on your library’s scale)
  float ax = imu.readAccelX() / 835.07f; 
  float ay = imu.readAccelY() / 835.07f;
  float az = imu.readAccelZ() / 835.07f;

  // Check if we are still in the taring phase (first TARE_DURATION seconds)
  if (!tareComplete) {
    unsigned long elapsedTareMillis = millis() - tareStartMillis;
    if (elapsedTareMillis < (unsigned long)(TARE_DURATION * 1000)) {
      // Accumulate sums and sample count
      sumX += ax;
      sumY += ay;
      sumZ += az;
      sampleCount++;
    } else {
      // Tare phase ended — compute average offsets
      offsetX = sumX / sampleCount;
      offsetY = sumY / sampleCount;
      offsetZ = sumZ / sampleCount;
      tareComplete = true;

      Serial.println("Tare complete!");
      Serial.print("Offset (g): ");
      Serial.print(offsetX); Serial.print(", ");
      Serial.print(offsetY); Serial.print(", ");
      Serial.println(offsetZ);
    }

    // Until taring is complete, don’t integrate into velocity/position
    // because we haven’t established the offset yet.
    return;
  }

  // 3) Subtract the tare offsets
  ax -= offsetX;
  ay -= offsetY;
  az -= offsetZ;



  // 4) Integrate for velocity
  vx += ax * dt;
  vy += ay * dt;
  vz += az * dt;

  // 5) Integrate for position
  xPos += vx * dt;
  yPos += vy * dt;
  zPos += vz * dt;

//   // Print results for debugging
//   Serial.print("Accel(m/s^2): ");
//   Serial.print(ax); Serial.print(", ");
//   Serial.print(ay); Serial.print(", ");
//   Serial.println(az);

//   Serial.print("Vel(m/s): ");
//   Serial.print(vx); Serial.print(", ");
//   Serial.print(vy); Serial.print(", ");
//   Serial.println(vz);

  Serial.print("Pos(m): ");
  Serial.print(xPos); Serial.print(", ");
  Serial.print(yPos); Serial.print(", ");
  Serial.println(zPos);
  Serial.println();

  delay(50); 
}
