/*
  Read an 8x8 array of distances from the VL53L5CX
  By: Nathan Seidle
  SparkFun Electronics
  Date: October 26, 2021
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to get all 64 pixels, at 15Hz, comma seperated output.
  This is handy for transmission to visualization programs such as Processing.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/18642
*/

/*
  VL53L5CX 8×8 grid with per-zone noise filter + EWMA smoothing
  Prints NA where σ < 0.5 mm or σ > 100 mm
  Otherwise prints the exponentially-smoothed distance (α = 0.2)
*/

/*
  VL53L5CX – average of bottom 6 rows, EWMA-smoothed
  Prints one value per frame (mm)
*/

#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>

SparkFun_VL53L5CX sensor;
VL53L5CX_ResultsData measurementData;

/* ------------ parameters ------------ */
const float ALPHA = 0.2f;     // EWMA weight (0.0 … 1.0)
/* ------------------------------------ */

float avgEWMA  = 0.0f;
bool  first    = true;

long frames = 0;
long t0;

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("VL53L5CX bottom-rows average with EWMA");

  Wire1.setSCL(7);  Wire1.setSDA(6);
  Wire1.begin();    Wire1.setClock(1'000'000);        // 1 MHz I²C

  sensor.setWireMaxPacketSize(128);
  while (!sensor.begin((byte)41U, Wire1)) {
    Serial.println(F("Sensor not found – check wiring"));
    delay(10);
  }

  sensor.setResolution(8 * 8);                        // full 64-zone grid
  sensor.setRangingFrequency(10);                     // 10 Hz (≤ 15 Hz)
  sensor.setTargetOrder(SF_VL53L5CX_TARGET_ORDER::CLOSEST);
  sensor.startRanging();

  t0 = millis();
}

void loop()
{
  if (!sensor.isDataReady())               return;
  if (!sensor.getRangingData(&measurementData)) return;

  /* ---- average bottom-six rows (rows 0-5) ---- */
  long sum = 0;
  for (int row = 0; row <= 5; ++row) {               // rows 0 … 5
    for (int col = 0; col < 8; ++col) {
      sum += measurementData.distance_mm[row * 8 + col];
    }
  }
  const float rawAvg = sum / 48.0f;                  // 48 cells

  /* ---- EWMA update ---- */
  if (first) {
    avgEWMA = rawAvg;
    first   = false;
  } else {
    avgEWMA = ALPHA * rawAvg + (1.0f - ALPHA) * avgEWMA;
  }

  /* ---- print ---- */
  // Serial.print("Bottom-6-row mean (EWMA, alpha = ");
  // Serial.print(ALPHA, 2);
  // Serial.print("): ");
  Serial.print(avgEWMA, 1);                          // one decimal place
  Serial.println(" mm");

  /* optional fps counter */
  frames++;
  float dt = (millis() - t0) / 1000.0f;
  if (frames % 100 == 0) {                           // every ~10 s @ 10 Hz
    Serial.print("fps: ");
    Serial.println(frames / dt, 2);
  }
}


//noise and movivng averag lidar vals code
// #include <Wire.h>
// #include <SparkFun_VL53L5CX_Library.h>

// SparkFun_VL53L5CX sensor;
// VL53L5CX_ResultsData measurementData;

// /* ───────────── Parameters ───────────── */
// const uint8_t NOISE_WINDOW = 10;          // frames kept for σ
// const float   SIGMA_MIN    = 0.5f;       // frozen threshold
// const float   SIGMA_MAX    = 500.0f;     // noisy  threshold
// const float   ALPHA        = 0.2f;       // EWMA weight
// /* ───────────────────────────────────────*/

// /* ───── Rolling history & EWMA storage ───── */
// int16_t hist[64][NOISE_WINDOW];
// uint8_t histPos  = 0;
// bool    histFull = false;

// float   sigma[64];
// float   ewma[64];
// bool    firstFrame = true;
// /* ────────────────────────────────────────── */

// int imageWidth = 8;
// long frames = 0;
// long t0;

// /* Pretty printer with validity mask */
// void printGrid(const int16_t *vals, const bool *valid,
//                int w, bool flipV, bool flipH)
// {
//   Serial.println("==================================================================");
//   for (int r = 0; r < w; ++r) {
//     int row = flipV ? (w - 1 - r) : r;
//     Serial.print("||");
//     for (int c = 0; c < w; ++c) {
//       int col = flipH ? (w - 1 - c) : c;
//       int i   = row * w + col;

//       if (!valid[i]) {
//         Serial.print("  NA ");          // 4-char placeholder
//       } else {
//         int v = vals[i];
//         if      (v <   10) Serial.print("   ");
//         else if (v < 1000) Serial.print("  ");
//         else               Serial.print(" ");
//         Serial.print(v);
//       }
//       Serial.print("||");
//     }
//     Serial.println();
//     Serial.println("==================================================================");
//   }
//   Serial.println();
// }

// void setup()
// {
//   Serial.begin(115200);
//   delay(1000);
//   Serial.println("VL53L5CX noise-filter + EWMA demo");

//   Wire1.setSCL(7);  Wire1.setSDA(6);
//   Wire1.begin();    Wire1.setClock(1'000'000);

//   sensor.setWireMaxPacketSize(128);
//   while (!sensor.begin((byte)41U, Wire1)) {
//     Serial.println(F("Sensor not found – check wiring"));
//     delay(10);
//   }

//   sensor.setResolution(8 * 8);
//   sensor.setRangingFrequency(10);   // 10 Hz (≤ 15 Hz for 8×8)
//   // sensor.setTargetOrder(SF_VL53L5CX_TARGET_ORDER::CLOSEST);
//   sensor.startRanging();

//   t0 = millis();
// }

// void loop()
// {
//   if (!sensor.isDataReady())               return;
//   if (!sensor.getRangingData(&measurementData)) return;

//   /* ── store current frame in circular buffer ─────────── */
//   for (int i = 0; i < 64; ++i) hist[i][histPos] = measurementData.distance_mm[i];
//   histPos = (histPos + 1) % NOISE_WINDOW;
//   if (histPos == 0) histFull = true;

//   /* ── compute σ per zone ─────────────────────────────── */
//   uint8_t N = histFull ? NOISE_WINDOW : histPos;   // samples available
//   for (int i = 0; i < 64; ++i) {
//     if (N < 2) { sigma[i] = 0.0f; continue; }

//     float mean = 0.0f;
//     for (uint8_t k = 0; k < N; ++k) mean += hist[i][k];
//     mean /= N;

//     float var = 0.0f;
//     for (uint8_t k = 0; k < N; ++k) {
//       float d = hist[i][k] - mean;
//       var += d * d;
//     }
//     sigma[i] = sqrt(var / (N - 1));
//   }

//   /* ── update EWMA & validity mask ────────────────────── */
//   static bool   valid[64];
//   static int16_t toPrint[64];

//   for (int i = 0; i < 64; ++i) {
//     bool ok = (sigma[i] >= SIGMA_MIN && sigma[i] <= SIGMA_MAX);
//     valid[i] = ok;

//     if (firstFrame) {
//       ewma[i] = measurementData.distance_mm[i];   // seed EWMA
//     } else if (ok) {
//       ewma[i] = ALPHA * measurementData.distance_mm[i]
//               + (1.0f - ALPHA) * ewma[i];
//     } // else keep previous ewma[i]

//     toPrint[i] = int16_t(ewma[i] + 0.5f);         // round for display
//   }
//   firstFrame = false;

//   /* ── print grid ─────────────────────────────────────── */
//   printGrid(toPrint, valid, imageWidth, /*flipV=*/true, /*flipH=*/false);

//   /* ── fps counter (optional) ─────────────────────────── */
//   frames++;
//   float dt = (millis() - t0) / 1000.0f;
//   Serial.print("fps: "); Serial.println(frames / dt, 2);
// }

//noise tracking code
// #include <Wire.h>
// #include <SparkFun_VL53L5CX_Library.h>

// SparkFun_VL53L5CX sensor;
// VL53L5CX_ResultsData measurementData;

// /* ───── Noise-tracking parameters ─────────────────────────── */
// const uint8_t NOISE_WINDOW = 5;                 // history length
// int16_t hist[64][NOISE_WINDOW];                 // circular buffer
// uint8_t histPos   = 0;                          // write index
// bool    histFull  = false;                      // have we wrapped yet?
// float   sigma[64];                              // latest σ per zone
// /* ─────────────────────────────────────────────────────────── */

// int imageWidth;
// long frames = 0;
// long t0;

// void printGrid(int16_t arr[], int w, bool flipV, bool flipH)
// {
//   Serial.println("==================================================================");
//   for (int r = 0; r < w; r++) {
//     int row = flipV ? (w - 1 - r) : r;
//     Serial.print("||");
//     for (int c = 0; c < w; c++) {
//       int col = flipH ? (w - 1 - c) : c;
//       int val = arr[row * w + col];
//       if      (val <   10) Serial.print("   ");
//       else if (val < 1000) Serial.print("  ");
//       else                 Serial.print(" ");
//       Serial.print(val);
//       if (val < 100) Serial.print("  "); else Serial.print(" ");
//       Serial.print("||");
//     }
//     Serial.println();
//     Serial.println("==================================================================");
//   }
//   Serial.println();
// }

// void setup()
// {
//   Serial.begin(115200);
//   delay(1000);
//   Serial.println("VL53L5CX noise-map demo");

//   Wire1.setSCL(7); Wire1.setSDA(6);
//   Wire1.begin();  Wire1.setClock(1'000'000);

//   sensor.setWireMaxPacketSize(128);
//   while (!sensor.begin((byte)41U, Wire1)) {
//     Serial.println(F("Sensor not found…"));
//     delay(10);
//   }

//   sensor.setResolution(8 * 8);
//   imageWidth = 8;
//   sensor.setRangingFrequency(10);                 // 10 Hz
//   sensor.startRanging();

//   t0 = millis();
// }

// void loop()
// {
//   if (!sensor.isDataReady()) return;
//   if (!sensor.getRangingData(&measurementData))   return;

//   /* ── store reading in circular buffer ─────────────────── */
//   for (int i = 0; i < 64; ++i) hist[i][histPos] = measurementData.distance_mm[i];
//   histPos = (histPos + 1) % NOISE_WINDOW;
//   if (histPos == 0) histFull = true;

//   /* ── compute σ for each zone ───────────────────────────── */
//   uint8_t N = histFull ? NOISE_WINDOW : histPos;          // samples available
//   for (int i = 0; i < 64; ++i) {
//     if (N < 2) { sigma[i] = 0; continue; }               // need ≥2 to define σ

//     float mean = 0;
//     for (uint8_t k = 0; k < N; ++k) mean += hist[i][k];
//     mean /= N;

//     float var = 0;
//     for (uint8_t k = 0; k < N; ++k) {
//       float d = hist[i][k] - mean;
//       var += d * d;
//     }
//     var /= (N - 1);                                      // sample variance
//     sigma[i] = sqrt(var);                                // standard deviation
//   }

//   /* ── print σ-grid (rounded to mm) ──────────────────────── */
//   static int16_t toPrint[64];
//   for (int i = 0; i < 64; ++i) toPrint[i] = (int16_t)(sigma[i] + 0.5f);
//   printGrid(toPrint, imageWidth, /*flipV=*/true, /*flipH=*/false);

//   /* ── optional: fps counter ─────────────────────────────── */
//   frames++;
//   float dt = (millis() - t0) / 1000.0f;
//   Serial.print("fps: "); Serial.println(frames / dt, 2);
// }
