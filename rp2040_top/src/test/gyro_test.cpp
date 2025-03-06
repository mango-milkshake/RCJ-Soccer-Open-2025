#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU

// #define QUAT_ANIMATION // Uncomment this line to output data in the correct format for ZaneL's Node.js Quaternion animation tool: https://github.com/ZaneL/quaternion_sensor_3d_nodejs

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

// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1

ICM_20948_SPI ICM; 
#define speedMaximum 400000
SPISettings ICMSetting(speedMaximum, MSBFIRST, SPI_MODE1);

void setup()
{

  Serial.begin(115200); // Start the serial console


  while (Serial.available()) // Make sure the serial RX buffer is empty
    Serial.read();

  while (!Serial.available()) // Wait for the user to press a key (send any serial character)
    ;
  
  SPI.setRX(MISO_PIN);
  SPI.setTX(MOSI_PIN);
  SPI.setSCK(SCK_PIN);
  SPI.setCS(CS_PIN);
  SPI.begin();

  ICM.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  bool initialized = false;
  while (!initialized)
  {
    ICM.begin(CS_PIN, SPI);

    Serial.print(F("Initialization of the sensor returned: "));
    Serial.println(ICM.statusString());

    if (ICM.status != ICM_20948_Stat_Ok)
    {

      Serial.println(F("Trying again..."));

      delay(500);
    }
    else
    {
      initialized = true;
    }
  }

  Serial.println(F("Device connected!"));

  bool success = true; // Use success to show if the DMP configuration was successful

  // Initialize the DMP. initializeDMP is a weak function. You can overwrite it if you want to e.g. to change the sample rate
  success &= (ICM.initializeDMP() == ICM_20948_Stat_Ok);

  // Enable the DMP Game Rotation Vector sensor
  success &= (ICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);

  // Enable any additional sensors / features
  // success &= (ICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok);
  // success &= (ICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_Stat_Ok);
  // success &= (ICM.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_Stat_Ok);

  // Configuring DMP to output data at multiple ODRs:
  // DMP is capable of outputting multiple sensor data at different rates to FIFO.
  // Setting value can be calculated as follows:
  // Value = (DMP running rate / ODR ) - 1
  // E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
  success &= (ICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  // success &= (ICM.setDMPODRrate(DMP_ODR_Reg_Accel, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  // success &= (ICM.setDMPODRrate(DMP_ODR_Reg_Gyro, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  // success &= (ICM.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  // success &= (ICM.setDMPODRrate(DMP_ODR_Reg_Cpass, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  // success &= (ICM.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum

  // Enable the FIFO
  success &= (ICM.enableFIFO() == ICM_20948_Stat_Ok);

  // Enable the DMP
  success &= (ICM.enableDMP() == ICM_20948_Stat_Ok);

  // Reset DMP
  success &= (ICM.resetDMP() == ICM_20948_Stat_Ok);

  // Reset FIFO
  success &= (ICM.resetFIFO() == ICM_20948_Stat_Ok);

  // Check success
  if (success)
  {
    Serial.println(F("DMP enabled!"));
  }
  else
  {
    Serial.println(F("Enable DMP failed!"));
    Serial.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
    while (1)
      ; // Do nothing more
  }
}

float lastTime = 0, curTime = 0;

void loop()
{
  // Read any DMP data waiting in the FIFO
  // Note:
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFONoDataAvail if no data is available.
  //    If data is available, readDMPdataFromFIFO will attempt to read _one_ frame of DMP data.
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOIncompleteData if a frame was present but was incomplete
  //    readDMPdataFromFIFO will return ICM_20948_Stat_Ok if a valid frame was read.
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOMoreDataAvail if a valid frame was read _and_ the FIFO contains more (unread) data.
  icm_20948_DMP_data_t data;
  lastTime = curTime;
  ICM.readDMPdataFromFIFO(&data);

  if ((ICM.status == ICM_20948_Stat_Ok) || (ICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
  {
    //Serial.print(F("Received data! Header: 0x")); // Print the header in HEX so we can see what data is arriving in the FIFO
    //if ( data.header < 0x1000) Serial.print( "0" ); // Pad the zeros
    //if ( data.header < 0x100) Serial.print( "0" );
    //if ( data.header < 0x10) Serial.print( "0" );
    //Serial.println( data.header, HEX );

    if ((data.header & DMP_header_bitmap_Quat6) > 0) // We have asked for GRV data so we should receive Quat6
    {
      // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
      // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
      // The quaternion data is scaled by 2^30.

      //SERIAL_PORT.printf("Quat6 data is: Q1:%ld Q2:%ld Q3:%ld\r\n", data.Quat6.Data.Q1, data.Quat6.Data.Q2, data.Quat6.Data.Q3);

      curTime = millis();

      // Scale to +/- 1
      double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
      double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
      double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
      double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

      double qw = q0; 
      double qx = q2;
      double qy = q1;
      double qz = -q3;

      // roll (x-axis rotation)
      double t0 = +2.0 * (qw * qx + qy * qz);
      double t1 = +1.0 - 2.0 * (qx * qx + qy * qy);
      double roll = atan2(t0, t1) * 180.0 / PI;

      // pitch (y-axis rotation)
      double t2 = +2.0 * (qw * qy - qx * qz);
      t2 = t2 > 1.0 ? 1.0 : t2;
      t2 = t2 < -1.0 ? -1.0 : t2;
      double pitch = asin(t2) * 180.0 / PI;

      // yaw (z-axis rotation)
      double t3 = +2.0 * (qw * qz + qx * qy);
      double t4 = +1.0 - 2.0 * (qy * qy + qz * qz);
      double yaw = atan2(t3, t4) * 180.0 / PI;

      float duration = curTime - lastTime;
      Serial.print("Roll: ");
      Serial.print(roll, 3);
      Serial.print(" Pitch: ");
      Serial.print(pitch, 3);
      Serial.print(" Yaw: ");
      Serial.println(yaw, 3);  
      Serial.print(" duration: ");
      Serial.println(duration, 2);

    }

    if ((data.header & DMP_header_bitmap_Compass) > 0){
        int16_t compass_x = data.Compass.Data.X;
        int16_t compass_y = data.Compass.Data.Y;
        int16_t compass_z = data.Compass.Data.Z;
        // Serial.print("compass - X: ");
        // Serial.print(compass_x);
        // Serial.print(" Y: ");
        // Serial.print(compass_y);
        // Serial.print(" Z: ");
        // Serial.println(compass_z);
    }

    if ((data.header & DMP_header_bitmap_Gyro) > 0){
        int16_t gyro_x = data.Raw_Gyro.Data.X;
        int16_t gyro_y = data.Raw_Gyro.Data.Y;
        int16_t gyro_z = data.Raw_Gyro.Data.Z;
        // Serial.print("gyro - X: ");
        // Serial.print(gyro_x);
        // Serial.print(" Y: ");
        // Serial.print(gyro_y);
        // Serial.print(" Z: ");
        // Serial.println(gyro_z);
    }

    if ((data.header & DMP_header_bitmap_Accel) > 0){
        int16_t accel_x = data.Raw_Accel.Data.X;
        int16_t accel_y = data.Raw_Accel.Data.Y;
        int16_t accel_z = data.Raw_Accel.Data.Z;

        curTime = millis();
        float duration = curTime - lastTime;
        // Serial.print("accel - X: ");
        // Serial.print(accel_x);
        // Serial.print(" Y: ");
        // Serial.print(accel_y);
        // Serial.print(" Z: ");
        // Serial.println(accel_z);
        // Serial.print("duration: ");
        // Serial.println(duration);
    }
  }
  else {
    Serial.println("Waiting for data");
    // delay(100);
  }

  if (ICM.status != ICM_20948_Stat_FIFOMoreDataAvail) // If more data is available then we should read it right away - and not delay
  {
    delay(10);
  }
    // delay(100);
}