#include <Arduino.h>
#include "LidarBallDetector.h"

// Create a LidarBallDetector globally
// We'll pass Serial2 to it in setup
LidarBallDetector myBallDetector(Serial2);

// Standard Arduino setup
void setup() {
    Serial.begin(115200);

    // Adjust calibrations if needed
    myBallDetector.dbEps        = 10.0;
    myBallDetector.dbMinPts     = 10;
    myBallDetector.ballRadius   = 22.0;
    myBallDetector.radiusTolerance = 7.0;
    myBallDetector.maxResidual = 2.0;

    myBallDetector.lidarOffsetX  = 35.0;
    myBallDetector.lidarOffsetY  = 60.0;
    myBallDetector.lidarAngleDeg = 22.5;
    myBallDetector.lidarAngleRad = myBallDetector.lidarAngleDeg * (M_PI/180.0);

    // Lidar init
    Serial2.setTX(4);
    Serial2.setRX(5);
    bool ok = myBallDetector.initializeLidar(1);
    if(!ok){
        Serial.println("Failed to initialize Lidar!");
    } else {
        Serial.println("Lidar init OK, scanning started");
    }
}

void loop() {
    double bx_robot=0, by_robot=0;
    bool found = myBallDetector.findBall(bx_robot, by_robot);
    if(found) {
        Serial.print("Ball found in robot frame at (");
        Serial.print(bx_robot, 2);
        Serial.print(", ");
        Serial.print(by_robot, 2);
        Serial.println(").");
    } else {
        Serial.println("No ball found this scan.");
    }


}