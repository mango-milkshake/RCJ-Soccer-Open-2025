#ifndef LIDAR_BALL_DETECTOR_H
#define LIDAR_BALL_DETECTOR_H

#include <YDLiDar_gs2.h>  // for YDLiDar_GS2
#include <vector>         // for std::vector

// -----------------------------------------------------------
// Data structures
// -----------------------------------------------------------
struct ScanData {
    double angle;    
    double distance; 
    double x;        
    double y;        
};

struct DBPoint {
    double x;
    double y;
    bool visited;
    bool isNoise;
    int clusterID; // -1 if unassigned/noise
};

struct CircleFitResult {
    double cx;      
    double cy;      
    double r;       
    bool valid;     
    double residual;
};

// -----------------------------------------------------------
// The LidarBallDetector class
// -----------------------------------------------------------
class LidarBallDetector {
public:
    // Constructor: pass a reference to the lidar Serial, plus any config
    LidarBallDetector(HardwareSerial &lidarSerial);

    // Call once in setup() to initialize the lidar
    bool initializeLidar(int type = 1);

    // Core function: reads data from Lidar, does DBSCAN, circle fit, returns (x,y)
    // in the robot frame for the detected ball (if found).
    // Return 'true' if a ball is found, 'false' otherwise.
    bool findBall(double &ballX_robot, double &ballY_robot);

    // Optionally, you can set these public if you want direct manipulation
    // DBSCAN parameters
    double dbEps       = 10.0;  
    int    dbMinPts    = 10;

    // Ball-fitting parameters
    double ballRadius          = 22.0; // mm
    double radiusTolerance     = 7.0;  // +/- tolerance
    double maxResidual         = 2.0;  // circle residual

    // LiDAR -> Robot frame calibration
    double lidarOffsetX   = 35.0;   
    double lidarOffsetY   = 60.0;   
    double lidarAngleDeg  = 22.5;   
    double lidarAngleRad  = 0.0;   

private:
    // The YDLiDar driver
    YDLiDar_GS2 lidar;

    // Buffer for raw scan data
    static const int maxScanCount = 200;
    ScanData scanResults[maxScanCount];

    // Helper functions
    void collectScanData(int &validCount);
    int  runDBSCAN(std::vector<DBPoint> &dbPoints, double eps, int minPts);
    CircleFitResult fitCircleAlgebraic(const std::vector<std::pair<double,double>> &pts);

    // Extra small helpers
    double pointDistance(double x1, double y1, double x2, double y2);
    std::vector<int> regionQuery(const std::vector<DBPoint> &points, int idx, double eps);
    void expandCluster(std::vector<DBPoint> &points, int idx, const std::vector<int> &neighbors,
                       int clusterID, double eps, int minPts);
};

#endif // LIDAR_BALL_DETECTOR_H
