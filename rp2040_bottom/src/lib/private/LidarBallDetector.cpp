#include "LidarBallDetector.h"
#include <math.h>     // for cos, sin, sqrt
#include <Arduino.h>  // for HardwareSerial, etc.

// Helper for absolute floating compares
static const double EPS = 1e-5;

// ----------------------------
// Constructor
// ----------------------------
LidarBallDetector::LidarBallDetector(HardwareSerial &lidarSerial)
    : lidar(&lidarSerial)
{
    // Precompute lidarAngleRad if desired
    lidarAngleRad = lidarAngleDeg * (M_PI / 180.0);
}

// ----------------------------
// Initialize Lidar
// ----------------------------
bool LidarBallDetector::initializeLidar(int type) {
    GS_error status = lidar.initialize(type);
    if (status != GS_OK) {
        // Possibly retry or return false
        return false;
    }
    lidar.startScanning();
    return true;
}

// ----------------------------
// High-level: findBall
// ----------------------------
bool LidarBallDetector::findBall(double &ballX_robot, double &ballY_robot)
{
    // 1) Collect scan data
    int validCount = 0;
    collectScanData(validCount);
    if (validCount < 3) {
        return false;  // not enough points to do anything
    }

    // 2) DBSCAN
    std::vector<DBPoint> dbPoints(validCount);
    for (int i = 0; i < validCount; i++) {
        dbPoints[i].x        = scanResults[i].x;
        dbPoints[i].y        = scanResults[i].y;
        dbPoints[i].visited  = false;
        dbPoints[i].isNoise  = false;
        dbPoints[i].clusterID= -1;
    }
    int totalClusters = runDBSCAN(dbPoints, dbEps, dbMinPts);

    // 3) Circle-fit each cluster, check if it's the ball
    bool ballFound = false;
    double ballX_lidar = 0.0;
    double ballY_lidar = 0.0;

    // Prepare cluster vectors
    std::vector<std::vector<int>> clusters(totalClusters);
    for(int i=0; i<validCount; i++){
        int cid = dbPoints[i].clusterID;
        if(cid >= 0) {
            clusters[cid].push_back(i);
        }
    }

    for(int cID = 0; cID < totalClusters; cID++) {
        std::vector<std::pair<double,double>> clusterPts;
        for(int idx : clusters[cID]) {
            clusterPts.push_back({ dbPoints[idx].x, dbPoints[idx].y });
        }

        // Fit circle
        CircleFitResult cfit = fitCircleAlgebraic(clusterPts);
        if(!cfit.valid) continue;

        // Print or debug if needed:
        // Serial.print("Cluster "); ...
        
        // Check radius
        double radDiff = fabs(cfit.r - ballRadius);
        if(radDiff <= radiusTolerance && cfit.residual < maxResidual) {
            // This cluster is likely the ball
            ballFound     = true;
            ballX_lidar   = cfit.cx;
            ballY_lidar   = cfit.cy;
            // if only first match is needed, break here
        }
    }

    if (!ballFound) {
        return false;
    }

    // 4) Transform from LiDAR->Robot for final output
    ballX_robot = lidarOffsetX + (ballX_lidar*cos(lidarAngleRad) - ballY_lidar*sin(lidarAngleRad));
    ballY_robot = lidarOffsetY + (ballX_lidar*sin(lidarAngleRad) + ballY_lidar*cos(lidarAngleRad));

    return true;
}

// --------------------------------------------------
// Private Methods
// --------------------------------------------------

// 1) collectScanData
void LidarBallDetector::collectScanData(int &validCount) {
    validCount = 0;
    iter_Scan scan = lidar.iter_scans();

    for (int i = 0; i < maxScanCount; i++) {
        if (!scan.valid[i]) {
            continue;
        }
        double correctedAngle = scan.angle[i];
        double distance       = scan.distance[i];

        if (correctedAngle > 180) {
            correctedAngle -= 360;
        }

        // Convert to LiDAR coords
        double rad = (correctedAngle - 270.0) * M_PI / 180.0;
        double xVal = distance * -cos(rad);
        double yVal = distance * sin(rad);

        // optional check for duplicates
        bool isRepeated = false;
        for (int j = 0; j < validCount; j++) {
            if (fabs(scanResults[j].x - xVal) < EPS &&
                fabs(scanResults[j].y - yVal) < EPS ) {
                isRepeated = true;
                break;
            }
        }

        if (!isRepeated) {
            scanResults[validCount].angle    = correctedAngle;
            scanResults[validCount].distance = distance;
            scanResults[validCount].x        = xVal;
            scanResults[validCount].y        = yVal;
            validCount++;
        }
    }
}

// 2) runDBSCAN
int LidarBallDetector::runDBSCAN(std::vector<DBPoint> &dbPoints, double eps, int minPts)
{
    // regionQuery, expandCluster used inside
    int clusterID = 0;
    for (int i = 0; i < (int)dbPoints.size(); i++) {
        if (!dbPoints[i].visited) {
            dbPoints[i].visited = true;
            auto neighbors = regionQuery(dbPoints, i, eps);
            if ((int)neighbors.size() < minPts) {
                dbPoints[i].isNoise = true;
            } else {
                expandCluster(dbPoints, i, neighbors, clusterID, eps, minPts);
                clusterID++;
            }
        }
    }
    return clusterID;
}

// 3) expandCluster
void LidarBallDetector::expandCluster(std::vector<DBPoint> &points, int idx, 
                                      const std::vector<int> &neighbors,
                                      int clusterID, double eps, int minPts)
{
    points[idx].clusterID = clusterID;
    std::vector<int> seeds(neighbors);

    for (size_t i = 0; i < seeds.size(); i++) {
        int currIdx = seeds[i];
        if (!points[currIdx].visited) {
            points[currIdx].visited = true;
            auto currNeighbors = regionQuery(points, currIdx, eps);
            if ((int)currNeighbors.size() >= minPts) {
                for (int nb : currNeighbors) {
                    if (points[nb].clusterID < 0) {
                        seeds.push_back(nb);
                    }
                }
            }
        }
        if (points[currIdx].clusterID < 0) {
            points[currIdx].clusterID = clusterID;
        }
    }
}

// 4) regionQuery
std::vector<int> LidarBallDetector::regionQuery(const std::vector<DBPoint> &points, int idx, double eps)
{
    std::vector<int> neighbors;
    double x1 = points[idx].x;
    double y1 = points[idx].y;
    for (int i = 0; i < (int)points.size(); i++) {
        double dist = pointDistance(x1, y1, points[i].x, points[i].y);
        if (dist <= eps) {
            neighbors.push_back(i);
        }
    }
    return neighbors;
}

// 5) pointDistance
double LidarBallDetector::pointDistance(double x1, double y1, double x2, double y2)
{
    double dx = x1 - x2;
    double dy = y1 - y2;
    return sqrt(dx*dx + dy*dy);
}

// --------------------------------------------------
// Circle Fit
// --------------------------------------------------
CircleFitResult LidarBallDetector::fitCircleAlgebraic(const std::vector<std::pair<double,double>> &pts)
{
    CircleFitResult res{0,0,0,false,1e9};
    int n = (int)pts.size();
    if(n < 3) return res;

    double sumX=0, sumY=0, sumX2=0, sumY2=0, sumXY=0;
    double sumR=0, sumXR=0, sumYR=0;

    // R_i = x_i^2 + y_i^2
    for(const auto &p : pts) {
        double x = p.first;
        double y = p.second;
        double r2= x*x + y*y;

        sumX  += x; 
        sumY  += y; 
        sumX2 += x*x;
        sumY2 += y*y;
        sumXY += x*y;
        sumR  += r2;
        sumXR += x*r2;
        sumYR += y*r2;
    }

    // Solve the system of 3x3
    double M[3][3] = {
        { sumX2, sumXY, sumX },
        { sumXY, sumY2, sumY },
        { sumX,  sumY,  (double)n }
    };
    double RHS[3] = { -sumXR, -sumYR, -sumR };

    // Basic solver
    auto solve3x3 = [&](double m[3][3], double r[3]){
        double x[3]{0,0,0};
        // forward elimination
        for(int i=0; i<3; i++){
            // pivot
            double maxEl = fabs(m[i][i]);
            int pivot = i;
            for(int k=i+1; k<3; k++){
                double val = fabs(m[k][i]);
                if(val>maxEl){
                    maxEl=val;
                    pivot=k;
                }
            }
            // swap row pivot
            if(pivot!=i){
                for(int col=0; col<3; col++){
                    std::swap(m[i][col], m[pivot][col]);
                }
                std::swap(r[i], r[pivot]);
            }
            // eliminate below
            for(int k=i+1; k<3; k++){
                double c = -m[k][i]/m[i][i];
                for(int col=i; col<3; col++){
                    if(i==col) {
                        m[k][col]=0;
                    } else {
                        m[k][col] += c*m[i][col];
                    }
                }
                r[k] += c*r[i];
            }
        }
        // back-substitution
        for(int i2=2; i2>=0; i2--){
            double sumVal = r[i2];
            for(int j=i2+1; j<3; j++){
                sumVal -= m[i2][j]*x[j];
            }
            x[i2] = sumVal/m[i2][i2];
        }
        return std::vector<double>{x[0], x[1], x[2]};
    };

    auto abc = solve3x3(M, RHS);
    double A = abc[0];
    double B = abc[1];
    double C = abc[2];

    double cx = -A*0.5;
    double cy = -B*0.5;
    double r2 = cx*cx + cy*cy - C;
    if(r2 <= 0) return res;

    double r = sqrt(r2);

    // Average squared residual
    double sumError = 0.0;
    for(const auto &p : pts){
        double dx = p.first - cx;
        double dy = p.second- cy;
        double dist = sqrt(dx*dx + dy*dy);
        double diff = dist - r;
        sumError += diff*diff;
    }
    double avgError = sumError / n;

    res.cx       = cx;
    res.cy       = cy;
    res.r        = r;
    res.valid    = true;
    res.residual = avgError;
    return res;
}
