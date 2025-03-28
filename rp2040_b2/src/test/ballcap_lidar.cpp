#include <YDLiDar_gs2.h>
#include <math.h>  // or <cmath>
#include <vector>

YDLiDar_GS2 lidar(&Serial1);

const int maxScanCount = 200;
#define EPS 1e-5


static const double DB_EPS    = 15.0;
static const int    DB_MINPTS = 5;     


// Ball radius in mm, plus tolerances for acceptance.
static const double BALL_RADIUS      = 22;   // mm
static const double RADIUS_TOLERANCE = 5.0;     

static const double MAX_RESIDUAL     = 10.0;    


struct ScanData {
  double angle;
  double distance;  
  double x;        
  double y;        
};
ScanData scanResults[maxScanCount];


struct DBPoint {
  double x;
  double y;
  bool visited;
  bool isNoise;
  int clusterID; // -1 if unassigned/noise
};


inline double pointDistance(const DBPoint &a, const DBPoint &b) {
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return sqrt(dx * dx + dy * dy);
}

std::vector<int> regionQuery(const std::vector<DBPoint> &points, int idx, double eps) {
  std::vector<int> neighbors;
  for (int i = 0; i < (int)points.size(); i++) {
    if (pointDistance(points[idx], points[i]) <= eps) {
      neighbors.push_back(i);
    }
  }
  return neighbors;
}

void expandCluster(std::vector<DBPoint> &points,
                   int idx, 
                   const std::vector<int> &neighbors,
                   int clusterID,
                   double eps, 
                   int minPts) 
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

int dbscan(std::vector<DBPoint> &points, double eps, int minPts) {
  int clusterID = 0;
  for (int i = 0; i < (int)points.size(); i++) {
    if (!points[i].visited) {
      points[i].visited = true;
      std::vector<int> neighbors = regionQuery(points, i, eps);
      if ((int)neighbors.size() < minPts) {
        points[i].isNoise = true;
      } else {
        expandCluster(points, i, neighbors, clusterID, eps, minPts);
        clusterID++;
      }
    }
  }
  return clusterID; // total number of clusters found
}


struct CircleFitResult {
  double cx;      
  double cy;       
  double r;        
  bool valid;      
  double residual; 
};


CircleFitResult fitCircleAlgebraic(const std::vector<std::pair<double,double>> &pts)
{
  CircleFitResult res{0, 0, 0, false, 1e9};

  int n = (int)pts.size();
  if(n < 3) {
    return res; // can't fit a circle with fewer than 3 points
  }

  double sumX=0, sumY=0, sumX2=0, sumY2=0, sumXY=0;
  double sumR=0, sumXR=0, sumYR=0;

  // R_i = x_i^2 + y_i^2
  for(const auto &p : pts) {
    double x = p.first;
    double y = p.second;
    double r2 = x*x + y*y;

    sumX  += x;
    sumY  += y;
    sumX2 += x*x;
    sumY2 += y*y;
    sumXY += x*y;
    sumR  += r2;
    sumXR += x*r2;
    sumYR += y*r2;
  }

  // Solve the system:
  //   [ sumX2  sumXY  sumX ]   [ A ]   [ -sumXR ]
  //   [ sumXY  sumY2  sumY ] * [ B ] = [ -sumYR ]
  //   [ sumX   sumY   n    ]   [ C ]   [ -sumR  ]

  double M[3][3] = {
    { sumX2, sumXY, sumX },
    { sumXY, sumY2, sumY },
    { sumX,  sumY,  (double)n }
  };
  double RHS[3] = { -sumXR, -sumYR, -sumR };

  // Basic 3x3 solver via Gaussian elimination
  auto solve3x3 = [&](double m[3][3], double r[3]) {
    double x[3]{0,0,0};
    // Forward elimination
    for(int i=0;i<3;i++){
      // pivot
      double maxEl = fabs(m[i][i]);
      int pivot = i;
      for(int k=i+1;k<3;k++){
        double val = fabs(m[k][i]);
        if(val>maxEl){
          maxEl=val;
          pivot=k;
        }
      }
      // swap pivot row
      if(pivot!=i){
        for(int col=0;col<3;col++){
          std::swap(m[i][col], m[pivot][col]);
        }
        std::swap(r[i], r[pivot]);
      }
      // eliminate below pivot
      for(int k=i+1;k<3;k++){
        double c = -m[k][i]/m[i][i];
        for(int col=i; col<3; col++){
          if(i==col){
            m[k][col]=0;
          } else {
            m[k][col] += c*m[i][col];
          }
        }
        r[k]+= c*r[i];
      }
    }
    // back-substitution
    for(int i2=2;i2>=0;i2--){
      double sumVal = r[i2];
      for(int j=i2+1;j<3;j++){
        sumVal -= m[i2][j]*x[j];
      }
      x[i2] = sumVal/m[i2][i2];
    }
    return std::vector<double>{x[0],x[1],x[2]};
  };

  std::vector<double> abc = solve3x3(M, RHS);
  double A = abc[0];
  double B = abc[1];
  double C = abc[2];

  // Convert to (cx, cy, r)
  double cx = -A*0.5;
  double cy = -B*0.5;
  double r2 = cx*cx + cy*cy - C; 
  if(r2 <= 0) {
    return res; // invalid circle
  }
  double r = sqrt(r2);

  // Compute an average squared residual
  double sumError = 0.0;
  for(const auto &p : pts) {
    double dx = p.first  - cx;
    double dy = p.second - cy;
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


void setup() {
  Serial.begin(115200);

  GS_error status = lidar.initialize(1);
  while (status != GS_OK) {
    Serial.println("There was an error initializing the lidar, trying again...");
    status = lidar.initialize(1);
  }

  // Start scanning
  lidar.startScanning();
}


void loop() {
  // 1) Collect LiDAR data into scanResults
  iter_Scan scan = lidar.iter_scans();
  int validCount = 0;

  for (int i = 0; i < maxScanCount; i++) {
    if (!scan.valid[i]) {
      continue;
    }
    double correctedAngle = scan.angle[i];
    double distance       = scan.distance[i];


    if (correctedAngle > 180) {
      correctedAngle -= 360;
    }

    double rad = (correctedAngle - 270.0) * M_PI / 180.0;


    double xVal = distance * cos(rad);
    double yVal = distance * sin(rad);


    bool isRepeated = false;
    for (int j = 0; j < validCount; j++) {
      if (fabs(scanResults[j].x - xVal) < EPS) {
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


  std::vector<DBPoint> dbPoints(validCount);
  for (int i = 0; i < validCount; i++) {
    dbPoints[i].x = scanResults[i].x;
    dbPoints[i].y = scanResults[i].y;
    dbPoints[i].visited   = false;
    dbPoints[i].isNoise   = false;
    dbPoints[i].clusterID = -1;
  }


  int totalClusters = dbscan(dbPoints, DB_EPS, DB_MINPTS);


  std::vector<std::vector<int>> clusters(totalClusters);
  for(int i=0; i<validCount; i++) {
    int cid = dbPoints[i].clusterID;
    if(cid >= 0) {
      clusters[cid].push_back(i);
    }
  }


  bool ballFound = false;
  double ballX = 0, ballY = 0;
  
  for(int cID=0; cID<totalClusters; cID++) {

    std::vector<std::pair<double,double>> clusterPts;
    for(int idx : clusters[cID]) {
      clusterPts.push_back({dbPoints[idx].x, dbPoints[idx].y});
    }

    // Fit the circle algebraically
    CircleFitResult cfit = fitCircleAlgebraic(clusterPts);
    if(!cfit.valid) {
      // Possibly a degenerate circle. Print debug info if needed
      // Serial.print("Cluster ");
      // Serial.print(cID);
      // Serial.println(": invalid circle fit (r^2 < 0).");
      continue;
    }

    //Print cluster debug info
    Serial.print("Cluster ");
    Serial.print(cID);
    Serial.print(": center=(");
    Serial.print(cfit.cx, 2);
    Serial.print(",");
    Serial.print(cfit.cy, 2);
    Serial.print("), radius=");
    Serial.print(cfit.r, 2);
    Serial.print(", residual=");
    Serial.print(cfit.residual, 4);
    Serial.println();

    // Check if radius is near 21.35 mm
    double radDiff = fabs(cfit.r - BALL_RADIUS);
    if(radDiff <= RADIUS_TOLERANCE && cfit.residual < MAX_RESIDUAL) {
      // This cluster likely represents the ball
      ballFound = true;
      ballX = cfit.cx;
      ballY = cfit.cy;
      // Serial.print("  --> This cluster might be the ball!\n");
    }
  }

  // 6) Print final JSON or debugging (points + clusterID)
  // If you only want circle info, you can comment this out
  // Serial.print("{");
  // for (int i = 0; i < validCount; i++) {
  //   Serial.print("{");
  //   Serial.print(dbPoints[i].x, 3);
  //   Serial.print(", ");
  //   Serial.print(dbPoints[i].y, 3);
  //   Serial.print(", ");
  //   Serial.print(dbPoints[i].clusterID);
  //   Serial.print("}");
  //   if (i < validCount - 1) {
  //     Serial.print(", ");
  //   }
  // }
  // Serial.println("}");

  // 7) If ball found, print position
  if(ballFound) {
    Serial.print("Ball found at (");
    Serial.print(ballX, 2);
    Serial.print(", ");
    Serial.print(ballY, 2);
    Serial.println(")\n");
  } else {
    Serial.println("No valid ball detected in this scan.\n");
  }

  delay(1000);
}
