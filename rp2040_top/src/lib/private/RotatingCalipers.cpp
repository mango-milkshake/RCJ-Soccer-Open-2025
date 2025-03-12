#include "RotatingCalipers.h"
#include <CommonUtils.h>
#define NUM_POINTS 24
#define FIELD_WIDTH 1.82 // 0.91
#define FIELD_HEIGHT 2.43 // 1.21

Point nextToTop(stack<Point> &S)
{
    Point p = S.top();
    S.pop();
    Point res = S.top();
    S.push(p);
    return res;
}

float distSq(Point p1, Point p2)
{
    return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y);
}

int orientation(Point p, Point q, Point r)
{
    float val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
    if (fabs(val) < 1e-9) 
        return 0;         
    return (val > 0) ? 1 : 2;
}

int convexHull(Point points[], size_t n, Point hull[]) {
    if (n < 3) return 0;

    // Sort points lexicographically
    sort(points, points + n);

    // Temporary storage for hull
    Point tempHull[2 * NUM_POINTS];
    size_t k = 0;

    // Lower hull
    for (size_t i = 0; i < n; i++) {
        while (k >= 2 && orientation(tempHull[k - 2], tempHull[k - 1], points[i]) != 2)
            k--;
        tempHull[k++] = points[i];
    }

    // Upper hull
    for (size_t i = n - 1, t = k + 1; i > 0; i--) {
        while (k >= t && orientation(tempHull[k - 2], tempHull[k - 1], points[i - 1]) != 2)
            k--;
        tempHull[k++] = points[i - 1];
    }

    // Remove redundant points
    size_t hullSize = k - 1;
    for (size_t i = 0; i < hullSize; i++) {
        hull[i] = tempHull[i];
    }

    return hullSize; // Return actual hull size
}

MinAreaRect findMinAreaRect(Point hull[], size_t n) {
    if (n < 3) return {};

    float min_area = numeric_limits<float>::max();
    MinAreaRect result;

    for (size_t i = 0; i < n; i++) {
        Point edge = {hull[(i + 1) % n].x - hull[i].x, hull[(i + 1) % n].y - hull[i].y};
        float edge_length = sqrt(edge.x * edge.x + edge.y * edge.y);

        edge.x /= edge_length;
        edge.y /= edge_length;

        Point orthogonal = {-edge.y, edge.x};

        float min_proj_edge = numeric_limits<float>::max();
        float max_proj_edge = numeric_limits<float>::lowest();
        float min_proj_orthogonal = numeric_limits<float>::max();
        float max_proj_orthogonal = numeric_limits<float>::lowest();

        for (size_t j = 0; j < n; j++) {
            float proj_edge = hull[j].x * edge.x + hull[j].y * edge.y;
            float proj_orthogonal = hull[j].x * orthogonal.x + hull[j].y * orthogonal.y;

            min_proj_edge = min(min_proj_edge, proj_edge);
            max_proj_edge = max(max_proj_edge, proj_edge);
            min_proj_orthogonal = min(min_proj_orthogonal, proj_orthogonal);
            max_proj_orthogonal = max(max_proj_orthogonal, proj_orthogonal);
        }

        float width = max_proj_edge - min_proj_edge;
        float height = max_proj_orthogonal - min_proj_orthogonal;
        float area = width * height;

        if (area < min_area) {
            min_area = area;

            float corner_x = min_proj_edge * edge.x + min_proj_orthogonal * orthogonal.x;
            float corner_y = min_proj_edge * edge.y + min_proj_orthogonal * orthogonal.y;

            result.corner = {corner_x, corner_y};
            result.vector_width = edge;
            result.vector_height = orthogonal;
            result.width = width;
            result.height = height;
            result.area = area;

            // if(result.width > result.height){
            //     swap(result.width, result.height);
            //     swap(result.vector_width, result.vector_height);
            // }
        }
    }

    if(result.width > result.height){
        swap(result.width, result.height);
        swap(result.vector_width, result.vector_height);
        result.swap = true;
    }
    else result.swap = false;

    // Compute rectangle vertices
    result.bottom_left = result.corner;
    result.bottom_right = {result.bottom_left.x + result.width * result.vector_width.x,
        result.bottom_left.y + result.width * result.vector_width.y};
    result.top_right = {result.bottom_right.x + result.height * result.vector_height.x,
        result.bottom_right.y + result.height * result.vector_height.y};
    result.top_left = {result.bottom_left.x + result.height * result.vector_height.x,
        result.bottom_left.y + result.height * result.vector_height.y};

    if((result.bottom_right.x * result.top_left.y) - (result.bottom_right.y * result.top_left.x)<0){
        swap(result.bottom_left, result.bottom_right); // might mess up stuff
        result.vector_width.x = -result.vector_width.x;
        result.vector_width.y = -result.vector_width.y;
        result.flip = true;
    }
    else result.flip = false;

    return result;
}

Point getDist(float len, float angle){
    Point res;
    res.x = cosf(RAD(angle)) * len;
    res.y = sinf(RAD(angle)) * len;
    return res;
}

Point getCoords(MinAreaRect r, float angle){
    Point coord;

    float len = sqrt(r.bottom_left.x*r.bottom_left.x + r.bottom_left.y*r.bottom_left.y);
    angle = DEG(atanf(r.bottom_left.y/r.bottom_left.x)) - angle;

    Point p = getDist(len, angle);
    float width_left = abs(p.x), height_bottom = abs(p.y);
    float width_ratio = FIELD_WIDTH / (r.width);
    float height_ratio = FIELD_HEIGHT / (r.height);
    coord.x = width_left * width_ratio;
    coord.y = height_bottom * height_ratio;
    return coord;
}

Point rotatePoint(Point p, float angle){
    // rotate coordinates of point p by angle, clockwise
    // angle in radians
    Point res;
    res.x = -(p.x * cosf(angle) + p.y * sinf(angle));
    res.y = -(p.y * cosf(angle) - p.x * sinf(angle));
    return res;
}

Point scaleCoord(MinAreaRect r, Point p){
    Point coord;
    float width_ratio = FIELD_WIDTH / (r.width);
    float height_ratio = FIELD_HEIGHT / (r.height);
    coord.x = p.x * width_ratio;
    coord.y = p.y * height_ratio;
    return coord;
}