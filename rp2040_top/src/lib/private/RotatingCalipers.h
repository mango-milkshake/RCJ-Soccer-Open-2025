#ifndef ROTATING_CALIPERS_H
#define ROTATING_CALIPERS_H

#include <Arduino.h>
#include <stack>
#include <algorithm>
using namespace std;

struct Point
{
    float x, y;

    bool operator<(const Point &p) const
    {
        return x < p.x || (x == p.x && y < p.y);
    }
};


Point nextToTop(stack<Point> &S);

float distSq(Point p1, Point p2);

int orientation(Point p, Point q, Point r);

int convexHull(Point points[], size_t n, Point hull[]);

// Rotating Calipers: Minimum Area Bounding Rectangle
struct MinAreaRect
{
    Point corner, vector_width, vector_height; 
    float width, height, area;
    Point bottom_left, bottom_right, top_left, top_right;        
};

MinAreaRect findMinAreaRect(Point hull[], size_t n);

Point getDist(Point p, float angle);

Point getCoords(MinAreaRect r, float angle);

#endif