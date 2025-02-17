#ifndef COMMONUTILS_H
#define COMMONUTILS_H

// Angle Conversions
// Convert radians to degrees
#define RAD(x) ((x) / 180.0f * (float) PI)
// Convert degrees to radians
#define DEG(x) ((x) * 180.0f / (float) PI)

// Limit angle to 0 to 360 degrees
#define LIM_ANGLE(angle) (angle > 0 ? fmod(angle, 360) : fmod(angle, 360) + 360)

#define DELTA_ANGLE(a, b) (abs(fmod((a - b + 180), 360)) - 180)
#define ANGLE_360_TO_180(angle) (angle > 180 ? angle - 360 : angle)

#endif