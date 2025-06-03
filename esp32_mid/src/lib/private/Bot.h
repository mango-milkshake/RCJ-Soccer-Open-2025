#ifndef BOT_H
#define BOT_H

#include <Arduino.h>
#include <Data.h>
#include <CommonUtils.h>

// Dimensions
#define FIELD_WIDTH 1.82
#define FIELD_HEIGHT 2.43
#define FIELD_MARGIN 0.12
#define SELF_GOAL_LEFT_X 0.61
#define SELF_GOAL_RIGHT_X 1.21
#define SELF_GOAL_Y 0.12
#define OPP_GOAL_CENTRE_X 0.91
#define OPP_GOAL_CENTRE_Y 2.384
#define OPP_GOAL_MIDDLE_X 0.91
#define OPP_GOAL_MIDDLE_Y 2.06
#define OPP_GOAL_LEFT_X 0.61
#define OPP_GOAL_RIGHT_X 1.21
#define OPP_GOAL_Y 2.31
#define BOT_RADIUS_CM 8.5 // in cm
#define BOT_RADIUS_M 0.085 // in metres
#define Y_BOUND 1.50 

// Thresholds
#define OSCILLATE_WAIT_TIME 2000
#define MOVING_BACK_DURATION 200
#define BALLCAP_DISTANCE 0.014f
#define BALLCAP_WIDTH 0.0355f
#define CLEARANCE_X 0.20f
#define CLEARANCE_Y 0.15f
#define FIELD_MARGIN_X 0.51f
#define FIELD_MARGIN_Y 0.37f
#define ALIGN_DURATION 2000
#define ALIGN_THRESHOLD 3000
#define INITIAL_CHANGE 35.0f
#define GRADUAL_CHANGE 250.0f
#define BALLCAP_DURATION 250

class Bot{
    public:
        void moveToPoint(float pointx, float pointy, float rotation){
            move.x = pointx;
            move.y = pointy;
            move.rotation = rotation;
        }

        struct OscillateAboutPoint{
            bool oscState = true, reachTargetOsc = false;
            int reachOscTime = 0;
        } osc;

        void oscillateAboutPoint(float pointx, float pointy, float oscDist){
            float new_x = 0, new_y = pointy;
            if(osc.oscState) new_x = pointx - oscDist;
            else new_x = pointx + oscDist;
            float distToPoint = sqrt((new_x - self.x)*(new_x - self.x) + (new_y - self.y)*(new_y - self.y));
            if(distToPoint <= 0.10){
                if(!osc.reachTargetOsc) {
                    osc.reachTargetOsc = true;
                    osc.reachOscTime = millis();
                }
            }
            if(osc.reachTargetOsc && millis() - osc.reachOscTime >= OSCILLATE_WAIT_TIME){
                osc.oscState = !osc.oscState;
                osc.reachTargetOsc = false;
                osc.reachOscTime = 0;
            }
            move.x = new_x;
            move.y = new_y;
            move.rotation = 0.0;
        }

        struct BallTrack{
            bool aligned = false, moving_back;
            float initial_change = 0, initial_magnitude = 0;
            int last_moving_back = 0, aligning_time = 0, last_aligning = 0;
        } balltrack;

        void ballTrack(){
            // no dribbler
            balltrack.aligned = false;
            balltrack.initial_change = 0;
            balltrack.initial_magnitude = 0;

            float new_x, new_y;
            if(self.y > ball.absolute_y) balltrack.moving_back = true;
            if((balltrack.moving_back || millis() - balltrack.last_moving_back > MOVING_BACK_DURATION) &&
                (self.y > ball.absolute_y - BALLCAP_DISTANCE/3 || 
                (abs(self.x - ball.absolute_x) > BALLCAP_WIDTH/2 + 0.05 &&
                abs(self.x - ball.absolute_x) < CLEARANCE_X/2 &&
                self.y > ball.absolute_y - CLEARANCE_Y/2))){
                    if(ball.absolute_x < FIELD_MARGIN + CLEARANCE_X + 0.10) new_x = ball.absolute_x + (CLEARANCE_X/2 + 0.05);
                    else if(ball.absolute_x > FIELD_WIDTH - FIELD_MARGIN - CLEARANCE_X - 0.10) new_x = ball.absolute_x - (CLEARANCE_X/2 + 0.05);
                    else if(self.x > ball.absolute_x) new_x = ball.absolute_x + (CLEARANCE_X/2 + 0.05);
                    else new_x = ball.absolute_x - (CLEARANCE_X/2 + 0.05);
                    new_y = (abs(self.x - ball.absolute_x) > CLEARANCE_X/2 + 0.03) ? ball.absolute_y - CLEARANCE_Y/2 - 0.10 : self.y;
                    balltrack.moving_back = true;
            }
            else{
                if(balltrack.moving_back) {
                    balltrack.moving_back = false;
                    balltrack.last_moving_back = millis();
                }
                balltrack.aligning_time = millis() - balltrack.last_aligning;
                new_x = ball.absolute_x;
                if((balltrack.aligning_time > ALIGN_DURATION && balltrack.aligning_time < ALIGN_THRESHOLD) || abs(self.x - ball.absolute_x) < BALLCAP_WIDTH/2)
                    new_y = fmax(ball.absolute_y - BALLCAP_DISTANCE, self.y + 0.03);
                else{
                    if(balltrack.aligning_time > ALIGN_THRESHOLD) balltrack.last_aligning = millis();
                    new_y = ball.absolute_y - BALLCAP_DISTANCE;
                }
            }
            move.x = new_x;
            move.y = new_y;
            move.rotation = 0.0;
        }

        void dribblerBallTrack(){
            float xToBall = ball.absolute_x - self.x, yToBall = ball.absolute_y - self.y;
            float distToBall = sqrt(xToBall * xToBall + yToBall * yToBall);
            float new_x = self.x + xToBall * (distToBall - BALLCAP_DISTANCE) / distToBall;
            float new_y = self.y + yToBall * (distToBall - BALLCAP_DISTANCE) / distToBall;
        
            float absBallAngle = atan2(yToBall, xToBall);
            LIM_ANGLE_180(absBallAngle);
            
            move.x = new_x;
            move.y = new_y;
            move.rotation = 90-DEG(absBallAngle);
        }

    private:
} bot;

#endif