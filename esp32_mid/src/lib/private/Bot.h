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
#define BALLCAP_DISTANCE 0.065f
#define BALLCAP_DISTANCE 0.065f
#define BALLCAP_WIDTH 0.0355f
#define CLEARANCE_X 0.20f
#define CLEARANCE_Y 0.35f
#define CLEARANCE_Y 0.35f
#define FIELD_MARGIN_X 0.51f
#define FIELD_MARGIN_Y 0.37f
#define ALIGN_DURATION 2000
#define ALIGN_TIME_THRESHOLD 3000
#define ALIGN_THRESHOLD 0.10f
#define ALIGN_TIME_THRESHOLD 3000
#define ALIGN_THRESHOLD 0.10f
#define INITIAL_CHANGE 35.0f
#define GRADUAL_CHANGE 250.0f
#define BALLCAP_DURATION 2504
#define LAST_SEEN_BALL_TIME 200


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
            float dribblerDist = 0.60; // distance away from ball to turn on dribbler
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
                    Serial.println("moving back");
            }
            else{
                if(balltrack.moving_back) {
                    balltrack.moving_back = false;
                    balltrack.last_moving_back = millis();
                }
                balltrack.aligning_time = millis() - balltrack.last_aligning;
                new_x = ball.absolute_x;
                if((balltrack.aligning_time > ALIGN_DURATION && balltrack.aligning_time < ALIGN_TIME_THRESHOLD) || abs(self.x - ball.absolute_x) < BALLCAP_WIDTH/2){
                if((balltrack.aligning_time > ALIGN_DURATION && balltrack.aligning_time < ALIGN_TIME_THRESHOLD) || abs(self.x - ball.absolute_x) < BALLCAP_WIDTH/2){
                    new_y = fmax(ball.absolute_y - BALLCAP_DISTANCE, self.y + 0.03);
                    Serial.println("aligning 1");
                }
                    Serial.println("aligning 1");
                }
                else{
                    if(balltrack.aligning_time > ALIGN_TIME_THRESHOLD) balltrack.last_aligning = millis();
                    if(balltrack.aligning_time > ALIGN_TIME_THRESHOLD) balltrack.last_aligning = millis();
                    new_y = ball.absolute_y - BALLCAP_DISTANCE;
                    Serial.println("aligning 2");
                    Serial.println("aligning 2");
                }
                new_y = ball.absolute_y; // testing, remove me later idk
                new_y = ball.absolute_y; // testing, remove me later idk
            }

            float xToBall = ball.absolute_x - self.x, yToBall = ball.absolute_y - self.y;
            float absBallAngle = atan2(yToBall, xToBall);
            LIM_ANGLE_180(absBallAngle);
 
            move.x = new_x;
            move.y = new_y;
            move.rotation = 90-DEG(absBallAngle);
        }

        void aim(){
            if(!balltrack.aligned){
                if(abs(self.x - ball.absolute_x) < ALIGN_THRESHOLD) balltrack.aligned = true;
                move.x = ball.absolute_x;
                move.y = self.y;
                move.rotation = 0;
            }
            else{
                float xToGoal = OPP_GOAL_CENTRE_X - self.x, yToGoal = OPP_GOAL_CENTRE_Y - self.y;
                float distToGoal = sqrt(xToGoal * xToGoal + yToGoal * yToGoal);
                float angleToGoal = PI/2 - atan2(yToGoal, xToGoal);
                if(balltrack.initial_change == 0){
                    balltrack.initial_magnitude = distToGoal;
                    balltrack.initial_change = max(0.0f, cosf(angleToGoal)) * INITIAL_CHANGE;
                }
                float change = balltrack.initial_change + max(0.0f, balltrack.initial_magnitude - distToGoal) 
                    / balltrack.initial_magnitude * GRADUAL_CHANGE;
                change = min(change, max(0.0f, (self.y + FIELD_MARGIN_Y)*100/cosf(angleToGoal)));
                move.x = self.x + change * sinf(angleToGoal) / 100;
                move.y = self.y + change * cosf(angleToGoal) / 100;
                move.rotation = DEG(angleToGoal);
            }
            float minAngleFace = 90-DEG(atan2(OPP_GOAL_Y - self.y, OPP_GOAL_LEFT_X - self.x));
            float maxAngleFace = 90-DEG(atan2(OPP_GOAL_Y - self.y, OPP_GOAL_RIGHT_X - self.x));
            LIM_ANGLE_180(minAngleFace);
            LIM_ANGLE_180(maxAngleFace);
            if(minAngleFace > maxAngleFace) std::swap(minAngleFace, maxAngleFace);
            if(ball.ballCap && self.y > 1.62 && (self.heading >= minAngleFace && self.heading <= maxAngleFace)) {
                move.kick = true;
            }
            float xToBall = ball.absolute_x - self.x, yToBall = ball.absolute_y - self.y;
            float absBallAngle = atan2(yToBall, xToBall);
            move.rotation = 90-DEG(absBallAngle);
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
            if(distToBall <= balltrack.dribblerDist) move.dribblerSpeed = move.dribbler_maxspeed;
            else move.dribblerSpeed = 0;
        }

        void dribblerAim(){
            float angleToFace = atan2(OPP_GOAL_CENTRE_Y - self.y, OPP_GOAL_CENTRE_X - self.x);
            move.x = OPP_GOAL_MIDDLE_X;
            move.y = OPP_GOAL_MIDDLE_Y;
            move.rotation = 90-DEG(angleToFace);
            float minAngleFace = 90-DEG(atan2(OPP_GOAL_Y - self.y, OPP_GOAL_LEFT_X - self.x));
            float maxAngleFace = 90-DEG(atan2(OPP_GOAL_Y - self.y, OPP_GOAL_RIGHT_X - self.x));
            LIM_ANGLE_180(minAngleFace);
            LIM_ANGLE_180(maxAngleFace);
            if(minAngleFace > maxAngleFace) std::swap(minAngleFace, maxAngleFace);
            if(ball.ballCap && self.y > 1.62 && (self.heading >= minAngleFace && self.heading <= maxAngleFace)) {
                move.kick = true;
                move.dribblerSpeed = -100;
            }
        }

        float last_top_absolute_ball_x = 0;
        float last_top_absolute_ball_y = 0;
        float self_velocityw = 0, self_velocityx = 0, self_velocityy = 0;
        float last_self_w = 0, last_self_x = 0, last_self_y = 0;
        unsigned long last_vel_time = 0;
        float dt_ball;
        bool updatedBallV = false;
        float inst_ball_vx = 0, inst_ball_vy = 0;
        void updateSelfVelocityEWMA(float current_self_w, float current_self_x, float current_self_y) { 
            unsigned long now = micros();
            float dt = (now - last_vel_time) / 1000000.0f; 
            if(!updatedBallV){
                dt_ball += abs((now - last_vel_time) / 1000000.0f);
            }
            else{
                dt_ball = abs((now - last_vel_time) / 1000000.0f);
            }
            // Serial.println(dt, 6);
            if (dt < 1e-6f) {
                return;
            }
            
            if(abs(current_self_w - last_self_w) > 6){ //account for 0 -> 2pi
                if(current_self_w > last_self_w){
                    current_self_w = 2*3.1415 - current_self_w;
                }
                else{
                    last_self_w = 2*3.1415 - last_self_w;
                }
            }

            float inst_vw = (current_self_w - last_self_w) / dt;
            float inst_vx = (current_self_x - last_self_x) / dt;  
            float inst_vy = (current_self_y - last_self_y) / dt; 
            if((ball.absolute_x == 0 && ball.absolute_y == 0) || (ball.absolute_x != last_top_absolute_ball_x || ball.absolute_y != last_top_absolute_ball_y)) {   
                inst_ball_vx = (ball.absolute_x - last_top_absolute_ball_x) / dt_ball;
                inst_ball_vy = (ball.absolute_y - last_top_absolute_ball_y) / dt_ball;
                updatedBallV = true;
            }
            else{
                updatedBallV = false;
            }


            // Exponential Weighted Moving Average update, beta parameter used = 0.8
            self_velocityw = 0.2f * inst_vw + (0.8f) * self_velocityw;    
            self_velocityx = 0.2f * inst_vx + (0.8f) * self_velocityx;
            self_velocityy = 0.2f * inst_vy + (0.8f) * self_velocityy;
            if (updatedBallV && !ball.noBall && abs(pow((inst_ball_vx*inst_ball_vx+inst_ball_vy*inst_ball_vy),0.5)) < 4){    
                ball.vx = 0.5f * inst_ball_vx + (0.5f) * ball.vx;
                ball.vy = 0.5f * inst_ball_vy + (0.5f) * ball.vy;
            }
            else if(abs(pow((inst_ball_vx*inst_ball_vx+inst_ball_vy*inst_ball_vy),0.5)) > 4){
                Serial.println("anomalous data cancelled");
            }
            // DEBUG(noBall);
            // DEBUG(ball.absolute_x);
            // DEBUG(ball.absolute_y);

            // Save current data for next iteration
            last_self_w = current_self_w;
            last_self_x = current_self_x;
            last_self_y = current_self_y;
            last_top_absolute_ball_x = ball.absolute_x;
            last_top_absolute_ball_y = ball.absolute_y;
            last_vel_time = now;
            // DEBUG(self_velocityx);
            // DEBUG(self_velocityy);

        }
        int LA_ball_seen;
        float prev_ball_vx[50];
        float prev_ball_vy[50];
        float prev_prev_ball_vx, prev_prev_ball_vy;
        int pbvx_size = 49;
        bool moveToGoal = false;
        #define pbvx prev_ball_vx
        #define pbvy prev_ball_vy
        float noBallTimer = 0;
        bool checkv(int n, float threshold){ //n = how many previous velocities to check, threshold = requirement for velocities to be consistent
            for(int i = 0; i<n; i++){
                if(abs(pbvx[pbvx_size-n] - pbvx[pbvx_size-(n+1)]) > threshold){
                    return false;
                }
                if(abs(pbvy[pbvx_size-n] - pbvy[pbvx_size-(n+1)]) > threshold){
                    return false;
                }
            }
            return true;
        }
        bool checkzero(int n){
            for(int i = 0; i<n; i++){
                if (pbvx[pbvx_size-i] != 0 || pbvy[pbvx_size-i] != 0){
                    return false;
                }
            }
            return true;
        }
        float targetballposx = FIELD_WIDTH/2;
        float targetballposy = 0.80;
        float lastLAtargetx = 0, lastLAtargety = 0, lastLAtargetAngle = 0;
        float targetheadinglookahead = 0;
        float lastLookAhead = millis();
        float targetballposx_current = 0;
        float targetballposy_current = 0;
        float ballAngle_LA = 0;
        #define LOOK_AHEAD_THRESHOLD_T 0
        #define LOOK_AHEAD_THRESHOLD_DMIN 0
        #define LOOK_AHEAD_THRESHOLD_DMAX 2
        bool rotateBot_LA = true;
        bool tooklastball = false;
        void lookAhead(){
            float v = 1.2;
            float latency = 0.2;
            bool validt = false;
            bool useFrontCam = false;
            bool lookAheadConfirm = false;
            float t;
            float LAball_x, LAball_y, LAball_vx, LAball_vy;
            float targetballposx, targetballposy;
        
            LAball_x = ball.relative_x;
            LAball_y = ball.relative_y;
            LAball_vx = ball.vx;
            LAball_vy = ball.vy;
        
        
            int lookahead_n = 0;
            while(!lookAheadConfirm && lookahead_n < 5){ 
            lookahead_n++;
            float C = LAball_x*LAball_x + LAball_y*LAball_y;
            float B = 2*(LAball_x*LAball_vx + LAball_y*LAball_vy);
            float A = LAball_vx*LAball_vx + LAball_vy*LAball_vy - v*v;
            lookAheadConfirm = false;
        
            if (abs(A) > pow(10, -8) && (B*B - 4*A*C) >= 0){ //we get two solutions for time, so we want to find the minimum time that is not negative
                float t1 = (-1*B - pow((B*B - 4*A*C), 0.5))/(2*A); 
                float t2 = (-1*B + pow((B*B - 4*A*C), 0.5))/(2*A); 
                if (t1 >= 0 && t2 >= 0){
                    t = min(t1, t2);
                    lookAheadConfirm = true;
                }
                else if (t1 >= 0){
                    t = t1;
                    lookAheadConfirm = true;
                }
                else if (t2 >= 0){
                    t = t2;
                    lookAheadConfirm = true;
                }  
            }
        
            if(!lookAheadConfirm){ //ball is too fast
                LAball_vx *= 0.75;
                LAball_vy *= 0.75; 
                lookAheadConfirm = false;        
            } 
        } 
            targetballposx = LAball_x + LAball_vx*t;
            targetballposy = LAball_y + LAball_vy*t;
            targetballposx += self.x + 0.075*sin(self.heading); 
            targetballposy += self.y + 0.075*cos(self.heading);
         
        }
        
        void triggerLookAhead(){
        
            if(ball.noBall && millis() - ball.lastSeenBall <= LAST_SEEN_BALL_TIME){
                ball.absolute_x = ball.last_x;
                ball.absolute_y = ball.last_y;
                // noBall = false;
                
                 tooklastball = true;
            }
            else tooklastball = false;

            for (int i  = 0; i < pbvx_size; i++){ //stores the last 50 values
                pbvx[i] = pbvx[i+1];
                pbvy[i] = pbvy[i+1];
            }
            if (!ball.noBall){
                pbvx[pbvx_size] = ball.vx;
                pbvy[pbvx_size] = ball.vy;
            }
            else{
                pbvx[pbvx_size] = 0;
                pbvy[pbvx_size] = 0;  
            }

            updateSelfVelocityEWMA(RAD(self.heading), self.x, self.y); 
            if(ball.ballCap){
                switches.turnOff = true;
                noBallTimer = 0;
            }
            else{
                switches.turnOff = false;
                if(ball.noBall){

                    if(tooklastball) moveToGoal = false;
                    else moveToGoal = true;
                }
                else{
                    ball.last_x = ball.absolute_x;
                    ball.last_y = ball.absolute_y;
                    noBallTimer = 0;
                    moveToGoal = false;
                }
                
                if(tooklastball){
                    targetballposx = lastLAtargetx;
                    targetballposy = lastLAtargety;
                    ballAngle_LA = lastLAtargetAngle;
                }
                else if(!moveToGoal && ball.noBall && checkzero(10)){ // if no ball, stop bot
                    // esp_led.setPixelColor(0, esp_led.Color(0, 0, 0));
                    // sendI2C(zeroBuffer);
                    // esp_led.show();
                    // lookAhead(); //delete this later if needed
                    targetballposx = self.x;
                    targetballposy = self.y;
                }  
                // else if (sqrtf(ball.vx*ball.vx + ball.vy*ball.vy) < 0.15){} //don't look ahead if velocity is too small
                else if (!moveToGoal && !ball.noBall && checkv(5, 200)){ // if last n values are within x of each other, update look ahead target
                    if(abs(times.curTime - lastLookAhead) > LOOK_AHEAD_THRESHOLD_T){
                        //switches target only if last switch target was sufficiently long ago
                        // esp_led.setPixelColor(0, esp_led.Color(0, 20, 0));
                        // esp_led.show();
                        // Serial.println("look ahead called////////////////////////////////////////////////////////////");
                        lookAhead();
                        lastLookAhead = millis();
                    }
                } 

                if (moveToGoal){
                    // Serial.println("MOVING TO GOAL");
                    targetballposx = FIELD_WIDTH/2;
                    targetballposy = 0.5;
                    // movement(targetballposx, targetballposy, 0);
                }
                //DEBUG(moveToGoal);
                // DEBUG(targetballposx);
                // DEBUG(targetballposy);

            
                float LA_distchange = pow((targetballposx*targetballposx + targetballposy*targetballposy),0.5) - pow((targetballposx_current*targetballposx_current + targetballposy_current*targetballposy_current),0.5);
                //call movement exactly once every loop
                if(!moveToGoal && (targetballposx<0 || targetballposx>FIELD_WIDTH || targetballposy<0 || targetballposy>FIELD_HEIGHT)){
                    targetballposx = targetballposx_current;
                    targetballposy = targetballposy_current;
                    // movement(targetballposx, targetballposy, ballAngle_LA);
                    // sendI2C(zeroBuffer);
                }
                else if (!moveToGoal && abs(LA_distchange) >= LOOK_AHEAD_THRESHOLD_DMIN && abs(LA_distchange) <= LOOK_AHEAD_THRESHOLD_DMAX){
                    //switches target only if new target is far away from current target 
                    targetballposx_current = targetballposx;
                    targetballposy_current = targetballposy; 
                    // movement(targetballposx, targetballposy, ballAngle_LA);
                }
            }   
            ballAngle_LA = rotateBot_LA ? 90-DEG(atan2(ball.relative_y, ball.relative_x)) : 0;
            // ballAngle_LA = rotateBot_LA ? 90-DEG(atan2(ball.absolute_y, ball.absolute_x)) : 0;

            if (moveToGoal){
                ballAngle_LA = 0;
            }
            
            else ballAngle_LA = rotateBot_LA ? 90-DEG(atan2(ball.absolute_y - self.y, ball.absolute_x - self.x)) : 0;
            move.x = targetballposx;
            move.y = targetballposy;
            move.rotation = ballAngle_LA;

            lastLAtargetx = targetballposx;
            lastLAtargety = targetballposy;
            lastLAtargetAngle = ballAngle_LA;
            // DEBUG(ballAngle_LA);
        }
    private:
        } bot;

        #endif