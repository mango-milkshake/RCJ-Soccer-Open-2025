#ifndef DATA_H
#define DATA_H

// #define HIGHER_DRIB_THRESH

struct Localisation{
    float x = 0, y = 0, heading = 0;
    bool onLine = false;
    uint8_t line_status = 0;
} self;

struct Ball{
    float angle = 0, dist = 0, vx = 0, vy = 0;
    float relative_x = 0, relative_y = 0;
    float absolute_x = 0, absolute_y = 0;
    float last_x = 0, last_y = 0, last_dist = 0;
    bool noBall = false, tooklastball = false, tooklastballcap = false;
    int ballCap = 0;
    int lastBallCap = 0, lastNoBallCap = 0, lastSeenBall = 0;
    int ballCapTime = 1000;
} ball;

struct Goal{
    bool frontPathClear = false;
    int open_rows_start = 0, open_rows_end = 0, total_rows = 24;
    float fov = 80;
} goal;

struct Switches{
    bool topOff = true, turnOff = false, dribOff = false;
    bool motorTest = false;
} switches;

struct Times{
    int lastDribblerRev = 0;
    int lastFault = 0;
    int curTime = 0;
    int motorTestPressed = 0, motorTestWait = 1000;
    int lastBallhideTime = 0;
    int bhScore_timeout = 0;
    int defender_balltrack = 0;
} times;

struct Movement{
    float x = 0, y = 0, rotation = 0;
    float last_x = 0, last_y = 0, last_rotation = 0;
    bool kick = false;
    bool dont_move = false;
    int translation_default = 60, rotation_default = 60;
    int translation_ballcap = 20, rotation_ballcap = 10, rotation_lowered = 5;
    int min_translation = -translation_default, max_translation = translation_default;
    int min_rotation = -rotation_default, max_rotation = rotation_default;
    int x_offset = 18, y_offset = 18, rotation_offset = 6;
} move;

struct Dribbler{
    //dribbler
    int maxspeed = 150, reach_speed = 120, track_speed = 120, minspeed = 40;
    int speed = 0, desired = 0;
    #ifdef HIGHER_DRIB_THRESH
    float max_voltage = 0.15;
    #else
    float max_voltage = 0.1;
    #endif
    float exceed_thresh = 0.70;
    int inc = 1, dec = 2;
    int num_frames = 5, num_check = 10; // used as size of arrays below respectively
    float v[5] = {0, 0, 0, 0, 0}; 
    int c[10]; 
    int avg_cnt = 0, check_cnt = 0;
    bool avg_filled = false, check_filled = false;
    float analogval = 0, voltage = 0, sum_avg = 0, sum_check = 0, avgV = 0;
} drib;

struct Comms{
    float ypos = 0, xpos = 0;
    int bh_stage = 0;
    int isPresent = 0;
    int type = 0;
    bool hasBall = false;
} comms;

struct State{
    bool isDefender = true;
    int lastChange = 0;
    int lastType = 0, curType = 0;
    int botType = 0;  
    int botID = 0;
    int curStratIdx = 0;
    bool ready_to_shoot = false;
    int ballhide_stage = 0;
    int strip = true;
    int topStrat = 1;
    bool goleft = false;

    enum StratType{
        NO_BALL = 0,
        BALL_TRACK = 1,
        SCORE = 2
    };

    enum Strategies{
        NONE = 0,
        MOVE_TO_POINT = 1,
        OSCILLATE_ABOUT_POINT = 2,
        NO_DRIBBLER_BALL_TRACK = 3,
        NO_DRIBBLER_SCORE = 4,
        DRIBBLER_BALL_TRACK = 5,
        DRIBBLER_SCORE = 6,
        DEFEND = 7,
        BALLHIDE = 8,
        ATTACK_MODE2 = 9,
        LOOK_AHEAD = 10,
        BALLHIDE_LEAD = 11,
        BALLHIDE_FOLLOW = 12, 
        ATTACK_BASIC = 13,
        DEFEND_BASIC = 14
    } strategies;
} state;

#endif