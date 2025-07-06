#ifndef DATA_H
#define DATA_H

struct Localisation{
    float x = 0, y = 0, heading = 0;
    bool onLine = false;
    uint8_t line_status = 0;
} self;

struct Ball{
    float angle = 0, dist = 0, vx = 0, vy = 0;
    float relative_x = 0, relative_y = 0;
    float absolute_x = 0, absolute_y = 0;
    float last_x = 0, last_y = 0;
    bool noBall = false;
    int ballCap = 0;
    int lastBallCap = 0, lastNoBallCap = 0, lastSeenBall = 0;
    int ballCapTime = 1000;
} ball;

struct Goal{
    bool frontPathClear = false;
    int open_rows_start = 0, open_rows_end = 0, total_rows = 24;
    float fov = 100;
} goal;

struct Switches{
    bool topOff = true, turnOff = false;
    bool motorTest = false;
} switches;

struct Times{
    int lastDribblerRev = 0;
    int lastFault = 0;
    int curTime = 0;
    int motorTestPressed = 0, motorTestWait = 1000;
} times;

struct Movement{
    float x = 0, y = 0, rotation = 0;
    bool kick = false;
    int dribbler_maxspeed = 150, dribblerSpeed = 0;
    int translation_default = 40, rotation_default = 25;
    int translation_ballcap = 20, rotation_ballcap = 10, rotation_lowered = 5;
    int min_translation = -translation_default, max_translation = translation_default;
    int min_rotation = -rotation_default, max_rotation = rotation_default;
    int x_offset = 18, y_offset = 18, rotation_offset = 12;
    bool dont_move = false;
} move;

struct State{
    bool isDefender = true;
    int lastChange = 0;
    int lastType = 0, curType = 0;
    int curStratIdx = 0;

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
        ATTACK_MODE1 = 8,
        ATTACK_MODE2 = 9
    } strategies;
} state;

#endif