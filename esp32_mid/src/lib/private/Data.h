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
    bool noBall = false, ballCap = false;
    int lastBallCap = 0, lastNoBallCap = 0, lastSeenBall = 0;
} ball;

struct Switches{
    bool topOff = true, turnOff = false;
} switches;

struct Movement{
    float x = 0, y = 0, rotation = 0;
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
        ATTACK_MODE1 = 8
    } strategies;
} state;

#endif