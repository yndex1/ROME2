/*
 * Controller.h
 * Copyright (c) 2024, ZHAW
 * All rights reserved.
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <cstdlib>
#include <mbed.h>
#include "EncoderCounter.h"
#include "Motion.h"
#include "Point.h"
#include "LowpassFilter.h"
#include "ThreadFlag.h"

/**
 * This class implements a controller that regulates the
 * speed of the two motors of the ROME2 mobile robot.
 */
class Controller {
    
    public:
        
                Controller(PwmOut& pwmLeft, PwmOut& pwmRight, EncoderCounter& counterLeft, EncoderCounter& counterRight);
        virtual ~Controller();
        void    setTranslationalVelocity(float velocity);
        void    setRotationalVelocity(float velocity);
        float   getActualTranslationalVelocity();
        float   getActualRotationalVelocity();
        void    setX(float x);
        float   getX();
        void    setY(float y);
        float   getY();
        void    setAlpha(float alpha);
        float   getAlpha();
        void    correctPoseWithBeacon(Point actualBeacon, Point measuredBeacon);
        
    private:
        
        static const unsigned int   STACK_SIZE = 4096;  // stack size of thread, given in [bytes]
        static const float          PERIOD;             // period of control task, given in [s]
        
        static const float  M_PI;                       // the mathematical constant PI
        static const float  WHEEL_DISTANCE;             // distance between wheels, given in [m]
        static const float  WHEEL_RADIUS;               // radius of wheels, given in [m]
        static const float  MAXIMUM_VELOCITY;           // maximum wheel velocity, given in [rpm]
        static const float  MAXIMUM_ACCELERATION;       // maximum wheel acceleration, given in [rpm/s]
        static const float  COUNTS_PER_TURN;            // encoder resolution
        static const float  LOWPASS_FILTER_FREQUENCY;   // given in [rad/s]
        static const float  KN;                         // speed constant in [rpm/V]
        static const float  KP;                         // speed controller gain in [V/rpm]
        static const float  MAX_VOLTAGE;                // battery voltage in [V]
        static const float  MIN_DUTY_CYCLE;             // minimum duty-cycle
        static const float  MAX_DUTY_CYCLE;             // maximum duty-cycle
        static const float  SIGMA_TRANSLATION;          // standard deviation of estimated translation per period, given in [m]
        static const float  SIGMA_ORIENTATION;          // standard deviation of estimated orientation per period, given in [rad]
        static const float  SIGMA_DISTANCE;             // standard deviation of distance measurement, given in [m]
        static const float  SIGMA_GAMMA;                // standard deviation of angle measurement, given in [rad]

        PwmOut&             pwmLeft;
        PwmOut&             pwmRight;
        EncoderCounter&     counterLeft;
        EncoderCounter&     counterRight;
        float               translationalVelocity;
        float               rotationalVelocity;
        float               actualTranslationalVelocity;
        float               actualRotationalVelocity;
        float               desiredSpeedLeft;
        float               desiredSpeedRight;
        float               actualSpeedLeft;
        float               actualSpeedRight;
        Motion              motionLeft;
        Motion              motionRight;
        short               previousValueCounterLeft;
        short               previousValueCounterRight;
        LowpassFilter       speedLeftFilter;
        LowpassFilter       speedRightFilter;
        float               x;
        float               y;
        float               alpha;
        float               p[3][3];
        ThreadFlag          threadFlag;
        Thread              thread;
        Ticker              ticker;
        
        void    sendThreadFlag();
        void    run();
};

#endif /* CONTROLLER_H_ */
