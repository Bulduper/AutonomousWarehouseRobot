#ifndef _STEPPERMOTOR_H
#define _STEPPERMOTOR_H

#include <Arduino.h>

#define MICROSTEPS 4
#define RESET_OFFSET 5.0f
#define MM_PER_FULL_STEP 0.04f
#define MM_PER_USTEP (MM_PER_FULL_STEP/MICROSTEPS)
#define Z_MAX 210.0f
#define IO_PORT PORTB


enum Direction
{
    FORWARD,
    BACKWARD
};

class StepperMotor
{
    public:
    byte stepPin, dirPin, enPin;
    uint8_t pStep, pDir;
    uint16_t pulsePeriod_us;
    uint32_t pulseCounter = 0;
    float currentStepPulse = 0.0f;
    uint32_t previousStepPulse = 0;

    float currentPosition = 0.0f;
    float targetPosition = 0.0f;

    bool currentDirection;
    bool side;
    bool isPositionControl = true;
    bool isEnabled = true;

    public:
    float desiredStepInterval = 0.0f;
    void init(byte,byte,byte,bool,uint16_t);
    void enable();
    void disable();
    void run();
    void setPositionControl(){isPositionControl = true;}
    void setSpeedControl(){isPositionControl = false;}
    void setRPS(float);
    void setTargetPosition(float _target);
    float getCurrentPosition(){return currentPosition;}
    float getTargetPosition(){return targetPosition;}
    void resetCurrentPosition();
    void regulation();
    bool isTargetAchieved();
};
#endif