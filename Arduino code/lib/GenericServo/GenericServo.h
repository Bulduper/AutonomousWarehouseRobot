#include <Arduino.h>

#define MM_PER_STEP 2.14f
#define RESET_OFF 10.0f
#define MAX_PWM 255
#define MIN_PWM 80
#define X_MAX 175.0f

class GenericServo
{
    public:
    float currentPosition;
    float targetPosition;
    bool currentDirection;
    byte in1Pin, in2Pin, enPin;
    bool reversed = false;
    float error_sum = 0.0f;
    

    public:
    GenericServo(byte in1, byte in2, byte en, bool reversed = false);
    void setSpeed(float percent);
    void setTargetPosition(float target);
    void resetPosition();
    float run();
    void setForward();
    void setBackward();
    void setReverse(bool rev){reversed = rev;}
    float getCurrentPosition(){return currentPosition;}
    float getTargetPosition(){return targetPosition;}
    bool getCurrentDirection(){return currentDirection;}
    void pulseUpdate();
    bool isTargetAchieved();
};