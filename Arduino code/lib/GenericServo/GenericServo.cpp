#include "GenericServo.h"

GenericServo::GenericServo(byte in1, byte in2, byte en, bool reversed = false)
: in1Pin(in1), in2Pin(in2), enPin(en)
{
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    //pinMode(en, OUTPUT);
    setReverse(reversed);
}

void GenericServo::setTargetPosition(float target)
{
    targetPosition = min(target,X_MAX);
}

void GenericServo::setForward()
{
    digitalWrite(in1Pin, reversed);
    digitalWrite(in2Pin, !reversed);
    currentDirection = true;
}

void GenericServo::setBackward()
{
    digitalWrite(in1Pin, !reversed);
    digitalWrite(in2Pin, reversed);
    currentDirection = false;
}


void GenericServo::pulseUpdate()
{
    if(currentDirection)
    {
        currentPosition += MM_PER_STEP;
    }
    else{
        currentPosition -= MM_PER_STEP;
    }
}

float GenericServo::run()
{
    float error = targetPosition-currentPosition;
    if(fabs(error)<=0.6f)error=0;
    error_sum +=error;

    float kP = 30.0f;
    float kI = 0.0f;
    float I_term = constrain(kI*error_sum,-100.0f,100.0f);
    float output = kP*error + I_term;
    output = constrain(output,-100.0,100.0);
    setSpeed(output);
    return output;
}
void GenericServo::setSpeed(float percent)
{
    int pwm;
    percent = constrain(percent,-100.0f,100.0f);
    if(percent<0.0)
    {
        setBackward();
        pwm = -percent*MAX_PWM/100;
        if(pwm<MIN_PWM)pwm=0;
        //pwm = constrain(pwm,MIN_PWM,MAX_PWM);
        analogWrite(enPin,pwm);    
    }
    else
    {
        setForward();
        pwm = percent*MAX_PWM/100;
        if(pwm<MIN_PWM)pwm=0;
        //pwm = constrain(pwm,MIN_PWM,MAX_PWM);
        //Serial.println(pwm);
        analogWrite(enPin,pwm);    
    }
}

void GenericServo::resetPosition()
{
    currentPosition = -RESET_OFF;
    targetPosition = 0.0f;
    //setSpeed(0);
}

bool GenericServo::isTargetAchieved()
{
    return fabs(targetPosition-currentPosition) < MM_PER_STEP;

}