#include "StepperMotor.h"

void StepperMotor::init(byte step, byte dir, byte en, bool side, uint16_t pulse_T)
{
    stepPin = step;
    dirPin = dir;
    enPin = en;
    pStep = (1<<step%8);
    pDir = (1<<dir%8);
    this->side = side;
    pulsePeriod_us = pulse_T;
    
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(enPin, OUTPUT);
}

void StepperMotor::enable()
{
    digitalWrite(enPin, LOW);
    isEnabled = true;
}

void StepperMotor::disable()
{
    digitalWrite(enPin, HIGH);
    isEnabled = false;
}

void StepperMotor::setTargetPosition(float _target)
{
    targetPosition = min(_target,Z_MAX);
}

/*
We can perform the rotation in steps that are made only in fixed intervals (i.e. 30us, 40us, 50us)
So for a specific range of rational RPS values we want to achieve, the motors would actually spin at the constant speed, 
eventually causing the whole values jumping back and forth and bad oscillations for higher speeds.
The solution is varying the step intervals so that they are rational in average (and close to the desired value).
In the result no oscillation occur and we may even increase the PULSE_PERIOD.
run() and setRPS() methods implement that solution.
For instace:
- if the desiredStepInterval equals 8.2 then every 5th step will be performed after 9th (and not 8th) pulse
- or if the desiredStepInterval equals 11.9 then 9 out of 10 steps will be performed after 12th (and not 11th) pulse
*/

void StepperMotor::run()
{
    if(!isEnabled || (currentPosition >= Z_MAX && !(currentDirection ^ side)))return;
    pulseCounter++;
    if (pulseCounter > currentStepPulse - previousStepPulse)
    {
        IO_PORT ^= ((-(currentDirection ^ side) ^ IO_PORT) & pDir);
        //digitalWrite(dirPin, currentDirection ^ side); //set direction of rotation on dirPin according to what motor it is
        
        pulseCounter = 0;

        previousStepPulse = currentStepPulse;
        currentStepPulse += desiredStepInterval;    
    }
    else if (pulseCounter == 1) //create a step signal
    {
        IO_PORT |=pStep;
        //digitalWrite(stepPin, HIGH);
    }
    else if (pulseCounter == 2)
    {
        IO_PORT &= ~pStep;
        //digitalWrite(stepPin, LOW);
        if(currentDirection ^ side)currentPosition -= MM_PER_USTEP;
        else currentPosition += MM_PER_USTEP;
    }
}

void StepperMotor::setRPS(float speed_rps)
{
    float speed_rps_abs = fabsf(speed_rps);
    if (speed_rps_abs < 0.001f)
    {
        disable();
        desiredStepInterval = 0.0f;
        currentStepPulse = 0.0f;
        previousStepPulse = 0;
        return;
    }
    enable();
    currentDirection = (speed_rps > 0 ? FORWARD : BACKWARD);
    desiredStepInterval = 5000.0f / (float)pulsePeriod_us / (float)MICROSTEPS / speed_rps_abs;
    currentStepPulse = desiredStepInterval;
    previousStepPulse = 0;
}

void StepperMotor::regulation()
{
    if(!isPositionControl)return;
    float error = targetPosition - currentPosition;
    float kP = 0.5f;
    float output = kP*error;
    output = constrain(output,-3.0f, 3.0f);

    if(fabsf(error)<MM_PER_FULL_STEP/(float)MICROSTEPS)
    {
        disable();
        return;
    }
    else enable();
    setRPS(output);
}

void StepperMotor::resetCurrentPosition()
{
    setRPS(0.0f);
    setPositionControl();
    currentPosition = -RESET_OFFSET;
    targetPosition = 0.0f;
}

bool StepperMotor::isTargetAchieved()
{
    return fabs(targetPosition-currentPosition) < MM_PER_FULL_STEP;

}