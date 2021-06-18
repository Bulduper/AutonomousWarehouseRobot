#include <Arduino.h>
#include "StepperMotor.h"
#include "GenericServo.h"
#include <Servo.h>

//PWM on 3,5,6,9,10,11 (PWM on pins 9&10 disabled because of Servo library)
//PWM on pins 3&11 disabled because of use of Timer 2 interrupt :/ 
#define ENC_IN 2
#define X_MOT_EN 11
#define X_MOT_IN1 8
#define X_MOT_IN2 9
#define X_ENDSTOP 4
#define Z_MOT_DIR 13
#define Z_MOT_STEP 10
#define Z_MOT_EN 12
#define Z_ENDSTOP A1

#define LEFT_Y_EN 5
#define LEFT_Y_IN1 A0
#define LEFT_Y_IN2 7

#define RIGHT_Y_EN 6
#define RIGHT_Y_IN1 A4
#define RIGHT_Y_IN2 A5

#define GRIPPER_SERVO 3
#define GRIPPER_SERVO_MIN 40
#define GRIPPER_SERVO_MAX 170

#define L_SENSOR A3
#define R_SENSOR A2

#define pulsePeriod_us 80
#define BLACK_THRESHOLD 0

StepperMotor zMotor;
GenericServo xMotor = GenericServo(X_MOT_IN1,X_MOT_IN2,X_MOT_EN);
GenericServo l_yMotor = GenericServo(LEFT_Y_IN1,LEFT_Y_IN2,LEFT_Y_EN);
GenericServo r_yMotor = GenericServo(RIGHT_Y_IN1,RIGHT_Y_IN2,RIGHT_Y_EN);
Servo gServo;

char incomingByte; // for incoming serial data
long lastInterrupt = 0;
String receivedMsg;
int intCnt;
float lineCoeff = 0.12f;
bool followLine = false;

bool xAwaitAnswer, yAwaitAnswer, zAwaitAnswer, gAwaitAnswer;

uint8_t timer2cnt = 0;

void encoderInterruptRoutine();
void setTimerInterrupt();
void decodeMessage();
void debugSensors();
void goAlongLine();
void setServo(int _degrees);

void setup() {

  Serial.begin(9600);
  pinMode(ENC_IN, INPUT);
  pinMode(X_ENDSTOP, INPUT_PULLUP);
  pinMode(Z_ENDSTOP, INPUT_PULLUP);

  pinMode(L_SENSOR, INPUT_PULLUP);
  pinMode(R_SENSOR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_IN),encoderInterruptRoutine,FALLING);

  zMotor.init(Z_MOT_STEP,Z_MOT_DIR, Z_MOT_EN,false,(uint16_t)pulsePeriod_us);
  gServo.attach(GRIPPER_SERVO);
  setTimerInterrupt();

  xMotor.setTargetPosition(-1000.0f);
  zMotor.setTargetPosition(-1000.0f);
  //zMotor.setRPS(-1.0f);
  setServo(140);
}

void loop() {
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();
    if(incomingByte == '\n' || incomingByte == '\r')
    {
      decodeMessage();
      receivedMsg = "";
    }
    else
    {
      receivedMsg+=incomingByte;
    }
  }
  //Serial.println(receivedMsg);
  if(!digitalRead(X_ENDSTOP))xMotor.resetPosition();
  if(!digitalRead(Z_ENDSTOP))
  {
    //Serial.println("ZSTOP");
    zMotor.resetCurrentPosition();
  }

  float pid_out = xMotor.run();
  zMotor.regulation();
  if(followLine)goAlongLine();
  debugSensors();

  if(xMotor.isTargetAchieved() && xAwaitAnswer)
  {
    xAwaitAnswer = false;
    Serial.print("X OK\n\r");
  }
  if(zMotor.isTargetAchieved() && zAwaitAnswer)
  {
    zAwaitAnswer = false;
    Serial.print("Z OK\n\r");
  }


  //Serial.println(zMotor.currentPosition);
  // Serial.print(xMotor.targetPosition);
  // Serial.print("\t");
  // Serial.print(xMotor.currentPosition);
  // Serial.print("\t");
  // Serial.print(xMotor.targetPosition-xMotor.currentPosition);
  // Serial.print("\t");
  // Serial.print(pid_out);
  // Serial.print("\t");
  // Serial.println(intCnt);
}

void setTimerInterrupt()
{
  TCCR2B = 0; //reset control reg B
  TCCR2A = 0; //reset control register A
  TCCR2A |= (1 << WGM21); //Set counter 2 to CTC (clear timer on compare) mode
  TCCR2B |= (1 << CS21); // set prescaler to 8
  TCNT2 = 0;
  OCR2A = 2*pulsePeriod_us-1; //call every 20us ( every (40-1)th pulse)
  TIMSK2 |= (1 << OCIE2A); // enable timer2 compare interrupt

}

ISR(TIMER2_COMPA_vect)
{
  //leftStepper.rotate();
  zMotor.run();
}

void encoderInterruptRoutine()
{
  //tu znak zapytania
  if(micros()-lastInterrupt>50)
  {
    xMotor.pulseUpdate();
    lastInterrupt = micros();
    intCnt++;

  }
      //Serial.println('i');
}


void decodeMessage()
{
 //Serial.print("I received: ");
  //Serial.println(receivedMsg);

  char prefix = receivedMsg[0];
  char symbol = receivedMsg[1];
  float value = receivedMsg.substring(2).toFloat();
  //X/Y/Z/G
  switch(prefix)
  {
    //X=120 (mm)
    //X+10 
    //X-24
    //X? (zwraca 125.2)
    case 'X':
    {
      
      if(symbol=='=')
      {
        xMotor.setTargetPosition(value);
        xAwaitAnswer = true;
      }
      else if(symbol=='+')
      {
        xMotor.setTargetPosition(xMotor.getCurrentPosition()+value);
        xAwaitAnswer = true;
      }
      else if(symbol=='-')
      {
        xMotor.setTargetPosition(xMotor.getCurrentPosition()-value);
        xAwaitAnswer = true;
      }
      else if(symbol=='?')
      {
        Serial.println(xMotor.getCurrentPosition());
      }
      break;
    }
    //Y-
    //Y+
    //Y=1
    case 'Y':
    {
      
      if(symbol=='=')
      {
        //l_yMotor.setSpeed(value);
        //r_yMotor.setSpeed(value);
        if(value == 0.0f)
        {
          followLine = false;
          l_yMotor.setSpeed(0.0f);
          r_yMotor.setSpeed(0.0f);
        }
        else
        {
          followLine = true;
        }
        //yAwaitAnswer = true;
      }
      if(symbol=='+')
      {
        l_yMotor.setSpeed(55.0f);
        r_yMotor.setSpeed(55.0f);
        //yAwaitAnswer = true;
      }
      if(symbol=='-')
      {
        l_yMotor.setSpeed(-55.0f);
        r_yMotor.setSpeed(-55.0f);
        //yAwaitAnswer = true;
      }
      break;
    }


    case 'Z':
    {
      zMotor.setPositionControl();
      if(symbol=='=')
      {
        zMotor.setTargetPosition(value);
        zAwaitAnswer = true;
      }
      else if(symbol=='+')
      {
        zMotor.setTargetPosition(zMotor.getCurrentPosition()+value);
        zAwaitAnswer = true;
      }
      else if(symbol=='-')
      {
        zMotor.setTargetPosition(zMotor.getCurrentPosition()-value);
        zAwaitAnswer = true;
      }
      else if(symbol=='?')
      {
        Serial.println(zMotor.getCurrentPosition());
      }
      break;
    }

    case 'z':
    {
     zMotor.setSpeedControl(); 
      if(symbol=='=')
      {
        zMotor.setRPS(value);
        zAwaitAnswer = true;
      }
      // else if(symbol=='+')
      // {
      //   zMotor.setTargetPosition(zMotor.getTargetPosition()+value);
      //   zAwaitAnswer = true;
      // }
      // else if(symbol=='-')
      // {
      //   zMotor.setTargetPosition(zMotor.getTargetPosition()-value);
      //   zAwaitAnswer = true;
      // }
      // else if(symbol=='?')
      // {
      //   Serial.println(zMotor.getCurrentPosition());
      // }
      break;
    }

    case 'G':
    {
      //G=150 (deg) \\zacisniety chwytak=G=170 \\otwarty chwytak G=60   
      if(symbol=='=')
      {
       setServo((int)value);
       gAwaitAnswer = true;
      }
      else if(symbol=='?')
      {
        Serial.println(gServo.read());
      }
      break;
    }
    case 'C':
    {
      
      if(symbol=='=')
      {
        lineCoeff = value;
      }
      else if(symbol=='?')
      {
        Serial.println(lineCoeff);
      }
      break;
    }
  }
}

void setServo(int _degrees)
{
  gServo.write(constrain(_degrees,GRIPPER_SERVO_MIN,GRIPPER_SERVO_MAX));
}

void goAlongLine()
{
  static bool lastDirection;
  uint16_t leftSensor = analogRead(L_SENSOR);
  uint16_t rightSensor = analogRead(R_SENSOR);
  int16_t sensorDifference = leftSensor-rightSensor;
  float lineError = lineCoeff * sensorDifference;
  //Serial.println(lineError);
  float avThr = 70.0f;


  if(sensorDifference>0)
  {
    r_yMotor.setSpeed(avThr+lineError);
    // if(lineError>70.0f)l_yMotor.setSpeed(0.0f);
    // else l_yMotor.setSpeed(avThr);
    l_yMotor.setSpeed(avThr-lineError);
    //l_yMotor.setSpeed(value-lineError);
  }
  else if(sensorDifference<0)
  {
    //r_yMotor.setSpeed(value+lineError);
    l_yMotor.setSpeed(avThr-lineError);
    r_yMotor.setSpeed(avThr+lineError);
    // if(lineError<-70.0f)r_yMotor.setSpeed(0.0f);
    // else r_yMotor.setSpeed(avThr);
  } 
}

void debugSensors()
{
  Serial.print("LEFT LINE SENSOR:\t\t RIGHT LINE SENSOR:\n");
  Serial.print("\t\t\t");
  Serial.print(analogRead(L_SENSOR));
  Serial.print("\t\t\t");
  Serial.println(analogRead(R_SENSOR));
}