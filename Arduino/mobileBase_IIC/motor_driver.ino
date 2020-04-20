/***************************************************************
  Description: Motor driver definitions;
  Author: www.corvin.cn
  Hisotry: 20180209: init this file;
*************************************************************/
#include "motor_driver.h"


//three motors control pin define
static const int stby_A_Pin = 22;
static const int stby_B_Pin = 23;

static const int A_IN1  = 24;
static const int A_IN2  = 25;
static const int A_PWM  = 2;   //A wheel pwm pin

static const int B_IN1  = 28;
static const int B_IN2  = 29;
static const int B_PWM  = 3;   //B wheel pwm pin

static const int C_IN1  = 32;
static const int C_IN2  = 33;
static const int C_PWM  = 4;   //C wheel pwm pin

static boolean direcA = FORWARDS;
static boolean direcB = FORWARDS;
static boolean direcC = FORWARDS;

/* Wrap the motor driver initialization,
   set all the motor control pins to outputs **/
void initMotorsPinMode(void)
{
  pinMode(A_IN1, OUTPUT);
  pinMode(A_IN2, OUTPUT);

  pinMode(B_IN1, OUTPUT);
  pinMode(B_IN2, OUTPUT);

  pinMode(C_IN1, OUTPUT);
  pinMode(C_IN2, OUTPUT);

  pinMode(stby_A_Pin, INPUT);
  pinMode(stby_B_Pin, INPUT);

  digitalWrite(stby_A_Pin, HIGH);
  digitalWrite(stby_B_Pin, HIGH);
}

/*
   get wheel run direction
*/
boolean directionWheel(int wheel)
{
  if (wheel == A_WHEEL)
  {
    return direcA;
  }
  else if (wheel == B_WHEEL)
  {
    return direcB;
  }
  else
  {
    return direcC;
  }
}

// A convenience function for setting motors speed
void setMotorSpeeds(int ASpeed, int BSpeed, int CSpeed)
{ 
  if (ASpeed > 0) //drive motor A
  {
    direcA = FORWARDS;
    digitalWrite(A_IN1, LOW);
    digitalWrite(A_IN2, HIGH);
    analogWrite(A_PWM, ASpeed);
  }
  else if (ASpeed < 0)
  {
    direcA = BACKWARDS;
    digitalWrite(A_IN1, HIGH);
    digitalWrite(A_IN2, LOW);
    analogWrite(A_PWM, -ASpeed);
  }
  else
  {
    digitalWrite(A_IN1, LOW);
    digitalWrite(A_IN2, LOW);
    analogWrite(A_PWM, HIGH);
  }

  if (BSpeed > 0) //drive motor B
  {
    direcB = FORWARDS;
    digitalWrite(B_IN1, LOW);
    digitalWrite(B_IN2, HIGH);
    analogWrite(B_PWM, BSpeed);
  }
  else if (BSpeed < 0)
  {
    direcB = BACKWARDS;
    digitalWrite(B_IN1, HIGH);
    digitalWrite(B_IN2, LOW);
    analogWrite(B_PWM, -BSpeed);
  }
  else
  {
    digitalWrite(B_IN1, LOW);
    digitalWrite(B_IN2, LOW);
    analogWrite(B_PWM, HIGH);
  }

  if (CSpeed > 0) //drive motor C
  {
    direcC = FORWARDS;
    digitalWrite(C_IN1, LOW);
    digitalWrite(C_IN2, HIGH);
    analogWrite(C_PWM, CSpeed);
  }
  else if (CSpeed < 0)
  {
    direcC = BACKWARDS;
    digitalWrite(C_IN1, HIGH);
    digitalWrite(C_IN2, LOW);
    analogWrite(C_PWM, -CSpeed);
  }
  else
  {
    digitalWrite(C_IN1, LOW);
    digitalWrite(C_IN2, LOW);
    analogWrite(C_PWM, HIGH);
  }
}
