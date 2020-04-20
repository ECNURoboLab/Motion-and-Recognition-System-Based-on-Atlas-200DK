/**************************************************************
Description:  Encoder definitions需要连接到arduino DUE的正确引脚上：
   Encoder A: connect interrupt [pin 26, 27];
   Encoder B: connect interrupt [pin 30, 31];
   Encoder C: connect intterupt [pin 34, 35]
Author: www.corvin.cn ROS小课堂
History: 20190909:init this file;
**************************************************************/
#include "motor_driver.h"
#include "encoder_driver.h"

static volatile long A_enc_cnt = 0L;
static volatile long B_enc_cnt = 0L;
static volatile long C_enc_cnt = 0L;

/*init encoder connect pin, config ISR functions*/
void initEncoders(void)
{
  pinMode(ENC_A_PIN_A, INPUT);
  pinMode(ENC_A_PIN_B, INPUT);
  attachInterrupt(ENC_A_PIN_A, encoderA_ISR, FALLING);
  attachInterrupt(ENC_A_PIN_B, encoderA_ISR, FALLING);

  pinMode(ENC_B_PIN_A, INPUT);
  pinMode(ENC_B_PIN_B, INPUT);
  attachInterrupt(ENC_B_PIN_A, encoderB_ISR, FALLING);
  attachInterrupt(ENC_B_PIN_B, encoderB_ISR, FALLING);

  pinMode(ENC_C_PIN_A, INPUT);
  pinMode(ENC_C_PIN_B, INPUT);
  attachInterrupt(ENC_C_PIN_A, encoderC_ISR, FALLING);
  attachInterrupt(ENC_C_PIN_B, encoderC_ISR, FALLING);
}

/* Interrupt routine for A encoder, taking care of actual counting */
void encoderA_ISR (void)
{
  if (directionWheel(A_WHEEL) == BACKWARDS)
  {
    A_enc_cnt--;
  }
  else 
  {
    A_enc_cnt++;
  }
}

/* Interrupt routine for B encoder, taking care of actual counting */
void encoderB_ISR (void) 
{
  if (directionWheel(B_WHEEL) == BACKWARDS)
  {
    B_enc_cnt--;
  }
  else 
  {
    B_enc_cnt++;
  }
}

/* Interrupt routine for C encoder, taking care of actual counting */
void encoderC_ISR (void) 
{
  if (directionWheel(C_WHEEL) == BACKWARDS)
  {
    C_enc_cnt--;
  }
  else 
  {
    C_enc_cnt++;
  }
}

/* Wrap the encoder reading function */
long readEncoder(int i) 
{
  if (i == A_WHEEL)
  {
    return A_enc_cnt;
  }
  else if (i == B_WHEEL)
  {
    return B_enc_cnt;
  }
  else
  {
    return C_enc_cnt;
  }
}

/* Wrap the encoder count reset function */
void resetEncoders(void) 
{
  A_enc_cnt = 0L;
  B_enc_cnt = 0L;
  C_enc_cnt = 0L;
}
