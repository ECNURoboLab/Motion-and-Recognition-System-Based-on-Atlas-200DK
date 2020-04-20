/**************************************************************
Description: Encoder driver function definitions
Author: www.corvin.cn
History: 20180209:init this file;
*************************************************************/
#ifndef __ENCODER_DRIVER_H__
#define __ENCODER_DRIVER_H__

//A wheel encode pin
#define ENC_A_PIN_A  26  //pin 26 
#define ENC_A_PIN_B  27  //pin 27 

//B wheel encode pin
#define ENC_B_PIN_A  30  //pin 30
#define ENC_B_PIN_B  31  //pin 31 

//C wheel encode pin
#define ENC_C_PIN_A  34  //pin 34 
#define ENC_C_PIN_B  35  //pin 35 

#define A_WHEEL      1
#define B_WHEEL      2
#define C_WHEEL      3

void initEncoders(void);
long readEncoder(int i);
void resetEncoders(void);

#endif
