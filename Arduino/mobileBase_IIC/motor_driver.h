/***************************************************************
Description: Motor driver function definitions
Author: www.corvin.cn
History: 20180209: init this file;
****************************************************************/
#ifndef __MOTOR_DRIVER_H__
#define __MOTOR_DRIVER_H__

#define FORWARDS     true
#define BACKWARDS    false

void initMotorsPinMode(void);
bool directionWheel(int wheel);
void setMotorSpeeds(int ASpeed, int BSpeed, int CSpeed);

#endif
