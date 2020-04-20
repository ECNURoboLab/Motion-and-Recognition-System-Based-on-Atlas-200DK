/**************************************************************
Description: 锂电池电压检测
Author: www.corvin.cn
History: 20190927:init this file;
*************************************************************/
#ifndef _VOLTAGE_DETECT_
#define _VOLTAGE_DETECT_

#define AREF         3.3
#define VOLTAGE_PIN  A0

void initVoltageDetect();
float detectVoltage();

#endif
