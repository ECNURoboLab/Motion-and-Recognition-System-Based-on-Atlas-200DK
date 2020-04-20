/**************************************************************
Description: 锂电池输出电流检测
Author: www.corvin.cn
History: 20190927:init this file;
*************************************************************/
#ifndef _CURRENT_DETECT_
#define _CURRENT_DETECT_

#define CURRENT_AREF    3.3
#define CURRENT_PIN     A1
#define CURRENT_SENS    0.2
#define ADC_RESOLUTION  12
#define ADC_SENS        4095.0
#define CURRENT_OFFSET  1.48   //偏置电压,空载是A1的输出电压

void initCurrentDetect();
float detectCurrent();

#endif
