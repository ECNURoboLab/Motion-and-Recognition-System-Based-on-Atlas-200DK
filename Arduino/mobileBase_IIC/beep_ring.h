/**********************************************************
Description: 下位机arduino Due上的蜂鸣器模块头文件，
  定义蜂鸣器连接的引脚，各种声音的宏定义。
Author: ROS小课堂 www.corvin.cn
History: 20190927: init this file;
***********************************************************/
#ifndef _BEEP_RING_H_
#define _BEEP_RING_H_

#define  BEEP_PIN   36

#define  POWEROFF_BEEP  0
#define  POWERON_BEEP   1

#define  BEEP_ON  HIGH
#define  BEEP_OFF LOW

void initBeepPin();
void powerOnBeep();
void powerOffBeep();

#endif
