/***************************************************************
  Copyright: 2016-2019 ROS小课堂 www.corvin.cn
  Author: corvin
  Description:
    用于从arduino的串口获得命令的类，包含各种成员变量和函数.
  History:
   20181207: initial this file.
*************************************************************/
#ifndef _SERIAL_DATA_H_
#define _SERIAL_DATA_H_

#define  ENTER_CHAR  '\r'

class serialData
{
  public:
    void resetCmdParam();

    // A pair of varibles to help parse serial commands
    byte argCnt;
    byte argIndex;

    // Variable to hold the current single-character command
    char cmd_chr;

    // Character arrays to hold the first, second and third arguments
    char argv1[48];
    char argv2[4];
    char argv3[4];

    // The arguments converted to integers
    int arg1;
    int arg2;
    int arg3;
};

class sendData
{
  public:
    void resetData();
    
    String ackMsgStr;
    int byteCnt;
};

serialData serialObj;
sendData sendObj;
#endif
