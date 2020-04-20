/***************************************************************
  Copyright: 2016-2019 ROS小课堂 www.corvin.cn
  Author: corvin
  Description:
    用于arduino的串口获得命令类中函数的实现.
  History:
   20181207: initial this file.
*************************************************************/
void serialData::resetCmdParam()
{
  this->cmd_chr = ENTER_CHAR;

  memset(this->argv1, ENTER_CHAR, sizeof(this->argv1));
  memset(this->argv2, ENTER_CHAR, sizeof(this->argv2));
  memset(this->argv3, ENTER_CHAR, sizeof(this->argv3));

  this->argCnt   = 0;
  this->argIndex = 0;
}

void sendData::resetData()
{
  ackMsgStr = "";
  this->byteCnt = 0;
}
