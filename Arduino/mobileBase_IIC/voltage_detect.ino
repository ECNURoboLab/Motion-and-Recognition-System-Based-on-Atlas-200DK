/***************************************************************
  Description: 使用电阻分压原理来设计的电路来进行测量锂电池当前电压，根据
      锂电池的电池放电曲线，可以计算出当前锂电池剩余当前剩余的电量百分比。
  Author: www.corvin.cn ROS小课堂
  History: 20191006: init this file;
****************************************************************/

void initVoltageDetect()
{
  pinMode(VOLTAGE_PIN, INPUT);
}

float detectVoltage()
{
  float voltage = 0.0;
  int sum = 0;
  int cnt = 10;

  for (int i = 0; i < cnt; i++)
  {
    sum += analogRead(VOLTAGE_PIN);
  }
  sum = sum / cnt;
  voltage = sum / ADC_SENS * AREF * 9.0;

  return voltage;
}
