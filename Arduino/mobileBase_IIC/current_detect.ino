/***************************************************************
  Description:锂电池输出电流检测功能。
  Author: www.corvin.cn
  History: 20190927: init this file;
****************************************************************/
void initCurrentDetect()
{
  pinMode(CURRENT_PIN, INPUT);
  analogReference(AR_DEFAULT); //调用板载3.3V默认基准源
  analogReadResolution(ADC_RESOLUTION);
}

float detectCurrent()
{
  int sum = 0;
  int cnt = 10;

  for (int index = 0; index < cnt; index++)
  {
    sum += analogRead(CURRENT_PIN);
  }

  int readValue = sum / cnt;
  float current = (readValue / ADC_SENS * CURRENT_AREF - CURRENT_OFFSET) / CURRENT_SENS;

  return current;
}
