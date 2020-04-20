/************************************************
  Description: 在arduino DUE板上接蜂鸣器来生成各种声音
  反馈当前下位机的运行状态。
  Author: ROS小课堂 www.corvin.cn
  History: 20190927: init this file;
*************************************************/

void initBeepPin()
{
  pinMode(BEEP_PIN, OUTPUT);
  digitalWrite(BEEP_PIN, HIGH);//init beep not ring
  delay(100);
  digitalWrite(BEEP_PIN, LOW);
}

void basePowerOnBeep()
{
  for (int i = 0; i < 2; i++)
  {
    digitalWrite(BEEP_PIN, BEEP_ON);
    delay(100);
    digitalWrite(BEEP_PIN, BEEP_OFF);
    delay(100);
  }
//  digitalWrite(BEEP_PIN, HIGH);
}

void basePowerOffBeep()
{
  for (int i = 0; i < 2; i++)
  {
    digitalWrite(BEEP_PIN, BEEP_ON);
    delay(500);
    digitalWrite(BEEP_PIN, BEEP_OFF);
    delay(500);
  }
//  digitalWrite(BEEP_PIN, HIGH);
}
