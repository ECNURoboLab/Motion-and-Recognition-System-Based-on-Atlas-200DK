/***************************************************************
 Copyright: 2016-2019 ROS小课堂 www.corvin.cn
 Author: corvin
 Description:
    定义舵机操作的类SweepServo,舵机连接的引脚和初始的舵机角度.在这里连接
    的两个舵机分别连接5,6两个引脚,5号引脚连接左右旋转的舵机,6号引脚连接
    上下移动的舵机.其中设置左右旋转的舵机初始角度为90度,上下转动舵机的初始
    角度为0度.
 History:
   20190928: initial this comment.
*************************************************************/
#ifndef _SERVOS_CONTROL_H_
#define _SERVOS_CONTROL_H_

#define  SERVOS_CNT  2

#define  SERVO_ENABLE   1
#define  SERVO_DISABLE  0

//Define All Servos's Pins
byte myServoPins[SERVOS_CNT] = {5, 6};

// Initial Servo Position [0, 180] degrees
int servoInitPosition[SERVOS_CNT] = {0, 90};

class SweepServo
{
  public:
    SweepServo();
    void initServo(int servoPin, unsigned int initPosition, unsigned int stepDelayMs);
    void setTargetPos(unsigned int targetPos, unsigned int stepDelayMs);
    int getCurrentPos(void);
    void moveServo(void);
    Servo getServoObj();

  private:
    Servo servo;
    unsigned int stepDelayMs;
    unsigned long lastMoveTime;
    int currentPosDegrees;
    int targetPosDegrees;
};

SweepServo myServos[SERVOS_CNT];

#endif
