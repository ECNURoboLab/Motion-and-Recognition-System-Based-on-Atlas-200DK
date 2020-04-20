/*********************************************************************
  Description: A set of simple serial commands to control a omniWheel drive
  robot and receive back sensor and odometry data. Default configuration
  assumes use of an Arduino DUE + motor driver board. Edit the
  readEncoder() and setMotorSpeed() wrapper functions if using different
  motor controller or encoder method.
  Author: www.corvin.cn
  History: 20190716:init code;
    20190927：增加蜂鸣器控制代码,电压,电流检测的代码;
*********************************************************************/
#include <Servo.h>
#include <Wire.h>
#include <string.h>
#include "beep_ring.h"
#include "commands.h"
#include "sensors.h"
#include "serialData.h"
#include "servos_control.h"
#include "motor_driver.h"
#include "encoder_driver.h"
#include "pid_controller.h"
#include "current_detect.h"
#include "voltage_detect.h"


/******************** USER AREAR ********************/
/* current code version */
#define VERSION    1.0

/* Stop the robot if it hasn't received a movement command
  in this number of milliseconds */
#define  AUTO_STOP_INTERVAL  150
/******************** USER END **********************/

/* SERIAL_OUT port baud rate */
#define BAUDRATE   57600

#define IIC_ADDR  0x08

/* Run the PID loop at 40 times per second -Hz */
#define PID_RATE   40

/* Convert the rate into an interval */
const int PID_INTERVAL = 1000 / PID_RATE;

/* Track the next time we make a PID calculation */
unsigned long nextPID = PID_INTERVAL;

long lastMotorCommand = AUTO_STOP_INTERVAL;

/***************DEBUG DEFINE******************/
#define SERIAL_OUT Serial


/* Run a command.  Commands are defined in commands.h */
int runCommand()
{
  int i = 0;
  float out = 0;
  char *str = NULL;
  char *p = serialObj.argv1;
  int pid_args[12];

  if (serialObj.cmd_chr != UPDATE_PID)
  {
    serialObj.arg1 = atoi(serialObj.argv1);
    serialObj.arg2 = atoi(serialObj.argv2);
    serialObj.arg3 = atoi(serialObj.argv3);
  }
  sendObj.resetData();

  switch (serialObj.cmd_chr)
  {
    case GET_BAUDRATE: //'b'
      {
        SERIAL_OUT.println(BAUDRATE);
        int baud = BAUDRATE;
        sendObj.ackMsgStr.concat(baud);
      }
      break;

    case ANALOG_READ:  //'a'
      SERIAL_OUT.println(analogRead(serialObj.arg1));
      sendObj.ackMsgStr.concat(analogRead(serialObj.arg1));
      break;

    case DIGITAL_READ: //'d'
      SERIAL_OUT.println(digitalRead(serialObj.arg1));
      sendObj.ackMsgStr.concat(digitalRead(serialObj.arg1));
      break;

    case ANALOG_WRITE:
      analogWrite(serialObj.arg1, serialObj.arg2);
      SERIAL_OUT.println("OK");
      sendObj.ackMsgStr.concat("OK");
      break;

    case DIGITAL_WRITE: //'w'
      if (serialObj.arg2 == 0)
      {
        digitalWrite(serialObj.arg1, LOW);
      }
      else if (serialObj.arg2 == 1)
      {
        digitalWrite(serialObj.arg1, HIGH);
      }
      SERIAL_OUT.println("OK");
      sendObj.ackMsgStr.concat("OK");
      break;

    case PIN_MODE:
      if (serialObj.arg2 == 0)
      {
        pinMode(serialObj.arg1, INPUT);
      }
      else if (serialObj.arg2 == 1)
      {
        pinMode(serialObj.arg1, OUTPUT);
      }
      SERIAL_OUT.println("OK");
      sendObj.ackMsgStr.concat("OK");
      break;

    case READ_ENCODERS:  //'e'
      SERIAL_OUT.print(readEncoder(A_WHEEL));
      sendObj.ackMsgStr.concat(readEncoder(A_WHEEL));
      SERIAL_OUT.print(" ");
      sendObj.ackMsgStr.concat(" ");
      SERIAL_OUT.print(readEncoder(B_WHEEL));
      sendObj.ackMsgStr.concat(readEncoder(B_WHEEL));
      SERIAL_OUT.print(" ");
      sendObj.ackMsgStr.concat(" ");
      SERIAL_OUT.println(readEncoder(C_WHEEL));
      sendObj.ackMsgStr.concat(readEncoder(C_WHEEL));
      break;

    case RESET_ENCODERS: //'r'
      resetEncoders();
      resetPID();
      SERIAL_OUT.println("OK");
      sendObj.ackMsgStr.concat("OK");
      break;

    case MOTOR_SPEEDS: //'m'
      lastMotorCommand = millis();  /* Reset the auto stop timer */
      if (serialObj.arg1 == 0 && serialObj.arg2 == 0 && serialObj.arg3 == 0)
      {
        setMotorSpeeds(0, 0, 0);
        resetPID();
        setMoveStatus(0);
      }
      else
      {
        setMoveStatus(1);
      }
      setWheelPIDTarget(serialObj.arg1, serialObj.arg2, serialObj.arg3);
      SERIAL_OUT.println("OK");
      sendObj.ackMsgStr.concat("OK");
      break;

    case UPDATE_PID: //'u'
      memset(pid_args, 0, sizeof(pid_args));
      while ((str = strtok_r(p, ":", &p)) != '\0')
      {
        pid_args[i] = atoi(str);
        i++;
      }
      updatePIDParam(A_WHEEL, pid_args[0], pid_args[1], pid_args[2],  pid_args[3]);
      updatePIDParam(B_WHEEL, pid_args[4], pid_args[5], pid_args[6],  pid_args[7]);
      updatePIDParam(C_WHEEL, pid_args[8], pid_args[9], pid_args[10], pid_args[11]);
      SERIAL_OUT.println("OK");
      sendObj.ackMsgStr.concat("OK");
      break;

    case BEEP_RING: //'p'
      if (serialObj.arg1 == POWERON_BEEP)
      {
        basePowerOnBeep();
      }
      else if (serialObj.arg1 == POWEROFF_BEEP)
      {
        basePowerOffBeep();
      }
      SERIAL_OUT.println("OK");
      sendObj.ackMsgStr.concat("OK");
      break;

    case READ_PIDIN:
      SERIAL_OUT.print(readPidIn(A_WHEEL));
      sendObj.ackMsgStr.concat(readPidIn(A_WHEEL));
      SERIAL_OUT.print(" ");
      sendObj.ackMsgStr.concat(" ");
      SERIAL_OUT.print(readPidIn(B_WHEEL));
      sendObj.ackMsgStr.concat(readPidIn(B_WHEEL));
      SERIAL_OUT.print(" ");
      sendObj.ackMsgStr.concat(" ");
      SERIAL_OUT.println(readPidIn(C_WHEEL));
      sendObj.ackMsgStr.concat(readPidIn(C_WHEEL));
      break;

    case READ_PIDOUT:
      SERIAL_OUT.print(readPidOut(A_WHEEL));
      sendObj.ackMsgStr.concat(readPidOut(A_WHEEL));
      SERIAL_OUT.print(" ");
      sendObj.ackMsgStr.concat(" ");
      SERIAL_OUT.print(readPidOut(B_WHEEL));
      sendObj.ackMsgStr.concat(readPidOut(B_WHEEL));
      SERIAL_OUT.print(" ");
      sendObj.ackMsgStr.concat(" ");
      SERIAL_OUT.println(readPidOut(C_WHEEL));
      sendObj.ackMsgStr.concat(readPidOut(C_WHEEL));
      break;

    case GET_VOLTAGE:
      out = detectVoltage();
      SERIAL_OUT.println(out, 2);
      sendObj.ackMsgStr.concat('3');
      break;

    case GET_CURRENT:
      out = detectCurrent();
      SERIAL_OUT.println(out, 2);
      sendObj.ackMsgStr.concat(out);
      break;

    case CODE_VERSION:
      SERIAL_OUT.println(VERSION);
      sendObj.ackMsgStr.concat(VERSION);
      break;

    case SET_SERVO_POS:   // 's'
      myServos[serialObj.arg1].setTargetPos(serialObj.arg2, serialObj.arg3);
      SERIAL_OUT.println("OK");
      sendObj.ackMsgStr.concat("OK");
      break;

    case GET_SERVO_POS:   // 't'
      for (byte i = 0; i < SERVOS_CNT; i++)
      {
        SERIAL_OUT.print(myServos[i].getCurrentPos());
        sendObj.ackMsgStr.concat(myServos[i].getCurrentPos());
        SERIAL_OUT.print(' ');
        sendObj.ackMsgStr.concat(" ");
      }
      SERIAL_OUT.println("");
      break;

    default:
      SERIAL_OUT.println("Invalid Command");
      sendObj.ackMsgStr.concat("Invalid Command");
      break;
  }
  sendObj.ackMsgStr.concat("\r");
  
  return 0;
}

/* Setup function--runs once at startup */
void setup()
{
  SERIAL_OUT.begin(BAUDRATE);
  serialObj.resetCmdParam();
  initSensors();
  initBeepPin();
  initEncoders();
  initMotorsPinMode();
  initVoltageDetect();
  initCurrentDetect();
  resetPID();
  Wire.begin(IIC_ADDR);
  Wire.onReceive(receData);
  Wire.onRequest(sendData);

  /* when power on init all servos position */
  for (byte i = 0; i < SERVOS_CNT; i++)
  {
    myServos[i].initServo(myServoPins[i], servoInitPosition[i], 0);
  }
}

void sendData()
{
  Wire.write(sendObj.ackMsgStr.c_str());
}

void receData(int byteCount)
{
  while (Wire.available())
  {
    char tmp_chr = Wire.read();  //Read the next character

    if (tmp_chr == 13)  //Terminate a command with a CR
    {
      runCommand();
      serialObj.resetCmdParam();
    }
    else if (tmp_chr == ' ') // Use spaces to delimit parts of the command
    {
      serialObj.argCnt++;
      serialObj.argIndex = 0;
    }
    else // process single-letter command
    {
      if (serialObj.argCnt == 0) // The first arg is the single-letter command
      {
        serialObj.cmd_chr = tmp_chr;
      }
      else if (serialObj.argCnt == 1) // Get after cmd first param
      {
        serialObj.argv1[serialObj.argIndex] = tmp_chr;
        serialObj.argIndex++;
      }
      else if (serialObj.argCnt == 2)
      {
        serialObj.argv2[serialObj.argIndex] = tmp_chr;
        serialObj.argIndex++;
      }
      else if (serialObj.argCnt == 3)
      {
        serialObj.argv3[serialObj.argIndex] = tmp_chr;
        serialObj.argIndex++;
      }
    }
  }//end while()
}

/* Read and parse input from the serial port and run any valid commands.
   Run a PID calculation at the target interval and check for auto-stop conditions.*/
void loop()
{
  while (SERIAL_OUT.available())
  {
    char tmp_chr = SERIAL_OUT.read();  //Read the next character

    if (tmp_chr == 13)  //Terminate a command with a CR
    {
      runCommand();
      serialObj.resetCmdParam();
    }
    else if (tmp_chr == ' ') // Use spaces to delimit parts of the command
    {
      serialObj.argCnt++;
      serialObj.argIndex = 0;
    }
    else // process single-letter command
    {
      if (serialObj.argCnt == 0) // The first arg is the single-letter command
      {
        serialObj.cmd_chr = tmp_chr;
      }
      else if (serialObj.argCnt == 1) // Get after cmd first param
      {
        serialObj.argv1[serialObj.argIndex] = tmp_chr;
        serialObj.argIndex++;
      }
      else if (serialObj.argCnt == 2)
      {
        serialObj.argv2[serialObj.argIndex] = tmp_chr;
        serialObj.argIndex++;
      }
      else if (serialObj.argCnt == 3)
      {
        serialObj.argv3[serialObj.argIndex] = tmp_chr;
        serialObj.argIndex++;
      }
    }
  }//end while()

  //run a PID calculation at the appropriate intervals
  if (millis() > nextPID)
  {
    updatePID();
    nextPID += PID_INTERVAL;
  }

  //Check to see if we have exceeded the auto-stop interval
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL)
  {
    setMotorSpeeds(0, 0, 0); //stop motors
    setMoveStatus(0);
  }

  // Check everyone servos isEnabled, when true will move servo. Other don't move servo.
  for (byte i = 0; i < SERVOS_CNT; i++)
  {
    myServos[i].moveServo();
  }
}
