#include "Arduino.h"
#include <avr/io.h>
#include "batteryMonitor.h"
//This code uses direct port manipulation which only works on Arduino Mega 2560
// Serial1 L
// Serial2 R

#define MaxSpeed 25
#define LOOPTIME 10
#define RIGHT_WHEEL 1
#define LEFT_WHEEL 2
#define BatteryObserverTime 500
#define TimeOut 5000 //TimeOut = Max.Distance(cm) * 58

BatteryMonitor bm;

float omega_left_target = 0.0;
float omega_right_target = 0.0;
float omega_left_actual = 0;
float omega_right_actual = 0;
unsigned long lastMilli = 0;
unsigned long lastRecvMilli = 0;
unsigned long lastBool = 0;

unsigned int current_left = 0;
unsigned int current_right = 0;

char rec;
bool recieveComplete = false;
String recbuf;
String feedback;
String Lspeed, Rspeed;
int voltage = 0;
int current = 0;
bool connected;

//left wheel CCW forward
void sendCmd_wheel_angularVel_L()
{
  int left_target_send = int(omega_left_target / (double(MaxSpeed) / 32767)); //convert rad/s to 16 bit integer to send
  byte buf[4];
  buf[0] = '{';                        //send start byte
  buf[1] = highByte(left_target_send); //send high byte
  buf[2] = lowByte(left_target_send);  //send low byte
  buf[3] = '}';                        //send stop byte
  Serial1.write(buf, sizeof(buf));
}

//right wheel CW forward
void sendCmd_wheel_angularVel_R()
{
  int right_target_send = int(-omega_right_target / (double(MaxSpeed) / 32767)); //convert rad/s to 16 bit integer to send
  byte buf[4];
  buf[0] = '{';                         //send start byte
  buf[1] = highByte(right_target_send); //send high byte
  buf[2] = lowByte(right_target_send);  //send low byte
  buf[3] = '}';                         //send stop byte
  Serial2.write(buf, sizeof(buf));
}

//from 328P
void readFeadback_angularVel_L()
{
  int actual_receive;
  if (Serial1.available() >= 5)
  {
    char rT_L = (char)Serial1.read(); //read actual speed from Uno
    if (rT_L == '{')
    {
      char commandArray_L[4];
      Serial1.readBytes(commandArray_L, 4);
      byte rH_L = commandArray_L[0];
      byte rL_L = commandArray_L[1];
      byte rCS_L = commandArray_L[2];
      char rP_L = commandArray_L[3];
      if (rP_L == '}')
      {
        actual_receive = (rH_L << 8) + rL_L;
        omega_left_actual = double(actual_receive * (double(MaxSpeed) / 32767)); //convert received 16 bit integer to actual speed
        //max current is 20400mA, 255 * 80 = 20400mA
        current_left = (rCS_L * 80);
      }
    }
  }
}

//from 328P
void readFeadback_angularVel_R()
{
  int actual_receive;
  if (Serial2.available() >= 5)
  {
    char rT_R = (char)Serial2.read(); //read actual speed from Uno
    if (rT_R == '{')
    {
      char commandArray_R[4];
      Serial2.readBytes(commandArray_R, 4);
      byte rH_R = commandArray_R[0];
      byte rL_R = commandArray_R[1];
      byte rCS_R = commandArray_R[2];
      char rP_R = commandArray_R[3];
      if (rP_R == '}')
      {
        actual_receive = (rH_R << 8) + rL_R;
        omega_right_actual = double(-actual_receive * (double(MaxSpeed) / 32767)); //convert received 16 bit integer to actual speed
        current_right = rCS_R * 80;
      }
    }
  }
}

bool checksum(float firstpara, float secondpara, float sum)
{
  float mySum = firstpara + secondpara;
  if ((mySum - sum)>-0.1 && (mySum - sum)<0.1)
  {
    return true;
  }
  else
  {
    return false;
  }
}

//to odorid
void sendFeedBack(String fb)
{
  if (recieveComplete)
  {
    Serial.print(fb);
    recieveComplete = false;
    delay(1);
  }
}

//from odroid
void readCMD()
{
  while (!recieveComplete)
  {
    if (Serial.available() > 0)
    {
      rec = Serial.read();
      recbuf += rec;
    }
    else
    {
      recieveComplete = true;
    }
  }
  sendFeedBack(recbuf);
  recbuf = "";
}



void setup()
{
  Serial.begin(115200);
}

void loop()
{
  readCMD();                   //read from odroid
}
