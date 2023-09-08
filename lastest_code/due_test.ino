//CSN 48, CE 50
#include "TimerControl.h"
#include "Motor_.h"

extern void TimerInit(void);
extern void TimerControl(void);
extern void TimerStart(struct Timer* pTimer, int nCount);
extern void TimerReset(struct Timer* pTimer);
extern struct Timer pTimer[];

unsigned char Sendbuf[sendbufsize] = {NULL, };
unsigned char Recvbuf[recvbufsize] = {NULL, };
Sendcom sendcom;
Recvcom recvcom;

//EN, limit switch, Dir, PUL, REV/PUL, Gear Ratio
Motor OpticalTilt(35, 33, 3200, 20);
Motor OpticalPan(53, 51, 3200, 40);
Motor BodyTilt(22, 24, 6400, 20);
Motor BodyPan(26, 28, 6400, 40);

String data;
char cmd;
float Angle1 = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.setTimeout(1);
  TimerInit();

  OpticalTilt.Setting_A4988(23, 25, 27);
  OpticalPan.Setting_A4988(41, 43, 45);
  BodyTilt.Setting();
  BodyPan.Setting();
  
  sendcom.start = 65;
}


void loop() {
  // put your main code here, to run repeatedly:
  Serial.readBytes(Recvbuf, sizeof(Recvbuf));
  if (Recvbuf[23] == 65) {
    memcpy(&recvcom, Recvbuf, sizeof(Recvbuf));
  }

  sendcom.RealOpticalTilt = OpticalTilt.Ab_Angle;
  sendcom.RealOpticalPan = OpticalPan.Ab_Angle;
  sendcom.RealBodyTilt = BodyTilt.Ab_Angle;
  sendcom.RealBodyPan = BodyPan.Ab_Angle;
  // sendcom.IMUTilt = ;
  // sendcom.IMUPan = ;
  // sendcom.Lidardistance = ;
  // sendcom.GunVoltage = ;
  // sendcom.SentryPermission = ;
  // sendcom.fire = ;
  // sendcom.RGBmagnification = ;
  // sendcom.remaining_bullets = ;


  memcpy(Sendbuf, &sendcom, sizeof(Sendcom));

  Serial.write(Sendbuf, sizeof(Sendbuf));
  delay(100);



  // if(Serial.available())
  // {
  //   data = Serial.readStringUntil('\n');
  //   cmd = data[0];
  //   Angle1 = data.toFloat();
  // }
  // Serial.println(OpticalTilt.Ab_Angle);
  // Serial.print(' ');
  // Serial.println(OpticalPan.Ab_Angle);
}

void TC3_Handler(void)
{
  OpticalTilt.PID(recvcom.GoalOpticalTilt);
  OpticalPan.PID(recvcom.GoalOpticalPan);
  BodyTilt.PID(recvcom.GoalBodyTilt);
  BodyPan.PID(recvcom.GoalBodyPan);


  //pticalTilt.PID(Angle1);
  //OpticalPan.PID(Angle1);
  TC_GetStatus(TC1, 0);
  TimerControl();
}