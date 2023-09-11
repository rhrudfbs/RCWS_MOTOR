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

//Dir, PUL, REV/PUL, Gear Ratio
Motor OpticalTilt(35, 33, 3200, 20);
Motor OpticalPan(53, 51, 3200, 20);
Motor BodyTilt(20, 22, 6400, 20);
Motor BodyPan(24, 26, 6400, 40);

uint8_t OpticalTilt_interrupt = 34;
uint8_t OpticalPan_interrupt = 36;
uint8_t BodyTilt_interrupt = 38;
uint8_t BodyPan_interrupt = 40;

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
  
  pinMode(OpticalTilt_interrupt, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(OpticalTilt_interrupt), OpticalTilt_Init, FALLING);
  pinMode(OpticalPan_interrupt, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(OpticalPan_interrupt), OpticalPan_Init, FALLING);
  pinMode(BodyTilt_interrupt, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BodyTilt_interrupt), BodyTilt_Init, FALLING);
  pinMode(BodyPan_interrupt, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BodyPan_interrupt), BodyPan_Init, FALLING);
  
  sendcom.start = 65;
}


void loop() {
  //put your main code here, to run repeatedly:
  // Serial.readBytes(Recvbuf, sizeof(Recvbuf));
  // if (Recvbuf[23] == 65) {
  //   memcpy(&recvcom, Recvbuf, sizeof(Recvbuf));
  // }

  // sendcom.RealOpticalTilt = OpticalTilt.Ab_Angle;
  // sendcom.RealOpticalPan = OpticalPan.Ab_Angle;
  // sendcom.RealBodyTilt = BodyTilt.Ab_Angle;
  // sendcom.RealBodyPan = BodyPan.Ab_Angle;
  // // sendcom.IMUTilt = ;
  // // sendcom.IMUPan = ;
  // // sendcom.Lidardistance = ;
  // // sendcom.GunVoltage = ;
  // // sendcom.SentryPermission = ;
  // // sendcom.fire = ;
  // // sendcom.RGBmagnification = ;
  // // sendcom.remaining_bullets = ;


  // memcpy(Sendbuf, &sendcom, sizeof(Sendcom));

  // Serial.write(Sendbuf, sizeof(Sendbuf));
  // delay(100);



  if(Serial.available())
  {
    data = Serial.readStringUntil('\n');
    cmd = data[0];
    Angle1 = data.toFloat();
  }
  Serial.println(OpticalTilt.Ab_Angle);
  Serial.print(' ');
  Serial.println(OpticalPan.Ab_Angle);
}

void TC3_Handler(void)
{
  // OpticalTilt.PID(recvcom.GoalOpticalTilt);
  // OpticalPan.PID(recvcom.GoalOpticalPan);
  // BodyTilt.PID(recvcom.GoalBodyTilt);
  // BodyPan.PID(recvcom.GoalBodyPan);


  OpticalTilt.PID(Angle1);
  OpticalPan.PID(Angle1);
  TC_GetStatus(TC1, 0);
  TimerControl();
}

void OpticalTilt_Init()
{
  OpticalTilt.Ab_Angle = 0;
  OpticalTilt.Ab_Pulse = 0;
}

void OpticalPan_Init()
{
  OpticalPan.Ab_Angle = 0;
  OpticalPan.Ab_Pulse = 0;
}

void BodyTilt_Init()
{
  BodyTilt.Ab_Angle = 0;
  BodyTilt.Ab_Pulse = 0;
}

void BodyPan_Init()
{
  BodyPan.Ab_Angle = 0;
  BodyPan.Ab_Pulse = 0;
}