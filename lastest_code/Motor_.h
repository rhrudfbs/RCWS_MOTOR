#pragma once

#define CHECKER 65
#define sendbufsize 40
#define recvbufsize 24

typedef struct Recvcom {
  float GoalOpticalTilt;
  float GoalOpticalPan;
  float GoalBodyTilt;
  float GoalBodyPan;
  unsigned char Permission;  // Current Permission Status

  //Optical
  unsigned char magnification;  // 0 or 1
  unsigned char focus;

  //Gun
  unsigned char fire;

  unsigned char dead1 = 0;
  unsigned char dead2 = 0;
  unsigned char dead3 = 0;

  uint8_t start = CHECKER;
};

typedef struct Sendcom {
  float RealOpticalTilt;
  float RealOpticalPan;
  float RealBodyTilt;
  float RealBodyPan;

  float IMUTilt;  // nrf
  float IMUPan;   // nrf

  float Lidardistance;              // Optical
  float GunVoltage;                 // Gun
  unsigned char SentryPermission;   // Request nrf
  unsigned char fire;               // nrf
  unsigned char RGBmagnification;   // Optical
  unsigned char remaining_bullets;  // Gun
  unsigned char dead1 = 0;
  unsigned char dead2 = 0;
  unsigned char dead3 = 0;

  uint8_t start = CHECKER;
};

class Motor {
public:
  uint8_t ENA;
  uint8_t Dir;
  uint8_t PUL;
  uint8_t int_pin;

  float Ab_Angle = 0;
  float Ab_Pulse = 0;
  float tmp;
  float Limit_Angle = 180;

  float acc = 0;
  float c_Point = 0;

  /////////////수동조작 변수//////////////
  float acc = 0;
  float c_Point = 0;
  float acc_Speed = 1;
  float max_Speed = 1000;
  ////////////////////////////////////////

  Motor(uint8_t E, uint8_t D, uint8_t P, float REV_PUL)  //ENABLE, Direction, PULSE, 분주비
  {
    ENA = E, Dir = D, PUL = P;
    tmp = REV_PUL / (float)360;
  }

  Motor(uint8_t E, uint8_t D, uint8_t P, float REV_PUL, float Gear_Ratio)  //ENABLE, Direction, PULSE, 분주비, 기어비
  {
    ENA = E, Dir = D, PUL = P;
    tmp = REV_PUL * Gear_Ratio / (float)360;
  }

  Motor(uint8_t E, uint8_t in_pin, uint8_t D, uint8_t P, float REV_PUL, float Gear_Ratio)  //ENABLE, interrupt pin, Direction, PULSE, 분주비, 기어비
  {
    ENA = E, Dir = D, PUL = P, int_pin = in_pin;
    tmp = REV_PUL * Gear_Ratio / (float)360;
  }

  void Setting() {
    pinMode(ENA, OUTPUT);
    pinMode(Dir, OUTPUT);
    pinMode(PUL, OUTPUT);
  }

  void Set_MA_Speed(float ac, float ma)  //수동 동작 가속도, 속도 조정
  {
    acc_Speed = ac;
    max_Speed = ma;
  }

  void Manual_Active(uint8_t command)  //수동 동작
  {
    if (command == 'c') {
      acc += acc_Speed;
      if (acc > max_Speed) {
        acc = max_Speed;
      }
    } else if (command == 'r') {
      acc -= acc_Speed;
      if (acc < -max_Speed) {
        acc = -max_Speed;
      }
    } else if (command == 's') {
      if (acc > 0) {
        acc -= acc_Speed;
      } else if (acc < 0) {
        acc += acc_Speed;
      } else if (acc == 0) {
        acc += 0;
      }
    }

    if (c_Point > 0) {
      digitalWrite(Dir, HIGH);
    } else if (c_Point < 0) {
      digitalWrite(Dir, LOW);
    }

    c_Point += acc;
    if (abs(c_Point) >= max_Speed) {
      if (c_Point > 0) {
        Ab_Pulse += 1;
        c_Point -= max_Speed;
      } else if (c_Point < 0) {
        Ab_Pulse -= 1;
        c_Point += max_Speed;
      }
      Ab_Angle = Ab_Pulse / tmp;
      digitalWrite(PUL, HIGH);
      digitalWrite(PUL, LOW);
    }
  }

  /////////////PID 제어 변수///////////////
  float Kp = 1;
  float Ki = 0;
  float Kd = 3;

  float error = 0;
  float errorPrevious = 0;

  float PControl = 0;
  float IControl = 0;
  float DControl = 0;

  float Time = 0;
  float PreTime = 0;

  float PIDControl = 0;

  float PID_acc_Speed = 0.01;
  float PID_acc_max_Speed = 50;
  float PID_max_Speed = 50;
  ////////////////////////////////////////

  void Set_PID_Speed(float ac, float ac_ma, float ma)  //PID 제어 가속도, 속도 조정
  {
    PID_acc_Speed = ac;
    PID_acc_max_Speed = ac_ma;
    PID_max_Speed = ma;
  }

  void PID(float Angle)  //PID 제어
  {
    if(Angle >= Limit_Angle)
    {
      Angle = Limit_Angle;
    }
    else if(Angle <= -Limit_Angle)
    {
      Angle = -Limit_Angle;
    }
    
    Time = micros();
    error = Angle * tmp - Ab_Pulse;

    PControl = Kp * error;
    IControl += Ki * error * (Time - PreTime) / (float)1000000;
    DControl = Kd * (error - errorPrevious) * (float)1000000 / (Time - PreTime);

    PIDControl = PControl + IControl + DControl;

    if (abs(PIDControl) > 5) {
      if (PIDControl > 0) {
        acc += PID_acc_Speed;
        if (acc > PID_acc_max_Speed) {
          acc = PID_acc_max_Speed;
        }
      } else if (PIDControl < 0) {
        acc -= PID_acc_Speed;
        if (acc < -PID_acc_max_Speed) {
          acc = -PID_acc_max_Speed;
        }
      }
    } else if (abs(PIDControl) <= 5) {
      if (error > 0 && c_Point < 0) {
        acc += PID_acc_Speed;
      } else if (error < 0 && c_Point > 0) {
        acc -= PID_acc_Speed;
      } else if (error > 0 && c_Point > 0) {
        acc -= PID_acc_Speed;
      } else if (error < 0 && c_Point < 0) {
        acc += PID_acc_Speed;
      }
    }

    c_Point += acc;

    if (c_Point > 0) {
      digitalWrite(Dir, HIGH);
    } else if (c_Point < 0) {
      digitalWrite(Dir, LOW);
    }

    if (abs(c_Point) >= PID_max_Speed) {
      if (c_Point > 0) {
        Ab_Pulse += 1;
        c_Point -= PID_max_Speed;
      } else if (c_Point < 0) {
        Ab_Pulse -= 1;
        c_Point += PID_max_Speed;
      }
      Ab_Angle = Ab_Pulse / tmp;
      digitalWrite(PUL, HIGH);
      digitalWrite(PUL, LOW);
    }

    errorPrevious = error;
    PreTime = Time;
  }
};
