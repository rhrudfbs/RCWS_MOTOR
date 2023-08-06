#pragma once
/*class Step_Motor {
public:
    int ENA;
    int Dir;
    int PUL;
    int PUL_REV;
    float tmp;
    float Ab_Angle;
    int Ab_Pulse;
    Step_Motor(int E, int D, int P, int PR) {
        Ab_Angle = 0;
        Ab_Pulse = 0;
        ENA = E, Dir = D, PUL = P, PUL_REV = PR;
        tmp = (float)PUL_REV / (float)360;
    }
    void Setting() {
        pinMode(ENA, OUTPUT);
        pinMode(Dir, OUTPUT);
        pinMode(PUL, OUTPUT);
    }
    void Move(float Angle)
    {
        float ex = Ab_Pulse;

        int lowSpeed = 1200;
        int highSpeed = 20;

        float ratio = 10;

        float change;

        float d = lowSpeed;

        if (Angle > (float)Ab_Pulse / tmp)
        {
            change = ((float)lowSpeed - (float)highSpeed) / (Angle * tmp - ex) * ratio;
            digitalWrite(Dir, HIGH);
            for (Ab_Pulse; Ab_Pulse < Angle * tmp; Ab_Pulse += 1)
            {
                digitalWrite(PUL, HIGH);
                digitalWrite(PUL, LOW);
                delayMicroseconds(d);

                if (Ab_Pulse < ex + (Angle * tmp - ex) / ratio)
                {
                    d -= change;
                }
                else if (Ab_Pulse > Angle * tmp - (Angle * tmp - ex) / ratio)
                {
                    d += change;
                }
                //Ab_Pulse += 1;
            }
        }
        else if (Angle < (float)Ab_Pulse / tmp)
        {
            change = ((float)lowSpeed - (float)highSpeed) / (ex - Angle * tmp) * ratio;
            digitalWrite(Dir, LOW);
            for (Ab_Pulse; Ab_Pulse > Angle * tmp; Ab_Pulse -= 1)
            {
                digitalWrite(PUL, HIGH);
                digitalWrite(PUL, LOW);
                delayMicroseconds(d);

                if (Ab_Pulse > ex - (ex - Angle * tmp) / ratio)
                {
                    d -= change;
                }
                else if (Ab_Pulse < Angle * tmp + (ex - Angle * tmp) / ratio)
                {
                    d += change;
                }
                //Ab_Pulse -= 1;
            }
        }
        Ab_Angle = (float)Ab_Pulse / tmp;
    }
};*/

class Motor {
public:
    int ENA;
    int Dir;
    int PUL;
    int PUL_REV;
    int Gear_Ratio;

    float tmp;

    float Ab_Pulse;
    float Save_Ab_Pulse;
    float Ab_Angle;

    int LowSpeed;
    int HighSpeed;

    float Acc_Ratio;

    float Change_Speed;

    float Speed;

    bool Start;

    Motor(int E, int D, int P, int PR, int GR)
    {
        Start = 0;
        Ab_Angle = 0;
        Save_Ab_Pulse = Ab_Pulse;
        Ab_Pulse = 0;
        ENA = E, Dir = D, PUL = P, PUL_REV = PR;
        tmp = (float)PUL_REV * (float)GR / (float)360;
    }

    void Set_Motor(int Low, int High, int Acc_Ra)
    {
        pinMode(ENA, OUTPUT);
        pinMode(Dir, OUTPUT);
        pinMode(PUL, OUTPUT);
        LowSpeed = Low;
        HighSpeed = High;
        Acc_Ratio = Acc_Ra;
    }

    void Set_Change_Speed(float Angle)
    {
        Save_Ab_Pulse = Ab_Pulse;
        if (Save_Ab_Pulse < Angle)
        {
            Change_Speed = ((float)LowSpeed - (float)HighSpeed) / (Angle * tmp - Save_Ab_Pulse) * Acc_Ratio;
        }
        else if (Save_Ab_Pulse > Angle)
        {
            Change_Speed = ((float)LowSpeed - (float)HighSpeed) / (Save_Ab_Pulse - Angle * tmp) * Acc_Ratio;
        }
        Speed = LowSpeed;
        Start = 1;
    }

    void Move_Motor(float Angle)
    {
        if (Start)
        {
            if (Ab_Pulse < Angle * tmp)
            {
                digitalWrite(Dir, HIGH);
                digitalWrite(PUL, HIGH);
                digitalWrite(PUL, LOW);
                if (Ab_Pulse < Save_Ab_Pulse + (Angle * tmp - Save_Ab_Pulse) / Acc_Ratio)
                {
                    Speed -= Change_Speed;
                }
                else if (Ab_Pulse > Angle * tmp - (Angle * tmp - Save_Ab_Pulse) / Acc_Ratio)
                {
                    Speed += Change_Speed;
                }
                delayMicroseconds(Speed);
                if (Ab_Pulse + 1 < Angle * tmp)
                {
                    Ab_Pulse += 1;
                }
                else Ab_Pulse += 1, Start = 0;
                Ab_Angle = Ab_Pulse / tmp;
            }
            else if (Ab_Pulse > Angle * tmp)
            {
                digitalWrite(Dir, LOW);
                digitalWrite(PUL, HIGH);
                digitalWrite(PUL, LOW);
                if (Ab_Pulse > Save_Ab_Pulse - (Save_Ab_Pulse - Angle * tmp) / Acc_Ratio)
                {
                    Speed -= Change_Speed;
                }
                else if (Ab_Pulse < Angle * tmp + (Save_Ab_Pulse - Angle * tmp) / Acc_Ratio)
                {
                    Speed += Change_Speed;
                }
                delayMicroseconds(Speed);
                if (Ab_Pulse - 1 > Angle * tmp)
                {
                    Ab_Pulse -= 1;
                }
                else Ab_Pulse -= 1, Start = 0;
                Ab_Angle = Ab_Pulse / tmp;
            }
        }
    }
};