// I love nanachi

#include <math.h>
//car control.cpp
#define levelMax 2012.0
#define levelMin 989.0
#define levelMid 1500.0
#define levelGap 1023.0

#define buttonHigh  2000
#define buttonLow   999

//=======change parameter here==========
#define WeelRadius  10 
#define poleMax     100

#define Speedrange 255
#define viewDegreeMax 3.1415926 / 8
//======================================

int GasValue = 0;       // range 0~100
int RightWheelFix = 0;   // range -100~100
int LeftWheelFix = 0;    // range -100~100

float viewDegree;         

    // 0=foward, 1=backward

int PoleLength;

int RightSpeed;         // output range -100~100
int LeftSpeed;
bool GoBackward = 0;
// cha1 = Gas
// cha2 = turn
// cha3 = up/down
// cha4 = right/left (no use)
// cha4 = baku
int* car_control(int cha1, int cha2, int cha3, int cha4, int cha5) {
    GasValue = (((cha1 - levelMin) / levelGap)) * Speedrange;

    RightWheelFix = -((cha2 - levelMid) / (levelGap / 2) * Speedrange);
    LeftWheelFix = ((cha2 - levelMid) / (levelGap / 2) * Speedrange);

    viewDegree = ((cha3 - levelMin) / levelGap) * viewDegreeMax;
    PoleLength = (WeelRadius / sin(viewDegree + asin(WeelRadius/poleMax)));

    if (cha5 == buttonHigh)   GoBackward = 1;
    else                      GoBackward = 0;

    if (GoBackward == 0) {
        RightSpeed = GasValue + RightWheelFix;
        LeftSpeed  = GasValue + LeftWheelFix;
        if (RightSpeed > Speedrange) RightSpeed = Speedrange;
        if (LeftSpeed > Speedrange)  LeftSpeed  = Speedrange;  
    } else {
        RightSpeed = -(GasValue + RightWheelFix);
        LeftSpeed  = -(GasValue + LeftWheelFix);
        if (RightSpeed < -Speedrange) RightSpeed = -Speedrange;
        if (LeftSpeed  < -Speedrange) LeftSpeed  = -Speedrange;
    }

    static int ctrlValue[3] = {0, 0, 0};
    ctrlValue[0] = RightSpeed;
    ctrlValue[1] = LeftSpeed;
    ctrlValue[2] = PoleLength;

    return ctrlValue;
}
