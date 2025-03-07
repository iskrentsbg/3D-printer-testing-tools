#include "fan.h"

void ControlFan(int speed){
    int fanSpeed = map(speed, 0, 100, 0,255);
    analogWrite(GPIO_27, fanSpeed);
}
