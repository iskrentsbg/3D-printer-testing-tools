#ifndef NTC_H
#define NTC_H

#include <Arduino.h>

#define BETA_VALUE 3950.0
#define R1 100000
#define THERMISTORPIN 32
#define ADC_MAX 4095.0
#define VS 3.3
#define RO 100000.0
#define TO 298.15





float GetTemperature();




#endif