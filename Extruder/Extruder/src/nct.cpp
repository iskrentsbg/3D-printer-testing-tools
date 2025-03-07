#include "ntc.h"

float GetTemperature()
{
    double Vout, Rt = 0;
    float T, Tc;

    Vout = analogRead(THERMISTORPIN) * VS / ADC_MAX;
    Rt = R1 * Vout / (VS - Vout);
    T = 1 / (1 / TO + log(Rt / RO) / BETA_VALUE); // Temperature in Kelvin
    Tc = T - 273.15f;                        // Celsius
    return Tc;
}
