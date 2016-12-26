#ifndef PHYSICS_H
#define PHYSICS_H

#include <stdint.h>

#define COARSE(x) ((x)*16)
#define REV_COARSE(x) ((x)/16)

float PHY_batteryBeforeLoadVoltage();
float PHY_batteryAfterGPSVoltage();
float PHY_batteryAfterHFVoltage();
float PHY_solarVoltage();
float PHY_internalTemperature();

#endif
