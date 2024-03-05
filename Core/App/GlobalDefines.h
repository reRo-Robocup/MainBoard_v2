/*
 *  GlobalDefines.h
 *
 *  Created on: Dec 7, 2023
 *
 *  Author: onlydcx, G4T1PR0
 */

#ifndef __APP_GLOBAL_DEFINES_H__
#define __APP_GLOBAL_DEFINES_H__

#include <stdio.h>
#include "math.h"

#define CONTROLL_CYCLE 0.001  // 1KHz

float map (float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#define deg_to_rad (M_PI / 180)
#define rad_to_deg (180 / M_PI)

#define BATT_MIN_V 10.8
#define BATT_MAX_V 12.6

#endif /* __APP_GLOBAL_DEFINES_H__ */