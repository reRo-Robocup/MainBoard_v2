/*
 *  GlobalDefines.h
 *
 *  Created on: Dec 7, 2023
 * 
 *  Author: onlydcx, G4T1PR0
 */

#ifndef __APP_GLOBAL_DEFINES_H__
#define __APP_GLOBAL_DEFINES_H__

#include "math.h"
#include <stdio.h>

#define CONTROLL_CYCLE 0.001   // 1KHz

#define deg_to_rad(deg) (((deg) / 360) * 2 * M_PI)
#define rad_to_deg(rad) (((rad) / 2 / M_PI) * 360)

#define map2_180(angle) while(angle >  180) angle -= 180;\
    while(angle < -180) angle += 180;

#define BATT_MIN_V 10.8
#define BATT_MAX_V 12.6

#define DEBUG

#endif /* __APP_GLOBAL_DEFINES_H__ */