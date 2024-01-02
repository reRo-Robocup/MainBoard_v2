#ifndef __APP_GLOBAL_DEFINES_H__
#define __APP_GLOBAL_DEFINES_H__

#define MAIN_CONTROLL_CYCLE 0.001   // 1KHz
#define IMU_CONTROLL_CYCLE 0.000001 // 100KHZ

#define deg_to_rad(deg) (((deg) / 360) * 2 * M_PI)
#define rad_to_deg(rad) (((rad) / 2 / M_PI) * 360)

#define DEBUG

#endif