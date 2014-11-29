#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f4xx_hal.h"

struct rx_s {
    unsigned char header[3];
    unsigned char cmd;
};

struct hal_s {
    unsigned char lp_accel_mode;
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
    unsigned char motion_int_mode;
    unsigned long no_dmp_hz;
    unsigned long next_pedo_ms;
    unsigned long next_temp_ms;
    unsigned long next_compass_ms;
		unsigned long next_data_tx_ms;
    unsigned int report;
    unsigned short dmp_features;
    struct rx_s rx;
};

extern I2C_HandleTypeDef hi2c1;
extern struct hal_s hal;

#endif /* __MAIN_H */
