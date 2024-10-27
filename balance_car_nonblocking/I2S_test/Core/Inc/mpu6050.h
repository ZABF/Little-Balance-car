/*
 * mpu6050_SL.h
 *
 *  Created on: Feb 13, 2022
 *      Author: Huffer
 */

#ifndef MPU6050_MOTION_DRIVER_PORTING_MPU6050_SL_H_
#define MPU6050_MOTION_DRIVER_PORTING_MPU6050_SL_H_

#include "main.h"

#define USE_PRINTF_DEBUG 1
#define MPU_SELF_TEST 0
#define MOTION (0)
#define NO_MOTION (1)

#define ACCEL_ON (0x01)
#define GYRO_ON (0x02)
#define COMPASS_ON (0x04)

/* Starting sampling rate. */
#define DEFAULT_MPU_HZ (20)

#define FLASH_SIZE (512)
#define FLASH_MEM_START ((void *)0x1800)

#define PEDO_READ_MS (1000)
#define TEMP_READ_MS (500)
#define COMPASS_READ_MS (100)

/* Data read from MPL. */
#define PRINT_ACCEL     (0x01) 
#define PRINT_GYRO      (0x02) 
#define PRINT_QUAT      (0x04) 
#define PRINT_COMPASS 	(0x08) 
#define PRINT_EULER     (0x10) 
#define PRINT_ROT_MAT   (0x20) 
#define PRINT_HEADING   (0x40)
#define PRINT_PEDO      (0x80)
#define PRINT_LINEAR_ACCEL (0x100)
#define PRINT_GRAVITY_VECTOR (0x200)

/* Switch */
#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)

/* Starting sampling rate. */
#define DEFAULT_mpu_HZ  (1000)
#define TEMP_READ_TICK    (500)

#define FUN_1

#if defined FUN_1
struct rx_s
{
    unsigned char header[3];
    unsigned char cmd;
};
struct hal_s
{
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
    unsigned int report;
    unsigned short dmp_features;
    struct rx_s rx;
};

struct platform_data_s
{
    signed char orientation[9];
};

extern struct hal_s hal;
extern struct platform_data_s gyro_pdata;




uint8_t MPU6050_mpu_init(void);
uint8_t MPU6050_mpl_init(void);
uint8_t MPU6050_config(void);
void MPU6050_data_ready_cb(void);

void run_self_test(void);
void setup_gyro(void);
#endif

#ifdef COMPASS_ENABLED
void send_status_compass();
#endif


#endif /* MPU6050_MOTION_DRIVER_PORTING_MPU6050_SL_H_ */