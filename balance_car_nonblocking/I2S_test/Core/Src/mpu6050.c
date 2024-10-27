/*
 * mpu6050_SL.c
 *
 *  Created on: Feb 13, 2022
 *      Author: Huffer
 */

#include "mpu6050.h"
#include "main.h"

#include "eMPL_outputs.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "log.h"
#include "mltypes.h"
#include "mpl.h"
#include "mpu.h"
#include "packet.h"

#include "stdio.h"

unsigned char *mpl_key = (unsigned char *)"eMPL 5.1";
struct hal_s hal = {0};

struct platform_data_s gyro_pdata = {.orientation = {1, 0, 0, 0, 1, 0, 0, 0, 1}};

#if defined MPU9150 || defined MPU9250 //三轴磁场
static struct platform_data_s compass_pdata = {.orientation = {0, 1, 0, 1, 0, 0, 0, 0, -1}};
#define COMPASS_ENABLED 1
#elif defined AK8975_SECONDARY
static struct platform_data_s compass_pdata = {.orientation = {-1, 0, 0, 0, 1, 0, 0, 0, -1}};
#define COMPASS_ENABLED 1
#elif defined AK8963_SECONDARY
static struct platform_data_s compass_pdata = {.orientation = {-1, 0, 0, 0, -1, 0, 0, 0, 1}};
#define COMPASS_ENABLED 1
#endif

#if defined FUN_1
uint8_t MPU6050_mpu_init(void)
{
    inv_error_t result;
    struct int_param_s int_param;
    result = mpu_init(&int_param);
    if (result)
    {
#if USE_PRINTF_DEBUG
        printf("Could not initialize gyro.result =  %d\n", result);
#endif
        return 0;
    }
    else
    {
#if USE_PRINTF_DEBUG
        printf("Initialize gyro.result =  %d\n succeeded\n", result);
#endif
        return 1;
    }
}

uint8_t MPU6050_mpl_init(void)
{
    inv_error_t result;
    result = inv_init_mpl();
    if (result)
    {
#if USE_PRINTF_DEBUG
        printf("Could not initialize MPL.\n");
#endif
        return 0;
    }
    else
    {
#if USE_PRINTF_DEBUG
        printf("initialize MPL suceeded.\n");
#endif
        return 1;
    }
}

uint8_t MPU6050_config(void)//这三个官方给的例子仿写
{
    inv_error_t result;
    unsigned char accel_fsr;
    unsigned short gyro_rate, gyro_fsr;

#ifdef COMPASS_ENABLED
    unsigned char new_compass = 0;
    unsigned short compass_fsr;
#endif
    /* Compute 6-axis and 9-axis quaternions. */
    inv_enable_quaternion();
    inv_enable_9x_sensor_fusion();

    /* The MPL expects compass data at a constant rate (matching the rate
     * passed to inv_set_compass_sample_rate). If this is an issue for your
     * application, call this function, and the MPL will depend on the
     * timestamps passed to inv_build_compass instead.
     *
     * inv_9x_fusion_use_timestamps(1);
     */

    /* This function has been deprecated.
     * inv_enable_no_gyro_fusion();
     */

    /* Update gyro biases when not in motion.
     * WARNING: These algorithms are mutually exclusive.
     */
    inv_enable_fast_nomot();
    /* inv_enable_motion_no_motion(); */
    /* inv_set_no_motion_time(1000); */

    /* Update gyro biases when temperature changes. */
    inv_enable_gyro_tc();

    /* This algorithm updates the accel biases when in motion. A more accurate
     * bias measurement can be made when running the self-test (see case 't' in
     * handle_input), but this algorithm can be enabled if the self-test can't
     * be executed in your application.
     *
     * inv_enable_in_use_auto_calibration();
     */
#ifdef COMPASS_ENABLED
    /* Compass calibration algorithms. */
    inv_enable_vector_compass_cal();
    inv_enable_magnetic_disturbance();
#endif
    /* If you need to estimate your heading before the compass is calibrated,
     * enable this algorithm. It becomes useless after a good figure-eight is
     * detected, so we'll just leave it out to save memory.
     * inv_enable_heading_from_gyro();
     */

    /* Allows use of the MPL APIs in read_from_mpl. */
    inv_enable_eMPL_outputs();//fff
//////////////////		
    result = inv_enable_hal_outputs();//fff
//    if (result)
//    {
        // MPL_LOGE("Could not start the MPL.\n");
 //       printf("inv_enable_hal_outputs()\n");
 //   }
    result = inv_start_mpl();//fff
    if (result == INV_ERROR_NOT_AUTHORIZED)
    {
        while (1)
        {
            MPL_LOGE("Not authorized.\n");
            printf("Not authorized.\n");
        }
    }
    if (result)
    {
        MPL_LOGE("Could not start the MPL.\n");
        printf("Could not start the MPL.\n");
    }else printf("MPL start successful.\n");

    /* Get/set hardware configuration. Start gyro. */
    /* Wake up all sensors. */
#ifdef COMPASS_ENABLED
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
#else
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
#endif
    /* Push both gyro and accel data into the FIFO. */
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_set_sample_rate(DEFAULT_MPU_HZ);
#ifdef COMPASS_ENABLED
    /* The compass sampling rate can be less than the gyro/accel sampling rate.
     * Use this function for proper power management.
     */
    mpu_set_compass_sample_rate(1000 / COMPASS_READ_MS);
#endif
    /* Read back configuration in case it was set improperly. */
    mpu_get_sample_rate(&gyro_rate);
    mpu_get_gyro_fsr(&gyro_fsr);
    mpu_get_accel_fsr(&accel_fsr);
#ifdef COMPASS_ENABLED
    mpu_get_compass_fsr(&compass_fsr);
#endif
    /* Sync driver configuration with MPL. */
    /* Sample rate expected in microseconds. */
    inv_set_gyro_sample_rate(1000000L / gyro_rate);
    inv_set_accel_sample_rate(1000000L / gyro_rate);
#ifdef COMPASS_ENABLED
    /* The compass rate is independent of the gyro and accel rates. As long as
     * inv_set_compass_sample_rate is called with the correct value, the 9-axis
     * fusion algorithm's compass correction gain will work properly.
     */
    inv_set_compass_sample_rate(COMPASS_READ_MS * 1000L);
#endif
    /* Set chip-to-body orientation matrix.
     * Set hardware units to dps/g's/degrees scaling factor.
     */
    inv_set_gyro_orientation_and_scale(inv_orientation_matrix_to_scalar(gyro_pdata.orientation), (long)gyro_fsr << 15);
    inv_set_accel_orientation_and_scale(inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
                                        (long)accel_fsr << 15);
#ifdef COMPASS_ENABLED
    inv_set_compass_orientation_and_scale(inv_orientation_matrix_to_scalar(compass_pdata.orientation),
                                          (long)compass_fsr << 15);
#endif
    /* Initialize HAL state variables. */
#ifdef COMPASS_ENABLED
    hal.sensors = ACCEL_ON | GYRO_ON | COMPASS_ON;
#else
    hal.sensors = ACCEL_ON | GYRO_ON;
#endif
    hal.dmp_on = 0;
    hal.report = 0xfff;
    hal.rx.cmd = 0;
    hal.next_pedo_ms = 0;
    hal.next_compass_ms = 0;
    hal.next_temp_ms = 0;

    /* Compass reads are handled by scheduler. */

    /* To initialize the DMP:
     * 1. Call dmp_load_motion_driver_firmware(). This pushes the DMP image in
     *    inv_mpu_dmp_motion_driver.h into the MPU memory.
     * 2. Push the gyro and accel orientation matrix to the DMP.
     * 3. Register gesture callbacks. Don't worry, these callbacks won't be
     *    executed unless the corresponding feature is enabled.
     * 4. Call dmp_enable_feature(mask) to enable different features.
     * 5. Call dmp_set_fifo_rate(freq) to select a DMP output rate.
     * 6. Call any feature-specific control functions.
     *
     * To enable the DMP, just call mpu_set_dmp_state(1). This function can
     * be called repeatedly to enable and disable the DMP at runtime.
     *
     * The following is a short summary of the features supported in the DMP
     * image provided in inv_mpu_dmp_motion_driver.c:
     * DMP_FEATURE_LP_QUAT: Generate a gyro-only quaternion on the DMP at
     * 200Hz. Integrating the gyro data at higher rates reduces numerical
     * errors (compared to integration on the MCU at a lower sampling rate).
     * DMP_FEATURE_6X_LP_QUAT: Generate a gyro/accel quaternion on the DMP at
     * 200Hz. Cannot be used in combination with DMP_FEATURE_LP_QUAT.
     * DMP_FEATURE_TAP: Detect taps along the X, Y, and Z axes.
     * DMP_FEATURE_ANDROID_ORIENT: Google's screen rotation algorithm. Triggers
     * an event at the four orientations where the screen should rotate.
     * DMP_FEATURE_GYRO_CAL: Calibrates the gyro data after eight seconds of
     * no motion.
     * DMP_FEATURE_SEND_RAW_ACCEL: Add raw accelerometer data to the FIFO.
     * DMP_FEATURE_SEND_RAW_GYRO: Add raw gyro data to the FIFO.
     * DMP_FEATURE_SEND_CAL_GYRO: Add calibrated gyro data to the FIFO. Cannot
     * be used in combination with DMP_FEATURE_SEND_RAW_GYRO.
     */

    dmp_load_motion_driver_firmware();
    dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_pdata.orientation));

    //    dmp_register_tap_cb(tap_cb);

    // dmp_register_android_orient_cb(android_orient_cb);
    /*
     * Known Bug -
     * DMP when enabled will sample sensor data at 200Hz and output to FIFO at the rate
     * specified in the dmp_set_fifo_rate API. The DMP will then sent an interrupt once
     * a sample has been put into the FIFO. Therefore if the dmp_set_fifo_rate is at 25Hz
     * there will be a 25Hz interrupt from the MPU device.
     *
     * There is a known issue in which if you do not enable DMP_FEATURE_TAP
     * then the interrupts will be at 200Hz even if fifo rate
     * is set at a different rate. To avoid this issue include the DMP_FEATURE_TAP
     *
     * DMP sensor fusion works only with gyro at +-2000dps and accel +-2G
     */
    hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP | DMP_FEATURE_ANDROID_ORIENT |
                       DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL;
    dmp_enable_feature(hal.dmp_features);
    dmp_set_fifo_rate(DEFAULT_MPU_HZ);
    mpu_set_dmp_state(1);
    hal.dmp_on = 1;
    return 1;
}

void MPU6050_data_ready_cb(void) //中断回调函数
{
		static unsigned char new_temp = 0;
    static unsigned long timestamp;
    hal.new_gyro = 1;
	
#if 1
	if(FLAG_MAIN_INITIAL){
		while(hal.new_gyro ==1){
			if(FLAG_MAIN_INITIAL){
				unsigned long sensor_timestamp;
				int new_data = 0;
				
				get_tick_count(&timestamp);
				if (timestamp > hal.next_temp_ms){
						hal.next_temp_ms = timestamp + TEMP_READ_MS;//temperture read every 500ms
						new_temp = 1;
				}

				if (hal.motion_int_mode){
					/* Enable motion interrupt. */
					mpu_lp_motion_interrupt(500, 1, 5);
					/* Notify the MPL that contiguity was broken. */
					inv_accel_was_turned_off();
					inv_gyro_was_turned_off();
					inv_compass_was_turned_off();
					inv_quaternion_sensor_was_turned_off();
					/* Restore the previous sensor configuration. */
					mpu_lp_motion_interrupt(0, 0, 0);
					hal.motion_int_mode = 0;
				}

				if (hal.new_gyro && hal.dmp_on){
					short gyro[3], accel_short[3], sensors_short;
					unsigned char more;
					long accel[3], quat[4], temperature;
				
					dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors_short, &more);
					if (!more) hal.new_gyro = 0;
					if (sensors_short & INV_WXYZ_QUAT){
						inv_build_quat(quat, 0, sensor_timestamp);
						new_data = 1;
					}
					if (sensors_short & INV_XYZ_GYRO){
						/* Push the new data to the MPL. */
						inv_build_gyro(gyro, sensor_timestamp);
						new_data = 1;
						if (new_temp)
						{
								new_temp = 0;
								/* Temperature only used for gyro temp comp. */
								mpu_get_temperature(&temperature, &sensor_timestamp);
								inv_build_temp(temperature, sensor_timestamp);
						}
					}
					if (sensors_short & INV_XYZ_ACCEL){
						accel[0] = (long)accel_short[0];
						accel[1] = (long)accel_short[1];
						accel[2] = (long)accel_short[2];
						inv_build_accel(accel, 0, sensor_timestamp);
						new_data = 1;
					}	
				}
				
				if (new_data){
					inv_execute_on_data();
#if 0
				send_to_pc();
				
#elif 0
				read_from_mpl();
#endif
				output(); 
				}	
			}
		}
	}
#endif		
}

/* Handle sensor on/off combinations. */
void setup_gyro(void)
{
    unsigned char mask = 0, lp_accel_was_on = 0;

    // MPU_DEBUG_FUNC();
    if (hal.sensors & ACCEL_ON)
        mask |= INV_XYZ_ACCEL;
    if (hal.sensors & GYRO_ON)
    {
        mask |= INV_XYZ_GYRO;
        lp_accel_was_on |= hal.lp_accel_mode;
    }
#ifdef COMPASS_ENABLED
    if (hal.sensors & COMPASS_ON)
    {
        mask |= INV_XYZ_COMPASS;
        lp_accel_was_on |= hal.lp_accel_mode;
    }
#endif
    /* If you need a power transition, this function should be called with a
     * mask of the sensors still enabled. The driver turns off any sensors
     * excluded from this mask.
     */
    mpu_set_sensors(mask);
    mpu_configure_fifo(mask);
    if (lp_accel_was_on)
    {
        unsigned short rate;
        hal.lp_accel_mode = 0;
        /* Switching out of LP accel, notify MPL of new accel sampling rate. */
        mpu_get_sample_rate(&rate);
        inv_set_accel_sample_rate(1000000L / rate);
    }
}
uint8_t MPU6050_get_sensor(float *Acceleration, float *Angular_velocity, float *Quaternion, float *Euler)
{
    uint8_t return_value = 0;
    short gyro[3], accel_short[3], sensors;
    unsigned char more, new_data = 0;
    long accel[3], quat[4], temperature;
    unsigned long sensor_timestamp;
    dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors, &more);//fff
    if (!more)
        hal.new_gyro = 0;

    if (sensors & INV_XYZ_GYRO)
    {
        /* Push the new data to the MPL. */
        inv_build_gyro(gyro, sensor_timestamp);
        mpu_get_temperature(&temperature, &sensor_timestamp);
        inv_build_temp(temperature, sensor_timestamp);
        new_data = 1;
    }
    if (sensors & INV_XYZ_ACCEL)
    {
        accel[0] = (long)accel_short[0];
        accel[1] = (long)accel_short[1];
        accel[2] = (long)accel_short[2];
        inv_build_accel(accel, 0, sensor_timestamp);
        new_data = 1;
    }
    if (sensors & INV_WXYZ_QUAT)
    {
        inv_build_quat(quat, 0, sensor_timestamp);
        new_data = 1;
    }

    inv_execute_on_data();
    return return_value;
}
#ifdef COMPASS_ENABLED
void send_status_compass()
{
    long data[3] = {0};
    int8_t accuracy = {0};
    unsigned long timestamp;
    inv_get_compass_set(data, &accuracy, (inv_time_t *)&timestamp);
    MPL_LOGI("Compass: %7.4f %7.4f %7.4f ", data[0] / 65536.f, data[1] / 65536.f, data[2] / 65536.f);
    MPL_LOGI("Accuracy= %d\r\n", accuracy);
}
#endif

#if MPU_SELF_TEST
void run_self_test(void)
{
    int result;
    long gyro[3], accel[3];
    // MPU_DEBUG_FUNC();
#if defined(MPU6500) || defined(MPU9250)
    result = mpu_run_6500_self_test(gyro, accel, 0);
#elif defined(MPU6050) || defined(MPU9150)
    result = mpu_run_self_test(gyro, accel);
#endif
    if (result == 0x7)
    {
        MPL_LOGI("Passed!\n");
        MPL_LOGI("accel: %7.4f %7.4f %7.4f\n", accel[0] / 65536.f, accel[1] / 65536.f, accel[2] / 65536.f);
        MPL_LOGI("gyro: %7.4f %7.4f %7.4f\n", gyro[0] / 65536.f, gyro[1] / 65536.f, gyro[2] / 65536.f);
        /* Test passed. We can trust the gyro data here, so now we need to update calibrated data*/

#ifdef USE_CAL_HW_REGISTERS
        /*
         * This portion of the code uses the HW offset registers that are in the MPUxxxx devices
         * instead of pushing the cal data to the MPL software library
         */
        unsigned char i = 0;

        for (i = 0; i < 3; i++)
        {
            gyro[i] = (long)(gyro[i] * 32.8f); // convert to +-1000dps
            accel[i] *= 2048.f;                // convert to +-16G
            accel[i] = accel[i] >> 16;
            gyro[i] = (long)(gyro[i] >> 16);
        }

        mpu_set_gyro_bias_reg(gyro);

#if defined(MPU6500) || defined(MPU9250)
        mpu_set_accel_bias_6500_reg(accel);
#elif defined(MPU6050) || defined(MPU9150)
        mpu_set_accel_bias_6050_reg(accel);
#endif
#else
        /* Push the calibrated data to the MPL library.
         *
         * MPL expects biases in hardware units << 16, but self test returns
         * biases in g's << 16.
         */
        unsigned short accel_sens;
        float gyro_sens;

        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        inv_set_accel_bias(accel, 3);
        mpu_get_gyro_sens(&gyro_sens);
        gyro[0] = (long)(gyro[0] * gyro_sens);
        gyro[1] = (long)(gyro[1] * gyro_sens);
        gyro[2] = (long)(gyro[2] * gyro_sens);
        inv_set_gyro_bias(gyro, 3);
#endif
    }
    else
    {
        if (!(result & 0x1))
            MPL_LOGE("Gyro failed.\n");
        if (!(result & 0x2))
            MPL_LOGE("Accel failed.\n");
        if (!(result & 0x4))
            MPL_LOGE("Compass failed.\n");
    }
}
#endif

#endif



