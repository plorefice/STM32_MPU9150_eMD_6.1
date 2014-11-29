/**
  ******************************************************************************
  * File Name          : main.c
  * Date               : 26/10/2014 15:33:52
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2014 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "eMPL_outputs.h"
#include "mltypes.h"
#include "mpu.h"
#include "log.h"
#include "packet.h"

#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)
#define PRINT_COMPASS   (0x08)
#define PRINT_EULER     (0x10)
#define PRINT_ROT_MAT   (0x20)
#define PRINT_HEADING   (0x40)
#define PRINT_PEDO      (0x80)
#define PRINT_LINEAR_ACCEL (0x100)

volatile uint32_t hal_timestamp = 0;
#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
#define COMPASS_ON      (0x04)

#define MOTION          (0)
#define NO_MOTION       (1)

#define DEFAULT_MPU_HZ  (200)

#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)

#define PEDO_READ_MS    (1000)
#define TEMP_READ_MS    (500)
#define DATA_TX_MS      (20)
#define COMPASS_READ_MS (5)

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

struct hal_s hal = {0};

volatile unsigned char rx_new;

unsigned char *mpl_key = (unsigned char*)"eMPL 5.1";

struct platform_data_s {
	signed char orientation[9];
};

static struct platform_data_s gyro_pdata = {
    .orientation = {  1, 0, 0,
                      0, 1, 0,
                      0, 0, 1}
};

#if defined MPU9150 || defined MPU9250
static struct platform_data_s compass_pdata = {
    .orientation = { 0, 1, 0,
                     1, 0, 0,
                     0, 0, -1}
};
#define COMPASS_ENABLED 1
#elif defined AK8975_SECONDARY
static struct platform_data_s compass_pdata = {
    .orientation = {-1, 0, 0,
                     0, 1, 0,
                     0, 0,-1}
};
#define COMPASS_ENABLED 1
#elif defined AK8963_SECONDARY
static struct platform_data_s compass_pdata = {
    .orientation = {-1, 0, 0,
                     0,-1, 0,
                     0, 0, 1}
};
#define COMPASS_ENABLED 1
#endif

enum packet_type_e {
    PACKET_TYPE_ACCEL,
    PACKET_TYPE_GYRO,
    PACKET_TYPE_QUAT,
    PACKET_TYPE_TAP,
    PACKET_TYPE_ANDROID_ORIENT,
    PACKET_TYPE_PEDO,
    PACKET_TYPE_MISC,
    PACKET_TYPE_COMPASS,
};

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void Board_Init (void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_IRQ_Init (void);
static void Error_Handler (void);


static void read_from_mpl(void)
{
	long msg, data[9];
	int8_t accuracy;
	unsigned long timestamp;
	float float_data[3] = {0};

	if (inv_get_sensor_type_quat(data, &accuracy, (inv_time_t*)&timestamp))
	{
	 /* Sends a quaternion packet to the PC. Since this is used by the Python
		* test app to visually represent a 3D quaternion, it's sent each time
		* the MPL has new data.
		*/
		eMPL_send_quat(data);

		/* Specific data packets can be sent or suppressed using USB commands. */
		if (hal.report & PRINT_QUAT)
			eMPL_send_data(PACKET_DATA_QUAT, data);
	}

	if (hal.report & PRINT_ACCEL)
	{
		if (inv_get_sensor_type_accel(data, &accuracy, (inv_time_t*)&timestamp))
			eMPL_send_data(PACKET_DATA_ACCEL, data);
	}
	if (hal.report & PRINT_GYRO)
	{
		if (inv_get_sensor_type_gyro(data, &accuracy, (inv_time_t*)&timestamp))
			eMPL_send_data(PACKET_DATA_GYRO, data);
	}
#ifdef COMPASS_ENABLED
	if (hal.report & PRINT_COMPASS)
	{
		if (inv_get_sensor_type_compass(data, &accuracy, (inv_time_t*)&timestamp))
			eMPL_send_data(PACKET_DATA_COMPASS, data);
	}
#endif
	if (hal.report & PRINT_EULER)
	{
		if (inv_get_sensor_type_euler(data, &accuracy, (inv_time_t*)&timestamp))
			eMPL_send_data(PACKET_DATA_EULER, data);
	}
	if (hal.report & PRINT_ROT_MAT)
	{
		if (inv_get_sensor_type_rot_mat(data, &accuracy, (inv_time_t*)&timestamp))
			eMPL_send_data(PACKET_DATA_ROT, data);
	}
	if (hal.report & PRINT_HEADING)
	{
		if (inv_get_sensor_type_heading(data, &accuracy, (inv_time_t*)&timestamp))
			eMPL_send_data(PACKET_DATA_HEADING, data);
	}
	if (hal.report & PRINT_LINEAR_ACCEL)
	{
		if (inv_get_sensor_type_linear_acceleration(float_data, &accuracy, (inv_time_t*)&timestamp))
		{
			MPL_LOGI("Linear Accel: %7.5f %7.5f %7.5f\r\n",	float_data[0], float_data[1], float_data[2]);                                        
		}
	}
	if (hal.report & PRINT_PEDO)
	{
		unsigned long timestamp;
		timestamp = HAL_GetTick();
		
		if (timestamp > hal.next_pedo_ms)
		{
			hal.next_pedo_ms = timestamp + PEDO_READ_MS;
			unsigned long step_count, walk_time;
			dmp_get_pedometer_step_count(&step_count);
			dmp_get_pedometer_walk_time(&walk_time);
			MPL_LOGI("Walked %ld steps over %ld milliseconds..\n", step_count, walk_time);
		}
	}

	/* Whenever the MPL detects a change in motion state, the application can
	 * be notified. For this example, we use an LED to represent the current
	 * motion state.
	 */
	msg = inv_get_message_level_0(INV_MSG_MOTION_EVENT | INV_MSG_NO_MOTION_EVENT);
	if (msg)
	{
		if (msg & INV_MSG_MOTION_EVENT)
		{
			MPL_LOGI("Motion!\n");
		}
		else if (msg & INV_MSG_NO_MOTION_EVENT)
		{
			MPL_LOGI("No motion!\n");
		}
	}
}


#ifdef COMPASS_ENABLED
void send_status_compass()
{
	long data[3] = { 0 };
	int8_t accuracy = { 0 };
	unsigned long timestamp;
	
	inv_get_compass_set(data, &accuracy, (inv_time_t*) &timestamp);
	MPL_LOGI("Compass: %7.4f %7.4f %7.4f ", data[0]/65536.f, data[1]/65536.f, data[2]/65536.f);
	MPL_LOGI("Accuracy= %d\r\n", accuracy);
}
#endif


/* Handle sensor on/off combinations. */
static void setup_gyro(void)
{
	unsigned char mask = 0, lp_accel_was_on = 0;
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


static inline void run_self_test(void)
{
	int result;
	long gyro[3], accel[3];

#if defined (MPU6500) || defined (MPU9250)
	result = mpu_run_6500_self_test(gyro, accel, 0);
#elif defined (MPU6050) || defined (MPU9150)
	result = mpu_run_self_test(gyro, accel);
#endif
	if (result == 0x7)
	{
		MPL_LOGI("Passed!\n");
		MPL_LOGI("accel: %7.4f %7.4f %7.4f\n",
									accel[0]/65536.f,
									accel[1]/65536.f,
									accel[2]/65536.f);
		MPL_LOGI("gyro: %7.4f %7.4f %7.4f\n",
									gyro[0]/65536.f,
									gyro[1]/65536.f,
									gyro[2]/65536.f);
		/* Test passed. We can trust the gyro data here, so now we need to update calibrated data*/

#ifdef USE_CAL_HW_REGISTERS
		/*
		 * This portion of the code uses the HW offset registers that are in the MPUxxxx devices
		 * instead of pushing the cal data to the MPL software library
		 */
		unsigned char i = 0;

		for(i = 0; i<3; i++)
		{
			gyro[i] = (long)(gyro[i] * 32.8f); //convert to +-1000dps
			accel[i] *= 4096.f; //convert to +-8G
			accel[i] = accel[i] >> 16;
			gyro[i] = (long)(gyro[i] >> 16);
		}

		mpu_set_gyro_bias_reg(gyro);

	#if defined (MPU6500) || defined (MPU9250)
		mpu_set_accel_bias_6500_reg(accel);
	#elif defined (MPU6050) || defined (MPU9150)
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
		gyro[0] = (long) (gyro[0] * gyro_sens);
		gyro[1] = (long) (gyro[1] * gyro_sens);
		gyro[2] = (long) (gyro[2] * gyro_sens);
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


static void handle_input(void)
{
	char c = 0;

	switch (c) {
	/* These commands turn off individual sensors. */
	case '8':
		hal.sensors ^= ACCEL_ON;
		setup_gyro();
		if (!(hal.sensors & ACCEL_ON))
			inv_accel_was_turned_off();
		break;
	case '9':
		hal.sensors ^= GYRO_ON;
		setup_gyro();
		if (!(hal.sensors & GYRO_ON))
			inv_gyro_was_turned_off();
		break;
#ifdef COMPASS_ENABLED
	case '0':
		hal.sensors ^= COMPASS_ON;
		setup_gyro();
		if (!(hal.sensors & COMPASS_ON))
			inv_compass_was_turned_off();
		break;
#endif
	/* The commands send individual sensor data or fused data to the PC. */
	case 'a':
		hal.report ^= PRINT_ACCEL;
		break;
	case 'g':
		hal.report ^= PRINT_GYRO;
		break;
#ifdef COMPASS_ENABLED
	case 'c':
		hal.report ^= PRINT_COMPASS;
		break;
#endif
	case 'e':
		hal.report ^= PRINT_EULER;
		break;
	case 'r':
		hal.report ^= PRINT_ROT_MAT;
		break;
	case 'q':
		hal.report ^= PRINT_QUAT;
		break;
	case 'h':
		hal.report ^= PRINT_HEADING;
		break;
	case 'i':
		hal.report ^= PRINT_LINEAR_ACCEL;
		break;
#ifdef COMPASS_ENABLED
	case 'w':
		send_status_compass();
		break;
#endif
	/* This command prints out the value of each gyro register for debugging.
	 * If logging is disabled, this function has no effect.
	 */
	case 'd':
		mpu_reg_dump();
		break;
	/* Test out low-power accel mode. */
	case 'p':
		if (hal.dmp_on)
			/* LP accel is not compatible with the DMP. */
			break;
		mpu_lp_accel_mode(20);
		/* When LP accel mode is enabled, the driver automatically configures
		 * the hardware for latched interrupts. However, the MCU sometimes
		 * misses the rising/falling edge, and the hal.new_gyro flag is never
		 * set. To avoid getting locked in this state, we're overriding the
		 * driver's configuration and sticking to unlatched interrupt mode.
		 *
		 * TODO: The MCU supports level-triggered interrupts.
		 */
		mpu_set_int_latched(0);
		hal.sensors &= ~(GYRO_ON|COMPASS_ON);
		hal.sensors |= ACCEL_ON;
		hal.lp_accel_mode = 1;
		inv_gyro_was_turned_off();
		inv_compass_was_turned_off();
		break;
	/* The hardware self test can be run without any interaction with the
	 * MPL since it's completely localized in the gyro driver. Logging is
	 * assumed to be enabled; otherwise, a couple LEDs could probably be used
	 * here to display the test results.
	 */
	case 't':
		run_self_test();
		/* Let MPL know that contiguity was broken. */
		inv_accel_was_turned_off();
		inv_gyro_was_turned_off();
		inv_compass_was_turned_off();
		break;
	/* Depending on your application, sensor data may be needed at a faster or
	 * slower rate. These commands can speed up or slow down the rate at which
	 * the sensor data is pushed to the MPL.
	 *
	 * In this example, the compass rate is never changed.
	 */
	case '1':
		if (hal.dmp_on)
		{
			dmp_set_fifo_rate(10);
			inv_set_quat_sample_rate(100000L);
		}
		else
			mpu_set_sample_rate(10);
		inv_set_gyro_sample_rate(100000L);
		inv_set_accel_sample_rate(100000L);
		break;
	case '2':
		if (hal.dmp_on)
		{
			dmp_set_fifo_rate(20);
			inv_set_quat_sample_rate(50000L);
		}
		else
				mpu_set_sample_rate(20);
		inv_set_gyro_sample_rate(50000L);
		inv_set_accel_sample_rate(50000L);
		break;
	case '3':
		if (hal.dmp_on)
		{
			dmp_set_fifo_rate(40);
			inv_set_quat_sample_rate(25000L);
		}
		else
			mpu_set_sample_rate(40);
		inv_set_gyro_sample_rate(25000L);
		inv_set_accel_sample_rate(25000L);
		break;
	case '4':
		if (hal.dmp_on)
		{
			dmp_set_fifo_rate(50);
			inv_set_quat_sample_rate(20000L);
		}
		else
			mpu_set_sample_rate(50);
		inv_set_gyro_sample_rate(20000L);
		inv_set_accel_sample_rate(20000L);
		break;
	case '5':
		if (hal.dmp_on)
		{
			dmp_set_fifo_rate(100);
			inv_set_quat_sample_rate(10000L);
		}
		else
				mpu_set_sample_rate(100);
		inv_set_gyro_sample_rate(10000L);
		inv_set_accel_sample_rate(10000L);
		break;
	case ',':
		/* Set hardware to interrupt on gesture event only. This feature is
		 * useful for keeping the MCU asleep until the DMP detects as a tap or
		 * orientation event.
		 */
		dmp_set_interrupt_mode(DMP_INT_GESTURE);
		break;
	case '.':
		/* Set hardware to interrupt periodically. */
		dmp_set_interrupt_mode(DMP_INT_CONTINUOUS);
		break;
	case '6':
		/* Toggle pedometer display. */
		hal.report ^= PRINT_PEDO;
		break;
	case '7':
		/* Reset pedometer. */
		dmp_set_pedometer_step_count(0);
		dmp_set_pedometer_walk_time(0);
		break;
	case 'f':
		if (hal.lp_accel_mode)
			/* LP accel is not compatible with the DMP. */
			return;
		/* Toggle DMP. */
		if (hal.dmp_on)
		{
			unsigned short dmp_rate;
			unsigned char mask = 0;
			hal.dmp_on = 0;
			mpu_set_dmp_state(0);
			/* Restore FIFO settings. */
			if (hal.sensors & ACCEL_ON)
				mask |= INV_XYZ_ACCEL;
			if (hal.sensors & GYRO_ON)
				mask |= INV_XYZ_GYRO;
			if (hal.sensors & COMPASS_ON)
				mask |= INV_XYZ_COMPASS;
			mpu_configure_fifo(mask);
			/* When the DMP is used, the hardware sampling rate is fixed at
			 * 200Hz, and the DMP is configured to downsample the FIFO output
			 * using the function dmp_set_fifo_rate. However, when the DMP is
			 * turned off, the sampling rate remains at 200Hz. This could be
			 * handled in inv_mpu.c, but it would need to know that
			 * inv_mpu_dmp_motion_driver.c exists. To avoid this, we'll just
			 * put the extra logic in the application layer.
			 */
			dmp_get_fifo_rate(&dmp_rate);
			mpu_set_sample_rate(dmp_rate);
			inv_quaternion_sensor_was_turned_off();
			MPL_LOGI("DMP disabled.\n");
		}
		else
		{
			unsigned short sample_rate;
			hal.dmp_on = 1;
			/* Preserve current FIFO rate. */
			mpu_get_sample_rate(&sample_rate);
			dmp_set_fifo_rate(sample_rate);
			inv_set_quat_sample_rate(1000000L / sample_rate);
			mpu_set_dmp_state(1);
			MPL_LOGI("DMP enabled.\n");
		}
		break;
	case 'm':
			/* Test the motion interrupt hardware feature. */
#ifndef MPU6050 // not enabled for 6050 product
		hal.motion_int_mode = 1;
#endif 
		break;
	case 'v':
		/* Toggle LP quaternion.
		 * The DMP features can be enabled/disabled at runtime. Use this same
		 * approach for other features.
		 */
		hal.dmp_features ^= DMP_FEATURE_6X_LP_QUAT;
		dmp_enable_feature(hal.dmp_features);
		if (!(hal.dmp_features & DMP_FEATURE_6X_LP_QUAT))
		{
			inv_quaternion_sensor_was_turned_off();
			MPL_LOGI("LP quaternion disabled.\n");
		}
		else
			MPL_LOGI("LP quaternion enabled.\n");
		break;
	default:
		break;
	}
	hal.rx.cmd = 0;
}


void Board_Init (void)
{
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* System interrupt init*/
  /* Sets the priority grouping field */
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_I2C1_Init();
}


int main(void)
{
	inv_error_t result;
	struct int_param_s int_param;
	
	unsigned char accel_fsr, new_temp = 0, new_compass = 0;
	unsigned short gyro_rate, gyro_fsr, compass_fsr;
	unsigned long timestamp;
	
	int_param.cb = NULL;
	
	Board_Init();
	
	result = mpu_init(&int_param);
  if (result)
	{
		Error_Handler();
  }
	
	result = inv_init_mpl();
  if (result)
	{
		Error_Handler();
  }
	
	inv_enable_quaternion();
	inv_enable_9x_sensor_fusion();
  inv_enable_fast_nomot();
  inv_enable_gyro_tc();
	inv_enable_vector_compass_cal();
	inv_enable_magnetic_disturbance();
	inv_enable_eMPL_outputs();
	
	result = inv_start_mpl();
	if (result == INV_ERROR_NOT_AUTHORIZED)
	{
		Error_Handler();
	}
	if (result)
	{
		Error_Handler();
	}
	
	mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
	mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
	mpu_set_sample_rate(DEFAULT_MPU_HZ);
	mpu_set_compass_sample_rate(1000 / COMPASS_READ_MS);
	
	mpu_get_sample_rate(&gyro_rate);
  mpu_get_gyro_fsr(&gyro_fsr);
  mpu_get_accel_fsr(&accel_fsr);
  mpu_get_compass_fsr(&compass_fsr);
	
	inv_set_gyro_sample_rate(1000000L / gyro_rate);
	inv_set_accel_sample_rate(1000000L / gyro_rate);
  inv_set_compass_sample_rate(COMPASS_READ_MS * 1000L);
	
	
	inv_set_gyro_orientation_and_scale(
		  inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
			(long)gyro_fsr<<15);
	inv_set_accel_orientation_and_scale(
		  inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
			(long)accel_fsr<<15);
	inv_set_compass_orientation_and_scale(
			inv_orientation_matrix_to_scalar(compass_pdata.orientation),
			(long)compass_fsr<<15);
			
  hal.sensors = ACCEL_ON | GYRO_ON | COMPASS_ON;
	hal.dmp_on = 0;
	hal.report = 0;
	hal.next_pedo_ms = 0;
	hal.next_compass_ms = 0;
	hal.next_temp_ms = 0;
	
	timestamp = HAL_GetTick();
	
	dmp_load_motion_driver_firmware();
	dmp_set_orientation(
			inv_orientation_matrix_to_scalar(gyro_pdata.orientation));
	
	hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
											DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | 
											DMP_FEATURE_SEND_CAL_GYRO |	DMP_FEATURE_GYRO_CAL;
	dmp_enable_feature(hal.dmp_features);
	dmp_set_fifo_rate(DEFAULT_MPU_HZ);
	mpu_set_dmp_state(1);
	
	hal.dmp_on = 1;
	
	MX_IRQ_Init();
	
  /* Infinite loop */
  while (1)
  {
    unsigned long sensor_timestamp;
    int new_data = 0;
		
		timestamp = HAL_GetTick();
		
		if ((timestamp > hal.next_compass_ms) && !hal.lp_accel_mode && hal.new_gyro && (hal.sensors & COMPASS_ON))
		{
			hal.next_compass_ms = timestamp + COMPASS_READ_MS;
			new_compass = 1;
    }
		
		if (timestamp > hal.next_temp_ms) 
		{
      hal.next_temp_ms = timestamp + TEMP_READ_MS;
      new_temp = 1;
    }
		
		if (hal.motion_int_mode)
		{
			/* Enable motion interrupt. */
			mpu_lp_motion_interrupt(500, 1, 5);
			/* Notify the MPL that contiguity was broken. */
			inv_accel_was_turned_off();
			inv_gyro_was_turned_off();
			inv_compass_was_turned_off();
			inv_quaternion_sensor_was_turned_off();
			/* Wait for the MPU interrupt. */
			while (!hal.new_gyro) {}
			/* Restore the previous sensor configuration. */
			mpu_lp_motion_interrupt(0, 0, 0);
			hal.motion_int_mode = 0;
    }
		
		if (!hal.sensors || !hal.new_gyro)
		{
      continue;
    }    
		
		if (hal.new_gyro && hal.dmp_on)
		{
			short gyro[3], accel_short[3], sensors;
			unsigned char more;
			long accel[3], quat[4], temperature;
			
      dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors, &more);
			if (!more)
				hal.new_gyro = 0;
			
			if (sensors & INV_XYZ_GYRO)
			{
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
		}
		else if (hal.new_gyro)
		{			
			short gyro[3], accel_short[3];
			unsigned char sensors, more;
			long accel[3], temperature;
			
			hal.new_gyro = 0;
			mpu_read_fifo(gyro, accel_short, &sensor_timestamp, &sensors, &more);
			if (more)
				hal.new_gyro = 1;
			if (sensors & INV_XYZ_GYRO)
			{
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
			if (sensors & INV_XYZ_ACCEL)
			{
				accel[0] = (long)accel_short[0];
				accel[1] = (long)accel_short[1];
				accel[2] = (long)accel_short[2];
				inv_build_accel(accel, 0, sensor_timestamp);
				new_data = 1;
			}
		}
		
		if (new_compass)
		{
			short compass_short[3];
			long compass[3];
			new_compass = 0;
			/* For any MPU device with an AKM on the auxiliary I2C bus, the raw
			 * magnetometer registers are copied to special gyro registers.
			 */
			if (!mpu_get_compass_reg(compass_short, &sensor_timestamp))
			{
				compass[0] = (long)compass_short[0];
				compass[1] = (long)compass_short[1];
				compass[2] = (long)compass_short[2];
				/* NOTE: If using a third-party compass calibration library,
				 * pass in the compass data in uT * 2^16 and set the second
				 * parameter to INV_CALIBRATED | acc, where acc is the
				 * accuracy from 0 to 3.
				 */
				inv_build_compass(compass, 0, sensor_timestamp);
			}
			new_data = 1;
    }
		
		if (new_data)
		{
			inv_execute_on_data();
			/* This function reads bias-compensated sensor data and sensor
			 * fusion outputs from the MPL. The outputs are formatted as seen
			 * in eMPL_outputs.c. This function only needs to be called at the
			 * rate requested by the host.
			 */
			if (timestamp > hal.next_data_tx_ms)
			{
				hal.next_data_tx_ms += DATA_TX_MS;
				read_from_mpl();
			}
    }
  }
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

}

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
  HAL_I2C_Init(&hi2c1);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_IS;

  /* GPIO Ports Clock Enable */
  __GPIOH_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
	
	__GPIOD_CLK_ENABLE ();
	
	GPIO_IS.Pin = GPIO_PIN_12;
	GPIO_IS.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_IS.Pull = GPIO_NOPULL;
	GPIO_IS.Speed = GPIO_SPEED_MEDIUM;
	HAL_GPIO_Init(GPIOD, &GPIO_IS);

}


void MX_IRQ_Init (void)
{
	GPIO_InitTypeDef GPIO_IS;
	
	__GPIOB_CLK_ENABLE ();

	GPIO_IS.Pin = GPIO_PIN_4;
	GPIO_IS.Mode = GPIO_MODE_IT_RISING;
	GPIO_IS.Speed = GPIO_SPEED_MEDIUM;
	HAL_GPIO_Init(GPIOB, &GPIO_IS);
	
  HAL_NVIC_SetPriority (EXTI4_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ (EXTI4_IRQn);
}


void Error_Handler (void)
{
	while (1)
		;
}


#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
