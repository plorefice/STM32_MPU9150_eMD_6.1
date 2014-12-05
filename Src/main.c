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

volatile uint32_t hal_timestamp = 0;
#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
#define COMPASS_ON      (0x04)

#define DEFAULT_MPU_HZ  (100)

#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)

#define TEMP_READ_MS    (2000)
#define SERIAL_MS       (20)
#define COMPASS_READ_MS (20)

#define MAG_CALIB_SAMPLE_NUM   (200)
#define MAG_CALIB_EXIT_AFTER   (3)
#define MAG_CALIB_MAX_ITER     (300)
#define MAG_CALIB_TGT_ERROR    (0.001f)

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

struct hal_s hal = {0};

struct point3 { float x, y, z; };

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


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void Board_Init (void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_IRQ_Init (void);
static void Error_Handler (void);
float mean3 (float *data, int start, int disp, int count);
int   mag_online_calib (struct point3 *data, struct point3 *calib_vec);


int mag_online_calib (struct point3 *data, struct point3 *calib_vec)
{
	float  L[MAG_CALIB_SAMPLE_NUM];
	float  x_m, y_m, z_m;
	float  a, b, c;
	float  na, nb, nc;
	float  L_m, L_a, L_b, L_c;
	float  tgt_error;
	int    it_num, exit_cond;
	int    i;

	/* Initialize loop variables */
	it_num = 0;                          /* Iteration counter */
	tgt_error = MAG_CALIB_TGT_ERROR;     /* The maximum error we can accept */
	exit_cond = MAG_CALIB_EXIT_AFTER;    /* Specify after how many iterations
										 * for which calib_data_out < tgt_error
										 * we should exit */

	/* Initialize coordinate means */
	x_m = mean3((float *) data, 0, 3, MAG_CALIB_SAMPLE_NUM);
	y_m = mean3((float *) data, 1, 3, MAG_CALIB_SAMPLE_NUM);
	z_m = mean3((float *) data, 2, 3, MAG_CALIB_SAMPLE_NUM);

	/* Iteration 0 */
	a = x_m;
	b = y_m;
	c = z_m;

	while (it_num < MAG_CALIB_MAX_ITER)
	{
		/* Reset variables */
		L_a = 0.f;
		L_b = 0.f;
		L_c = 0.f;

		/* Increment iteration counter */
		it_num++;

		/* Compute Li */
		for (i = 0; i < MAG_CALIB_SAMPLE_NUM; i++)
			L[i]  = pow(data[i].x - a, 2) +
			        pow(data[i].y - b, 2) +
			        pow(data[i].z - c, 2);

		/* Compute Lm */
		L_m = mean3(L, 0, 1, MAG_CALIB_SAMPLE_NUM);

		/* Compute La */
		for (i = 0; i < MAG_CALIB_SAMPLE_NUM; i++)
			L_a += (a - data[i].x) / L[i];
		L_a /= MAG_CALIB_SAMPLE_NUM;

		/* Compute Lb */
		for (i = 0; i < MAG_CALIB_SAMPLE_NUM; i++)
			L_b += (b - data[i].y) / L[i];
		L_b /= MAG_CALIB_SAMPLE_NUM;

		/* Compute Lc */
		for (i = 0; i < MAG_CALIB_SAMPLE_NUM; i++)
			L_c += (c - data[i].z) / L[i];
		L_c /= MAG_CALIB_SAMPLE_NUM;

		/* Compute new values */
		na = x_m + L_m * L_a;
		nb = y_m + L_m * L_b;
		nc = z_m + L_m * L_c;

		/* Check exit conditions */
		if (fabs(na - a) < tgt_error &&
			fabs(nb - b) < tgt_error &&
			fabs(nc - c) < tgt_error)
			exit_cond++;
		else
			exit_cond = 0;

		/* Store new values */
		a = na;
		b = nb;
		c = nc;

		/* Exit if verified */
		if (exit_cond > MAG_CALIB_EXIT_AFTER)
			break;
	}

	calib_vec->x = a;
	calib_vec->y = b;
	calib_vec->z = c;

	return it_num;
}


float mean3 (float *data, int start, int disp, int count)
{
	float ret = 0.f;
	int i = 0;

	data += start;

	for (; i < count; i++)
		ret += data[i * disp];

	return (ret / count);
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
	struct point3 raw_mag_data[MAG_CALIB_SAMPLE_NUM];
	struct point3 calib_vec, data[3];
	int num_samples = 0;
	int calibrated = 0;
	
	int_param.cb = NULL;
	
	Board_Init();
	
	result = mpu_init(&int_param);
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
	
	hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL | 
										 DMP_FEATURE_SEND_CAL_GYRO |	DMP_FEATURE_GYRO_CAL;
	dmp_enable_feature(hal.dmp_features);
	dmp_set_fifo_rate(DEFAULT_MPU_HZ);
	mpu_set_dmp_state(1);
	
	hal.dmp_on = 1;
	
	MX_IRQ_Init();
	
	run_self_test();
	
  /* Infinite loop */
  while (1)
  {
    unsigned long sensor_timestamp;
    int new_data = 0;
		
		timestamp = HAL_GetTick();
		
		if ((timestamp > hal.next_compass_ms) && hal.new_gyro && (hal.sensors & COMPASS_ON))
		{
			hal.next_compass_ms = timestamp + COMPASS_READ_MS;
			new_compass = 1;
    }
		
		if (timestamp > hal.next_temp_ms) 
		{
      hal.next_temp_ms = timestamp + TEMP_READ_MS;
      new_temp = 1;
    }
		
		if (timestamp > hal.next_serial_ms)
		{
			hal.next_serial_ms = timestamp + SERIAL_MS;
			
			USBD_CDC_SetTxBuffer(&hUsbDeviceFS, (uint8_t *) data, sizeof(data));
			USBD_CDC_TransmitPacket(&hUsbDeviceFS);
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
				float sens;
				
				new_data = 1;
				if (new_temp)
				{
					new_temp = 0;
					/* Temperature only used for gyro temp comp. */
					mpu_get_temperature(&temperature, &sensor_timestamp);
				}
				
				switch(gyro_fsr)
				{
				case 250:
					sens = 131.0f;
					break;
				case 500:
					sens = 65.5f;
					break;
				case 1000:
					sens = 32.8f;
					break;
				case 2000:
					sens = 16.4f;
					break;
				default:
					sens = 0.f;
					break;
				}
				
				data[1].x = (float) gyro[0] / sens;
				data[1].y = (float) gyro[1] / sens;
				data[1].z = (float) gyro[2] / sens;
			}
			if (sensors & INV_XYZ_ACCEL)
			{
				accel[0] = (long)accel_short[0];
				accel[1] = (long)accel_short[1];
				accel[2] = (long)accel_short[2];
				
				new_data = 1;
				
				data[0].x = (float) accel[0] / 16384.0f;
				data[0].y = (float) accel[1] / 16384.0f;
				data[0].z = (float) accel[2] / 16384.0f;
			}
		}
		
		if (new_compass)
		{
			short compass_short[3];
			long compass[3];
			new_compass = 0;
			
			if (!mpu_get_compass_reg(compass_short, &sensor_timestamp))
			{
				compass[0] = (long)compass_short[0];
				compass[1] = (long)compass_short[1];
				compass[2] = (long)compass_short[2];
				
				if (calibrated == 0)
				{
					raw_mag_data[num_samples].x = (float) compass[0] * 0.3f;
					raw_mag_data[num_samples].y = (float) compass[1] * 0.3f;
					raw_mag_data[num_samples].z = (float) compass[2] * 0.3f;
					num_samples++;
					
					if (num_samples == MAG_CALIB_SAMPLE_NUM)
					{
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
						
						mag_online_calib(raw_mag_data, &calib_vec);
						calibrated = 1;
						num_samples = 0;
						
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
					}
				}
				else
				{
					data[2].x = ((float) compass[0] * 0.3f) - calib_vec.x;
					data[2].y = ((float) compass[1] * 0.3f) - calib_vec.y;
					data[2].z = ((float) compass[2] * 0.3f) - calib_vec.z;
				}
			}
			
			new_data = 1;
    }
		
		if (new_data)
		{
			new_data = 0;
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
	
	GPIO_IS.Pin = GPIO_PIN_13;
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
