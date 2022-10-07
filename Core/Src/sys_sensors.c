/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sys_sensors.c
  * @author  MCD Application Team
  * @brief   Manages the sensors on the application
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "platform.h"
#include "sys_conf.h"
#include "sys_sensors.h"
#if defined (SENSOR_ENABLED) && (SENSOR_ENABLED == 0)
#include "adc_if.h"
#endif /* SENSOR_ENABLED */

/* USER CODE BEGIN Includes */
#include "i2c.h"
/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */
/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/

/* USER CODE BEGIN PD */
#define HUMIDITY_DEFAULT_VAL      50.0f                 /*!< default humidity */
#define TEMPERATURE_DEFAULT_VAL   -10.0f                /*!< default temperature */
#define PRESSURE_DEFAULT_VAL      1000.0f               /*!< default pressure */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

static uint8_t cmd_single_high_stretch[] = { 0x2c, 0x06 };
static uint8_t cmd_read_status[] = { 0xf3, 0x2d };
const uint16_t sht3x_addr_l = 0x44;
const uint16_t sht3x_addr_h = 0x45;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

static uint8_t calculate_crc(const uint8_t *data, size_t length);

/* USER CODE END PFP */

/* Exported functions --------------------------------------------------------*/
int32_t EnvSensors_Read(sensor_t *sensor_data)
{
  /* USER CODE BEGIN EnvSensors_Read */
  return HAL_ERROR;
  /* USER CODE END EnvSensors_Read */
}

int32_t EnvSensors_Init(void)
{
#if defined( USE_IKS01A2_ENV_SENSOR_HTS221_0 ) || defined( USE_IKS01A2_ENV_SENSOR_LPS22HB_0 ) || \
    defined( USE_IKS01A3_ENV_SENSOR_HTS221_0 ) || defined( USE_IKS01A3_ENV_SENSOR_LPS22HH_0 ) || \
    defined( USE_BSP_DRIVER )
  int32_t ret = BSP_ERROR_NONE;
#else
  int32_t ret = 0;
#endif /* USE_BSP_DRIVER */
  /* USER CODE BEGIN EnvSensors_Init */

  MX_I2C2_Init();

  /* USER CODE END EnvSensors_Init */
  return ret;
}

/* USER CODE BEGIN EF */

int32_t EnvNgSensors_Read(ng_sensor_t *sensor_data) {

	HAL_StatusTypeDef ret;
	uint8_t buffer[6];

	// set default values
	sensor_data->humidity    = -1.0;
	sensor_data->temperature = -45.0;

	MX_I2C2_Init();
	ret = HAL_I2C_Master_Transmit(&hi2c2, sht3x_addr_l << 1, cmd_single_high_stretch, sizeof(cmd_single_high_stretch), 30);

	if (ret != HAL_OK) {
		return ret;
	}

	HAL_Delay(1);
	ret = HAL_I2C_Master_Receive(&hi2c2, sht3x_addr_l << 1, buffer, sizeof(buffer), 30);

	uint8_t temp_crc = calculate_crc(buffer, 2);
	uint8_t hum_crc = calculate_crc(buffer + 3, 2);
	if (temp_crc != buffer[2] || hum_crc != buffer[5]) {
		return HAL_ERROR;
	}
	else {
		int16_t temp_raw = (int16_t)buffer[0] << 8 | (uint16_t)buffer[1];
		uint16_t hum_raw = (int16_t)buffer[3] << 8 | (uint16_t)buffer[4];

		sensor_data->temperature = -45.0f + 175.0f * (float)temp_raw / 65535.0f;
		sensor_data->humidity = 100.0f * (float)hum_raw / 65535.0f;
	}

	return HAL_OK;
}

/* USER CODE END EF */

/* Private Functions Definition -----------------------------------------------*/
/* USER CODE BEGIN PrFD */

static uint8_t calculate_crc(const uint8_t *data, size_t length)
{
	uint8_t crc = 0xff;
	for (size_t i = 0; i < length; i++) {
		crc ^= data[i];
		for (size_t j = 0; j < 8; j++) {
			if ((crc & 0x80u) != 0) {
				crc = (uint8_t)((uint8_t)(crc << 1u) ^ 0x31u);
			} else {
				crc <<= 1u;
			}
		}
	}
	return crc;
}

/* USER CODE END PrFD */
