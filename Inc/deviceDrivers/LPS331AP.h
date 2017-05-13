/**
 * LPS331AP - barometer device driver
 * Autor: Krzysztof Stachanczyk
 */

#ifndef LPS331AP_DRIVER
#define LPS331AP_DRIVER

#include "FreeRTOS.h"
#include "i2c.h"
#include "LPS331AP_register_map.h"

/**
 *	Settings for "resconf" register
 */

/**
 * 	Pressure resolution configuration
 */
#define LPS331A_PRESSURE_1_SAMPLE_AVG 0x00
#define LPS331A_PRESSURE_2_SAMPLE_AVG 0x01
#define LPS331A_PRESSURE_4_SAMPLE_AVG 0x02
#define LPS331A_PRESSURE_8_SAMPLE_AVG 0x03
#define LPS331A_PRESSURE_16_SAMPLE_AVG 0x04
#define LPS331A_PRESSURE_32_SAMPLE_AVG 0x05
#define LPS331A_PRESSURE_64_SAMPLE_AVG 0x06
#define LPS331A_PRESSURE_128_SAMPLE_AVG 0x07
#define LPS331A_PRESSURE_256_SAMPLE_AVG 0x08
#define LPS331A_PRESSURE_384_SAMPLE_AVG 0x09
// Available only if ORD != 25Hz
#define LPS331A_PRESSURE_512_SAMPLE_AVG 0x0A

/**
 * Temperature resolution configuration
 */
#define LPS331A_TEMPERATURE_1_SAMPLE_AVG (0x00<<4)
#define LPS331A_TEMPERATURE_2_SAMPLE_AVG (0x01<<4)
#define LPS331A_TEMPERATURE_4_SAMPLE_AVG (0x02<<4)
#define LPS331A_TEMPERATURE_8_SAMPLE_AVG (0x03<<4)
#define LPS331A_TEMPERATURE_16_SAMPLE_AVG (0x04<<4)
#define LPS331A_TEMPERATURE_32_SAMPLE_AVG (0x05<<4)
#define LPS331A_TEMPERATURE_64_SAMPLE_AVG (0x06<<4)
// Available only if ORD != 25Hz
#define LPS331A_TEMPERATURE_128_SAMPLE_AVG (0x07<<4)


/**
 * Settings for  "ctrl_reg1" register
 */

/**
 * Power mode
 */
#define LPS331A_POWER_DOWN (0x00<<7)
#define LPS331A_POWER_UP (0x01<<7)

/**
 * Output data rate
 */
#define LPS331A_ODR_ONE_SHOT (0x00<<4)
#define LPS331A_ODR_PRESSURE_1_HZ_TEMPERATURE_1_HZ (0x01<<4)
#define LPS331A_ODR_PRESSURE_7_HZ_TEMPERATURE_1_HZ (0x02<<4)
#define LPS331A_ODR_PRESSURE_12_5_HZ_TEMPERATURE_1_HZ (0x03<<4)
#define LPS331A_ODR_PRESSURE_25_HZ_TEMPERATURE_1_HZ (0x04<<4)
#define LPS331A_ODR_PRESSURE_7_HZ_TEMPERATURE_7_HZ (0x05<<4)
#define LPS331A_ODR_PRESSURE_12_5_HZ_TEMPERATURE_12_5_HZ (0x06<<4)
#define LPS331A_ODR_PRESSURE_25_HZ_TEMPERATURE_25_HZ (0x07<<4)

#define LPS331A_IT_ENABLE (0x01<<3)
#define LPS331A_BDU_LOCK (0x01<<2)
#define LPS331A_DELTA_PRESSURE_ENABLE (0x01<<1)

/**
 * 	Settings for  "ctrl_reg2" register
 */
#define LPS331A_ONE_SHOT_FIRE 0x01

/**
 * 	Settings for ctrl_reg3 - IT control
 * 	supported configuration push-pull , active high
 * 	IT - data ready only
 */
#define LPS331A_INT2_DATA_READY (0x04<<3)
#define LPS331A_INT1_DATA_READY 0x04

/**
 * 	Status register
 */
#define LPS331A_NEW_PRESSURE 0x02
#define LPS331A_NEW_TEMPERATURE 0x01

/**
 *  LPS331AP device structure - represent device configuration
 */
typedef struct {
	uint8_t res_conf;
	uint8_t ctrl_reg1;
	uint8_t ctrl_reg3;
	uint16_t address;
	I2C_HandleTypeDef * i2cSource;
}LPS331AP_device ;

/**
 * Perform LPS331 initialization using configuration values in LPS331AP_device structure
 * @return HAL error code or HAL_OK if completed
 */
HAL_StatusTypeDef LPS331APInit(LPS331AP_device * config);

/**
 * LPS331 read pressure and temperature
 * @param  config - device configuration structure
 * @param temperature - pointer to temperature destination variable
 * @param pressure - pointer to pressure destination variable
 * @return HAL error code or HAL_OK if completed
 */
HAL_StatusTypeDef LPS331APRead(LPS331AP_device * config,float * temperature,float * pressure);

#endif
