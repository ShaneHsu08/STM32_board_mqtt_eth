#ifndef LPS331AP_DRIVER
#define LPS331AP_DRIVER

#include "FreeRTOS.h"
#include "i2c.h"


// register map
#define LPS331A_RW_REF_P_XL 0x08
#define LPS331A_RW_REF_P_L 0x09
#define LPS331A_RW_REF_P_H 0x0A
#define LPS331A_R_WHO_AM_I 0x0F
#define LPS331A_RW_RES_CONF 0x10
#define LPS331A_RW_CTRL_REG1 0x20
#define LPS331A_RW_CTRL_REG2 0x21
#define LPS331A_RW_CTRL_REG3 0x22
#define LPS331A_RW_INT_CFG_REG 0x23
#define LPS331A_R_INT_SOURCE_REG 0x24
#define LPS331A_RW_THS_P_LOW_REG 0x25
#define LPS331A_RW_THS_P_HIGH_REG 0x26
#define LPS331A_R_STATUS_REG 0x27
#define LPS331A_R_PRESS_OUT_XL_REG 0x28
#define LPS331A_R_PRESS_OUT_L 0x29
#define LPS331A_R_PRESS_OUT_H 0x2A
#define LPS331A_R_TEMP_OUT_L 0x2B
#define LPS331A_R_TEMP_OUT_H 0x2C

/**
 * 		   settings for res_conf
 */

// pressure resolution configuration
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
// if ORD != 25Hz
#define LPS331A_PRESSURE_512_SAMPLE_AVG 0x0A

// temperature resolution configuration
#define LPS331A_TEMPERATURE_1_SAMPLE_AVG (0x00<<4)
#define LPS331A_TEMPERATURE_2_SAMPLE_AVG (0x01<<4)
#define LPS331A_TEMPERATURE_4_SAMPLE_AVG (0x02<<4)
#define LPS331A_TEMPERATURE_8_SAMPLE_AVG (0x03<<4)
#define LPS331A_TEMPERATURE_16_SAMPLE_AVG (0x04<<4)
#define LPS331A_TEMPERATURE_32_SAMPLE_AVG (0x05<<4)
#define LPS331A_TEMPERATURE_64_SAMPLE_AVG (0x06<<4)

// if ORD != 25Hz
#define LPS331A_TEMPERATURE_128_SAMPLE_AVG (0x07<<4)


/**
 * 		   settings for ctrl_reg1
 */

// power up-down
#define LPS331A_POWER_DOWN (0x00<<7)
#define LPS331A_POWER_UP (0x01<<7)
// output data rate
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
 * 			ctrl_reg2 issues
 */
#define LPS331A_ONE_SHOT_FIRE 0x01

/**
 * 			settings for ctrl_reg3 - IT control
 * 			supported configuration push-pull , active high
 * 			IT - data ready only
 */
#define LPS331A_INT2_DATA_READY (0x04<<3)
#define LPS331A_INT1_DATA_READY 0x04

/**
 * 			STATUS REG
 */
#define LPS331A_NEW_PRESSURE 0x02
#define LPS331A_NEW_TEMPERATURE 0x01

typedef struct {
	// i2c handler
	uint8_t res_conf;
	uint8_t ctrl_reg1;
	uint8_t ctrl_reg3;
	uint16_t address;
	I2C_HandleTypeDef * i2cSource;
}LPS331AP_device ;


HAL_StatusTypeDef LPS331APInit(LPS331AP_device * config);
HAL_StatusTypeDef LPS331APRead(LPS331AP_device * config,float * temperature,float * pressure);


#endif
