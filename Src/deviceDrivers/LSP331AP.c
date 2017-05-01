/*
 * LSP331AP.c
 *
 *  Created on: 01.05.2017
 *      Author: krzys
 */
#include "LPS331AP.h"

HAL_StatusTypeDef LPS331APInit(LPS331AP_device * config){
	uint8_t PD_res_conf = (~LPS331A_POWER_UP)&config->ctrl_reg1;

	uint8_t status;

	status=HAL_I2C_Mem_Write(config->i2cSource,config->address,LPS331A_RW_CTRL_REG1,1,&PD_res_conf,1,100);
		if(status!=HAL_OK) return status;

	status = HAL_I2C_Mem_Write(config->i2cSource,config->address,LPS331A_RW_RES_CONF,1,&config->res_conf,1,100);
		if(status!=HAL_OK) return status;

	status = HAL_I2C_Mem_Write(config->i2cSource,config->address,LPS331A_RW_CTRL_REG3,1,&config->ctrl_reg3,1,100);
		if(status!=HAL_OK) return status;

	status=HAL_I2C_Mem_Write(config->i2cSource,config->address,LPS331A_RW_CTRL_REG1,1,&config->ctrl_reg1,1,100);

	return status;
}

HAL_StatusTypeDef LPS331APRead(LPS331AP_device * config,float * temperature,float * pressure){
	uint8_t buffer[5];
	uint8_t status;

	// 1 on MSB of register adress enable multiple read with SUB autoincrement
	status=HAL_I2C_Mem_Read(config->i2cSource,config->address,(0x01<<7) | LPS331A_R_PRESS_OUT_XL_REG,1,buffer,5,1000);

	if(status!=HAL_OK) return status;

	*pressure = ((float)(((uint32_t)buffer[0]) | ((uint32_t)buffer[1])<<8 | ((uint32_t)buffer[2])<<16))/4096.0;

	int16_t t = ((int16_t) buffer[3]) | ((int16_t) buffer[4]<<8);
	*temperature = 42.5+(float) t/480.0;

	return HAL_OK;
}



