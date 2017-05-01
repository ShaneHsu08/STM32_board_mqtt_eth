/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "usart.h"
#include "i2c.h"

#include "logger.h"
#include "LPS331AP.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
osThreadId UartControlHandle;
osThreadId LPS331MeasuHandle;

/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);
void UartControlFunction(void const * argument);
void LPS331MeasureFunction(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  LOG_init();
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of UartControl */
  osThreadDef(UartControl, UartControlFunction, osPriorityIdle, 0, 128);
  UartControlHandle = osThreadCreate(osThread(UartControl), NULL);

  /* definition and creation of LPS331Measu */
  osThreadDef(LPS331Measu, LPS331MeasureFunction, osPriorityHigh, 0, 128);
  LPS331MeasuHandle = osThreadCreate(osThread(LPS331Measu), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {

	  // internall buffer
	//  char buffer[40];
	//  uint32_t size =  sprintf(buffer,"Hellow %d",23);
	  // task blocking send
	//  LOG(LOG_ERROR,buffer,size);
	  osDelay(10000);
  }
  /* USER CODE END StartDefaultTask */
}

/* UartControlFunction function */
void UartControlFunction(void const * argument)
{
  /* USER CODE BEGIN UartControlFunction */
  /* Infinite loop */
  for(;;)
  {
	  // internall buffe
	  osDelay(10000);
  }
  /* USER CODE END UartControlFunction */
}

/* LPS331MeasureFunction function */
void LPS331MeasureFunction(void const * argument)
{
  /* USER CODE BEGIN LPS331MeasureFunction */
  /* Infinite loop */

  // TODO make it secure
  LPS331AP_device lps331;
  lps331.ctrl_reg1 =LPS331A_POWER_UP | LPS331A_ODR_PRESSURE_1_HZ_TEMPERATURE_1_HZ;
  lps331.ctrl_reg3 = 0;
  // HAL BUG
  lps331.address = 0x5D<<1;
  lps331.res_conf = LPS331A_PRESSURE_384_SAMPLE_AVG | LPS331A_TEMPERATURE_128_SAMPLE_AVG;
  lps331.i2cSource= &hi2c1;

  uint8_t status = LPS331APInit(&lps331);

  char buffer[40];
  uint32_t size =  sprintf(buffer,"LPS init status %d",status);
  // task blocking send
  LOG(LOG_MSG,buffer,size);

  for(;;)
  {
	  float x,y;
    LPS331APRead(&lps331,&x,&y);
    osDelay(1000);
  }
  /* USER CODE END LPS331MeasureFunction */
}

/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
