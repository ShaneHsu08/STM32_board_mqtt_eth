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
#include "userDefines.h"
#include "usart.h"
#include "i2c.h"
#include "rtc.h"
//#include "iwdg.h"
#include "logger.h"
#include "LPS331AP.h"

#include <string.h>
// UI stack

#include "uip.h"
#include "uip_arp.h"
#include "enc28j60.h"

#include "services/sntp.h"


#ifndef BUF
#define BUF ((struct uip_eth_hdr *)&uip_buf[0])
#endif
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
osThreadId LPS331MeasuHandle;
osThreadId iwdtTaskHandle;
osThreadId stack_perHandle;
osThreadId stackHandle;
osThreadId time_updateHandle;

/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);
void LPS331MeasureFunction(void const * argument);
void iwdtTaskFunction(void const * argument);
void vTask_stack_periodic(void const * argument);
void vTask_stack_main(void const * argument);
extern void sntpTimeUpdateTask(void const * argument);

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

  /* definition and creation of LPS331Measu */
  osThreadDef(LPS331Measu, LPS331MeasureFunction, osPriorityHigh, 0, 128);
  LPS331MeasuHandle = osThreadCreate(osThread(LPS331Measu), NULL);

  /* definition and creation of iwdtTask */
  osThreadDef(iwdtTask, iwdtTaskFunction, osPriorityRealtime, 0, 256);
  iwdtTaskHandle = osThreadCreate(osThread(iwdtTask), NULL);

  /* definition and creation of stack_per */
  osThreadDef(stack_per, vTask_stack_periodic, osPriorityHigh, 0, 512);
  stack_perHandle = osThreadCreate(osThread(stack_per), NULL);

  /* definition and creation of stack */
  osThreadDef(stack, vTask_stack_main, osPriorityHigh, 0, 512);
  stackHandle = osThreadCreate(osThread(stack), NULL);

  /* definition and creation of time_update */
  osThreadDef(time_update, sntpTimeUpdateTask, osPriorityHigh, 0, 128);
  time_updateHandle = osThreadCreate(osThread(time_update), NULL);

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
	for (;;) {
		osDelay(1000);
		LOG(LOG_MSG, "DefaultTask", 11);
	}
  /* USER CODE END StartDefaultTask */
}

/* LPS331MeasureFunction function */
void LPS331MeasureFunction(void const * argument)
{
  /* USER CODE BEGIN LPS331MeasureFunction */
	/* Infinite loop */


	 for (;;) {
		osDelay(1000);
		}
		/**
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

	int counter = 0;
	for (;;) {
		counter++;

		char buffer[40];
		uint8_t s = sprintf(buffer, "LSP331Task C: %d", counter);
		//LOG(LOG_MSG,"LSP331Task",10);
		LOG(LOG_MSG, buffer, s);
		osDelay(1000);
	}
	*/
  /* USER CODE END LPS331MeasureFunction */
}

/* iwdtTaskFunction function */
void iwdtTaskFunction(void const * argument)
{
  /* USER CODE BEGIN iwdtTaskFunction */
	/* Infinite loop */

#ifndef NO_IWDT

	HAL_IWDG_Start(&hiwdg);

	for(;;)
	{
		osDelay(1000);
		HAL_IWDG_Refresh(&hiwdg);
		//LOG(LOG_MSG,"IWDG Refresh",12);
	}

#else

	for (;;) {
		osDelay(1000);
		char buf [10];

		RTC_TimeTypeDef time;
		HAL_RTC_GetTime(&hrtc,&time,RTC_FORMAT_BIN);

		int count = sprintf(buf,"%d:%d:%d",time.Hours,time.Minutes,time.Seconds);
		LOG(LOG_MSG, buf, count);
	}
#endif

  /* USER CODE END iwdtTaskFunction */
}

/* vTask_stack_periodic function */
void vTask_stack_periodic(void const * argument)
{
  /* USER CODE BEGIN vTask_stack_periodic */
  /* Infinite loop */
	uint32_t i;
		uint8_t delay_arp = 0;
		volatile uint32_t st;

		for (;;) {
			vTaskDelay(configTICK_RATE_HZ/2); // ����������
			delay_arp++;
			for (i = 0; i < UIP_CONNS; i++) {
				uip_periodic(i);
				if (uip_len > 0) {
					uip_arp_out();
					enc28j60_send_packet((uint8_t *) uip_buf, uip_len);
				}
			}

	#if UIP_UDP
			for(i = 0; i < UIP_UDP_CONNS; i++) {
				uip_udp_periodic(i);
				if(uip_len > 0) {
					uip_arp_out();
					enc28j60_send_packet((uint8_t *) uip_buf, uip_len);
				}
			}
	#endif /* UIP_UDP */

			if (delay_arp >= 50) {
				delay_arp = 0;
				uip_arp_timer();
			}

			st = uxTaskGetStackHighWaterMark(NULL);
		}
  /* USER CODE END vTask_stack_periodic */
}

/* vTask_stack_main function */
void vTask_stack_main(void const * argument)
{
  /* USER CODE BEGIN vTask_stack_main */
  /* Infinite loop */
	volatile uint32_t st;

 //TODO integrate with incomming intrerupt  from enc to make it faster
	for (;;) {
		vTaskDelay(100);
		uip_len = enc28j60_recv_packet((uint8_t *) uip_buf, UIP_BUFSIZE);

		if (uip_len > 0) {
			if (BUF->type == htons(UIP_ETHTYPE_IP)) {
				uip_arp_ipin();
				uip_input();
				if (uip_len > 0) {
					uip_arp_out();
					enc28j60_send_packet((uint8_t *) uip_buf, uip_len);
				}
			} else if (BUF->type == htons(UIP_ETHTYPE_ARP)) {
				uip_arp_arpin();
				if (uip_len > 0) {
					enc28j60_send_packet((uint8_t *) uip_buf, uip_len);
				}
			}
		}
		taskYIELD();
		st = uxTaskGetStackHighWaterMark(NULL);
	}
  /* USER CODE END vTask_stack_main */
}

/* USER CODE BEGIN Application */
void uip_log(char *msg) {
	LOG(LOG_MSG, msg, strlen(msg) );
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
