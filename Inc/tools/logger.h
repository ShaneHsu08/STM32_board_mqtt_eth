#ifndef LOGGER_D
#define LOGGER_D

#include "usart.h"
#include "FreeRTOS.h"
#include "semphr.h"

SemaphoreHandle_t logSem;

#define LOG_ERROR 0
#define LOG_WARN 1
#define LOG_MSG 2

#define LOG_LVL LOG_MSG
#define LOG_UART_Handler huart4

void LOG_init();
void LOG(uint8_t mode,char* toLog,uint32_t size);

#endif

