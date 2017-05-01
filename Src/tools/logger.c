#include "logger.h"

void LOG_init(){
	logSem = xSemaphoreCreateMutex();
}
void LOG(uint8_t mode,char* toLog,uint32_t size){
#ifdef LOG_LVL
	xSemaphoreTake(logSem,portMAX_DELAY);
	if(LOG_LVL>=(mode)){

		switch (mode) {

			case 0:
				HAL_UART_Transmit(&LOG_UART_Handler,(uint8_t *) "[ERROR] ",8,portMAX_DELAY);
				break;

#if LOG_LVL>0
			case 1:
				HAL_UART_Transmit(&LOG_UART_Handler,(uint8_t *) "[WARN] ",7,portMAX_DELAY);
				break;
#endif

#if LOG_LVL>1
			case 2:
				HAL_UART_Transmit(&LOG_UART_Handler,(uint8_t *) "[MSG] ",6,portMAX_DELAY);
				break;
#endif
		}
	}
	HAL_UART_Transmit(&LOG_UART_Handler,(uint8_t *) toLog,size,portMAX_DELAY);
	HAL_UART_Transmit(&LOG_UART_Handler,(uint8_t *) "\n",1,portMAX_DELAY);

	xSemaphoreGive( logSem );
#endif
}
