#include "logger.h"

void LOG_init(){
	logSem = xSemaphoreCreateMutex();
}
void LOG(uint8_t * mode,char* toLog,uint32_t size){
	xSemaphoreTake(logSem,portMAX_DELAY);
	if(LOG_LVL>=(*mode)){

		switch (*mode) {
			case 0:
				HAL_UART_Transmit(&UART_Handler,(uint8_t *) "[ERROR] ",8,portMAX_DELAY);
				break;
			case 1:
				HAL_UART_Transmit(&UART_Handler,(uint8_t *) "[WARN] ",7,portMAX_DELAY);
				break;
			case 2:
				HAL_UART_Transmit(&UART_Handler,(uint8_t *) "[MSG] ",6,portMAX_DELAY);
				break;
		}
	}
	HAL_UART_Transmit(&UART_Handler,(uint8_t *) toLog,size,portMAX_DELAY);
	HAL_UART_Transmit(&UART_Handler,(uint8_t *) "\n",1,portMAX_DELAY);

	xSemaphoreGive( logSem );
}
