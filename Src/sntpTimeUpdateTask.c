/*
 * rtc_task.c
 *
 *  Created on: 14.05.2017
 *      Author: Krzysztof
 */

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "rtc.h"
#include "services/sntp.h"

void sntpTimeUpdateTask(void const * argument){
	for (;;) {
		osDelay(9000);
		sntp_makeQuery();

		osDelay(1000);
		if(sntp_state.applicationState==SNTP_DONE){
			struct tm * ptm;
			ptm = gmtime(( const time_t* ) &(sntp_state.timeStamp));

			RTC_TimeTypeDef rtcTime;
			rtcTime.Hours = ptm->tm_hour;
			rtcTime.Minutes = ptm->tm_min;
			rtcTime.Seconds= ptm->tm_sec;

			RTC_DateTypeDef rtcDate;
			rtcDate.Date = ptm->tm_mday;
			rtcDate.Month =ptm->tm_mon;
			rtcDate.Year = ptm->tm_year;
			rtcDate.WeekDay= ptm->tm_wday;

			HAL_RTC_SetTime(&hrtc,&rtcTime,RTC_FORMAT_BIN);
			HAL_RTC_SetDate(&hrtc,&rtcDate,RTC_FORMAT_BIN);

			LOG(LOG_MSG,"SNTP SYNC",10);
		}
	}
}
