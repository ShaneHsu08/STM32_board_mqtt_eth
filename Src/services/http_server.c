/*
 * http_server.c
 *
 *  Created on: 14.05.2017
 *      Author: Krzysztof
 */

#include "http_server.h"
#include "uip.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "rtc.h"
#include <string.h>


int http_server_handle(struct http_server_state *state)
{
	char buffer[200];

	//uint32_t runtime;
	PSOCK_BEGIN(&state->p);

  // ignore input

	PSOCK_SEND_STR(&state->p, "HTTP/1.1 200 OK \n");
	PSOCK_SEND_STR(&state->p, "Content-Type: text/html \n");
	PSOCK_SEND_STR(&state->p, "Connection: close \n");
	PSOCK_SEND_STR(&state->p, "\n");
	PSOCK_SEND_STR(&state->p, "<!DOCTYPE HTML>\n");
	PSOCK_SEND_STR(&state->p, "<html>");
	PSOCK_SEND_STR(&state->p, "<h1> STM32F103 </h1>");
	PSOCK_SEND_STR(&state->p, "<p> Server time: ");

	RTC_TimeTypeDef time;
	HAL_RTC_GetTime(&hrtc,&time,RTC_FORMAT_BIN);
	sprintf(buffer,"%d:%d:%d UTC",time.Hours,time.Minutes,time.Seconds);
	PSOCK_SEND_STR(&state->p, buffer);
	PSOCK_SEND_STR(&state->p, "</p><table><th><tr><td> Task </td><td> Minimal stack </td><td> Priority </td><td> Usage </td></tr></th>");

	state->taskCount=uxTaskGetNumberOfTasks();
	uxTaskGetSystemState(state->tasks,10,&state->runtime);

	state->index=0;
	// do runtime stats potrzebny dodatkowy element - timer inny ni¿ systick
	while(state->index<state->taskCount){
		sprintf(buffer,"<tr><td>%s</td><td>%d B</td><td>%d</td><td>%d</td></tr>",
				state->tasks[state->index].pcTaskName,
				state->tasks[state->index].usStackHighWaterMark*4,
				state->tasks[state->index].uxCurrentPriority,
				state->tasks[state->index].ulRunTimeCounter
		);
		state->index++;
		PSOCK_SEND_STR(&state->p, buffer);
	}


	PSOCK_SEND_STR(&state->p,"</table></html>");
 /* PSOCK_READTO(&s->p, 'T');
  strncpy(s->name, s->inputbuffer, sizeof(s->name));
  PSOCK_SEND_STR(&s->p, "Hello ");
  PSOCK_SEND_STR(&s->p, s->name);
*/
  PSOCK_CLOSE(&state->p);
  PSOCK_END(&state->p);
}

void http_server_init(void){
	uip_listen(HTONS(80));
}


void http_server_appcall(void){
	if(uip_conn->lport == HTONS(80)){
		struct http_server_state *connection_state = &(uip_conn->appstate.http_server_state_variable);

		if(uip_connected()) {
		    PSOCK_INIT(&connection_state->p, connection_state->inputbuffer, sizeof(connection_state->inputbuffer));

		}

		http_server_handle(connection_state);
	}
}


