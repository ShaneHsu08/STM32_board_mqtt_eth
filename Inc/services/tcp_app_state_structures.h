/*
 * http_server_structure.h
 *
 *  Created on: 14.05.2017
 *      Author: Krzysztof
 */

#ifndef SERVICES_TCP_APP_STATE_STRUCTURES_H_
#define SERVICES_TCP_APP_STATE_STRUCTURES_H_

#include "psock.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

struct http_server_state {
  struct psock p;
  int32_t index;
  TaskStatus_t tasks[10];
  uint32_t taskCount;
  uint32_t runtime;
  char inputbuffer[100];
};

struct hello_world_state {
  struct psock p;
  char inputbuffer[255];
  char name[255];
};


typedef union tcp_app_data{
	struct http_server_state http_server_state_variable;
	struct hello_world_state hello_world_variable;
} uip_tcp_appstate_t;



#endif /* SERVICES_TCP_APP_STATE_STRUCTURES_H_ */
