/*
 * tcp_connection_handler.h
 *
 *  Created on: 14.05.2017
 *      Author: Krzysztof
 */

#ifndef SERVICES_TCP_CONNECTION_HANDLER_H_
#define SERVICES_TCP_CONNECTION_HANDLER_H_
#define UIP_APPCALL tcp_appcall_custom

#include "services/tcp_app_state_structures.h"
#include "http_server.h"
#include "hello-world.h"

void tcp_appcall_custom(void);

#endif /* SERVICES_TCP_CONNECTION_HANDLER_H_ */
