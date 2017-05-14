/*
 * udp_connection_handler.h
 *
 *  Created on: 14.05.2017
 *      Author: Krzysztof
 */

#ifndef SERVICES_UDP_CONNECTION_HANDLER_H_
#define SERVICES_UDP_CONNECTION_HANDLER_H_


#include "sntp.h"



void udp_appcall_custom();

#define UIP_UDP_APPCALL udp_appcall_custom
typedef int  uip_udp_appstate_t;

#endif /* SERVICES_UDP_CONNECTION_HANDLER_H_ */
