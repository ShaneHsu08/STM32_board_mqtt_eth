/*
 * http_server.h
 *
 *  Created on: 14.05.2017
 *      Author: Krzysztof
 */



#ifndef SERVICES_HTTP_SERVER_H_
#define SERVICES_HTTP_SERVER_H_

#include "services/tcp_app_state_structures.h"
#include "uipopt.h"
#include "psock.h"


void http_server_init(void);
void http_server_appcall(void);




#endif /* SERVICES_HTTP_SERVER_H_ */
