#ifndef __HELLO_WORLD_H__
#define __HELLO_WORLD_H__

#include "uipopt.h"
#include "psock.h"
#include "services/tcp_app_state_structures.h"

/* Next, we define the uip_tcp_appstate_t datatype. This is the state
   of our application, and the memory required for this state is
   allocated together with each TCP connection. One application state
   for each TCP connection. */
/*
 It's possible to overload it with union - different approach to connection for ports
 */


/* Finally we define the application function to be called by uIP. */
void hello_world_appcall(void);
void hello_world_init(void);

#endif
