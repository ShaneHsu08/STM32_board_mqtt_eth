#ifndef __HELLO_WORLD_H__
#define __HELLO_WORLD_H__

#include "uipopt.h"
#include "psock.h"

/* Next, we define the uip_tcp_appstate_t datatype. This is the state
   of our application, and the memory required for this state is
   allocated together with each TCP connection. One application state
   for each TCP connection. */
/*
 It's possible to overload it with union - different approach to connection for ports
 */
typedef struct hello_world_state {
  struct psock p;
  char inputbuffer[255];
  char name[255];
} uip_tcp_appstate_t;


/* Finally we define the application function to be called by uIP. */
void hello_world_appcall(void);
#ifndef UIP_APPCALL
#define UIP_APPCALL hello_world_appcall
#endif /* UIP_APPCALL */

void hello_world_init(void);

#endif
