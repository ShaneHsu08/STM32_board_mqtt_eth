#ifndef __UIP_CONF_H__
#define __UIP_CONF_H__

//#include <inttypes.h>
#include <stdint.h>

// data types ports
typedef uint8_t u8_t;
typedef uint16_t u16_t;

// type for stack statistics
typedef unsigned short uip_stats_t;

// TCP connections limit
#define UIP_CONF_MAX_CONNECTIONS 40

// TCP  listening ports number
#define UIP_CONF_MAX_LISTENPORTS 40

// uIP buffer size
#define UIP_CONF_BUFFER_SIZE     420

// CPU byte order - for ARM STM32 LITTLE ENDIAN
#define UIP_CONF_BYTE_ORDER      LITTLE_ENDIAN

// Logging off
#define UIP_CONF_LOGGING         0

// UDP support off
#define UIP_CONF_UDP             1

// Check UDP checksum
#define UIP_CONF_UDP_CHECKSUMS   1

// Stack stats on
#define UIP_CONF_STATISTICS      1



#include "services/udp_connection_handler.h"
#include "services/tcp_connection_handler.h"

/*#include "smtp.h"*/
//#include "hello-world.h"
/*#include "telnetd.h"*/
//#include "webserver.h"
/*#include "dhcpc.h"*/
/*#include "resolv.h"*/
/*#include "webclient.h"*/


#endif
