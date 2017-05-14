/*
 * sntp.h
 *
 *  Created on: 14.05.2017
 *      Author: Krzysztof
 */

#ifndef SERVICES_SNTP_H_
#define SERVICES_SNTP_H_
#include "uipopt.h"
#include "psock.h"
#include "logger.h"
#include <time.h>

#include <stdint.h>
#include <string.h>

#define SNTP_DONE 1
#define SNTP_READY 0
#define SNTP_TIME_UPDATE_REQUESTED 2
#define SNTP_WAITING_FOR_RESPONCE 3
#define SNTP_TIMEOUT 4

#define NTP_LEAP_INDICATOR_NO_WARN 0
#define NTP_MODE_CLIENT 3
#define NTP_MODE_SERVER 4
#define NTP_VERSION_3 3
#define NTP_TIMESTAMP_DELTA 2208988800

struct ntp_mode{
	  unsigned li   : 2;
	  unsigned vn   : 3;
	  unsigned mode : 3;
};

struct ntp_packet
{
  unsigned mode : 3;
  unsigned vn   : 3;
  unsigned li   : 2;



  uint8_t stratum;
  uint8_t poll;
  uint8_t precision;

  uint32_t rootDelay;
  uint32_t rootDispersion;
  uint32_t referenceIdentifier;

  uint32_t referenceTimestamp;
  uint32_t referenceTimestamp_fraction;

  uint32_t originateTimestamp;
  uint32_t originateTimestamp_fraction;

  uint32_t receiveTimestamp;
  uint32_t receiveTimestamp_fraction;

  uint32_t transmitTimestamp ;
  uint32_t transmitTimestamp_fraction;
};

// sntp application context
struct sntp_state_structure{
	volatile uint8_t applicationState;
	volatile uint32_t timeStamp;
	volatile struct uip_udp_conn * connection;
	volatile uint16_t server[2];
};

struct sntp_state_structure sntp_state;

void sntp_init(uint16_t sntpServerInit [2]);
void sntp_appcall();
void sntp_makeQuery();
void sntp_process();


#endif /* SERVICES_SNTP_H_ */
