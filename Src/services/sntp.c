#include "sntp.h"
#include "uip.h"


void sntp_init(uint16_t sntpServerInit [2]){
	sntp_state.server[0] = sntpServerInit[0];
	sntp_state.server[1] = sntpServerInit[1];
	sntp_state.connection = uip_udp_new(&(sntp_state.server),htons(123));
	sntp_state.applicationState = SNTP_READY;
}


void sntp_makeQuery(void){
	sntp_state.applicationState = SNTP_TIME_UPDATE_REQUESTED;
}


void sntp_data_received(void){
	struct ntp_packet * result = (struct ntp_packet *)uip_appdata;

	// Check if its responce
	if(result->mode == NTP_MODE_SERVER ){
		uint8_t * timePointer = (uint8_t *) &(result->receiveTimestamp);
		uint32_t realTimeStamp;
		uint8_t * realTimeStampPointer = (uint8_t *) &realTimeStamp;

		realTimeStampPointer[0]=timePointer[3];
		realTimeStampPointer[1]=timePointer[2];
		realTimeStampPointer[2]=timePointer[1];
		realTimeStampPointer[3]=timePointer[0];

		realTimeStamp -= NTP_TIMESTAMP_DELTA;

		sntp_state.timeStamp=realTimeStamp;
		sntp_state.applicationState =SNTP_DONE;

	}

}

void sntp_appcall()
{
	// match udp package from server
	if(uip_udp_conn->rport == htons(123)) {
		if(uip_poll()) {
			sntp_process();
		}
		if(uip_newdata()) {
			sntp_data_received();
		}
	}
}


void sntp_process(){
	if(sntp_state.connection &&  sntp_state.applicationState == SNTP_TIME_UPDATE_REQUESTED){
		// clear package size
		memset(uip_appdata,0,48);
		struct ntp_packet * package;

		// map to structure
		package = (struct ntp_packet * ) uip_appdata;

		package->li = NTP_LEAP_INDICATOR_NO_WARN;
		package->vn = NTP_VERSION_3;
		package->mode = NTP_MODE_CLIENT;


		// work on internal stack buffer - avoid memcpy
		uip_udp_send(48);
		sntp_state.applicationState = SNTP_WAITING_FOR_RESPONCE;
	}
}



