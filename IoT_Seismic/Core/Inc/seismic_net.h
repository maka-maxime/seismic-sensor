#ifndef __SEISMIC_NET_H__
#define __SEISMIC_NET_H__

#include "seismic.h"

typedef enum net_msg_type
{
	NET_MSG_PRESENCE = 0x00,
	NET_MSG_DATA = 0x01
} NetMessageType;

extern osThreadId ethernetLinkMonitorHandle;
extern osThreadId networkBroadcastHandle;
extern osThreadId networkClientHandle;
extern osThreadId networkServerHandle;

extern uint8_t ethernetLinkState;
extern uint8_t tasksState;

void initNetwork(void);
void EthernetLinkMonitor(void const * argument);
void NetworkBroadcast(void const * argument);
void NetworkClient(void const * argument);
void NetworkServer(void const * argument);

#endif /* __SEISMIC_NET_H__ */
