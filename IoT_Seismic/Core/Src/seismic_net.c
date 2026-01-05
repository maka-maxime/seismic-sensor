#include <string.h>

#include "lwip/netif.h"
#include "lwip/sockets.h"

#include "seismic_log.h"
#include "seismic_net.h"

#define BROAD_PORT               (12345)
#define BROAD_SEND_DELAY         (10000)
#define CLIENT_LOOP_DELAY        (10000)
#define CLIENT_CONNECT_DELAY     (500)
#define CLIENT_CONNECT_TIMEOUT   (500000)  // microseconds
#define CLIENT_EOC_DELAY         (75)
#define CLIENT_FLAGS             (0)
#define CLIENT_MAX_RETRIES       (4)
#define CLIENT_POLL_REMOTES      (2000)
#define ETH_LINK_DOWN            (0)
#define ETH_LINK_PAUSED          (2)
#define ETH_LINK_POLLING_DELAY   (200)
#define ETH_LINK_STARTUP_DELAY   (2000)
#define ETH_LINK_UP              (1)
#define NET_BUFFER_LENGTH        (144)
#define NET_MAX_NODES            (20)
#define NET_MSGTYPE_LEN          (10)
#define NET_MSGTYPE_DATA         "data"
#define NET_MSGTYPE_PRES         "presence"
#define NET_RECVFROM_TIMEOUT     (500000) // microseconds
#define NODES_ALREADY            (-2)
#define NODES_ERROR              (-1)
#define NODES_SUCCESS            (0)
#define SERVER_ACCEPT_POLLING    (200)
#define SERVER_BACKLOG           (1)
#define SERVER_FLAGS             (0)
#define SERVER_LISTEN_ADDR       "0.0.0.0"
#define SERVER_LISTEN_PORT       (12345)
#define SOCK_BROADCAST           (0x00000002)
#define SOCK_BIND                (0x00000004)
#define SOCK_ERROR_DELAY         (1000)
#define SOCK_NONBLOCK            (0x00000001)

typedef struct node_entry {
	uint32_t node_addr;
	char node_ipstr[INET_ADDRSTRLEN];
	char node_id[NODE_ID_LEN];
} NodeEntry;

static const char *net_presence_format =
		"{"
		  "\"type\": \"presence\","
		  "\"id\": \"" NODE_ID "\","
		  "\"ip\": \"" NODE_IP "\","
		  "\"timestamp\": \"%s\""
		"}";
static const char *net_data_format =
		"{"
		  "\"type\":\"data\","
		  "\"id\":\"" NODE_ID "\","
		  "\"timestamp\":\"%s\","
		  "\"acceleration\":{"
		    "\"x\":%.2f,"
		    "\"y\":%.2f,"
		    "\"z\":%.2f"
		  "},"
			"\"status\":\"%s\""
		"}";
static ssize_t nodeCount = 0;
static osMutexId nodesMutexHandle;
static NodeEntry nodeRegister[NET_MAX_NODES] = {0};

static int32_t connectWithTimeout(int32_t hSocket, struct sockaddr *remoteAddress, socklen_t socksize);
static ssize_t formatNetMessage(NetMessageType type, char *messageBuffer, size_t bufferSize);
static ssize_t getNextRegisteredNode(ssize_t currentNode);
static inline NodeEntry *getNode(uint32_t nodeIndex);
static char *jsonGetValue(const char *__restrict json, const char *__restrict key, char *__restrict value, size_t value_length);
static int32_t genericSocket(int32_t domain, int32_t type, int32_t protocol, uint32_t flags, struct sockaddr *address, socklen_t addressLength);
static int32_t recvfromWithTimeout(int32_t hSocket, void *buffer, size_t bufferLenght, int32_t flags, struct sockaddr *address, socklen_t *addressLength);
static int32_t registerNode(uint32_t address, const char *__restrict ipstr, const char *__restrict id);
static int32_t removeNode(uint32_t idx);
static int32_t resetNodes(void);
static int32_t tcpSocket(uint32_t flags, struct sockaddr *address, socklen_t addressLength);
static int32_t udpSocket(uint32_t flags, struct sockaddr *address, socklen_t addressLength);
static int32_t setNonBlocking(int32_t hSocket);

osThreadId ethernetLinkMonitorHandle;
osThreadId networkBroadcastHandle;
osThreadId networkClientHandle;
osThreadId networkListenerHandle;
osThreadId networkServerHandle;

uint8_t ethernetLinkState = ETH_LINK_DOWN;

void initNetwork()
{
  osMutexDef(nodesMutex);
  nodesMutexHandle = osMutexCreate(osMutex(nodesMutex));

  osThreadDef(ethernetLinkMonitor, EthernetLinkMonitor, osPriorityNormal, 0, 256);
	ethernetLinkMonitorHandle = osThreadCreate(osThread(ethernetLinkMonitor), NULL);

	osThreadDef(networkBroadcast, NetworkBroadcast, osPriorityNormal, 0, 512);
	networkBroadcastHandle = osThreadCreate(osThread(networkBroadcast), NULL);

	osThreadDef(networkServer, NetworkServer, osPriorityNormal, 0, 384);
	networkServerHandle = osThreadCreate(osThread(networkServer), NULL);

	osThreadDef(networkClient, NetworkClient, osPriorityNormal, 0, 384);
	networkClientHandle = osThreadCreate(osThread(networkClient), NULL);

	osThreadDef(networkListener, NetworkListener, osPriorityNormal, 0, 384);
	networkListenerHandle = osThreadCreate(osThread(networkListener), NULL);
}

void EthernetLinkMonitor(void const * argument)
{
	uint8_t currentLinkState = ETH_LINK_DOWN;

	osSignalWait(SIG_LWIP, osWaitForever);
	// Wait for LWIP to finalise its initialisation.
	osDelay(ETH_LINK_STARTUP_DELAY);

	logMessage(TID_ETH "Ethernet link monitor ready." ENDL);
  while (1)
  {
  	currentLinkState = netif_is_link_up(netif_default);

  	if (tasksState == SYS_TASKS_PAUSED)
  		currentLinkState = ETH_LINK_PAUSED;

  	if (currentLinkState == ethernetLinkState)
  	{
  		osDelay(ETH_LINK_POLLING_DELAY);
  		continue;
  	}

  	ethernetLinkState = currentLinkState;
  	switch (ethernetLinkState)
  	{
  	case ETH_LINK_DOWN:
  		logMessage(TID_ETH "Link is down." ENDL);
  		resetNodes();
  		break;
  	case ETH_LINK_UP:
  		logMessage(TID_ETH "Link is up." ENDL);
  		osSignalSet(networkBroadcastHandle, SIG_LINK_UP);
  		osSignalSet(networkClientHandle, SIG_LINK_UP);
  		osSignalSet(networkListenerHandle, SIG_LINK_UP);
  		osSignalSet(networkServerHandle, SIG_LINK_UP);
  		break;
  	case ETH_LINK_PAUSED:
  		logMessage(TID_ETH "Network paused." ENDL);
  		osSignalWait(SIG_RESUME, osWaitForever);
  		resetNodes();
  		logMessage(TID_ETH "Network resumed." ENDL);
  		break;
  	}
  }
}

void NetworkBroadcast(void const * argument)
{
  static char broadcastNetBuffer[NET_BUFFER_LENGTH] = {0};
  int32_t hBroadcast = -1;
  ssize_t bytesSent;
  uint32_t previousTimeStamp;
  struct sockaddr_in broadcast_addr = {0};
  broadcast_addr.sin_family = AF_INET;
  broadcast_addr.sin_port = htons(BROAD_PORT);
  broadcast_addr.sin_addr.s_addr = IPADDR_BROADCAST;

  ssize_t formattedLength = 0;

  logMessage(TID_BROAD "Network broadcast ready."ENDL);
  while (1)
  {
  	while (hBroadcast < 0)
  	{
  		if (ethernetLinkState != ETH_LINK_UP)
  			osSignalWait(SIG_LINK_UP, osWaitForever);

  		hBroadcast = udpSocket(SOCK_BROADCAST, NULL, 0);
      if (hBroadcast == -1)
      {
      	logMessage(TID_BROAD "Unable to create a socket - ERRNO:%d." ENDL, errno);
      	osDelay(SOCK_ERROR_DELAY);
      }
  	}

  	previousTimeStamp = osKernelSysTick();
  	while (ethernetLinkState == ETH_LINK_UP)
  	{
  		formattedLength = formatNetMessage(NET_MSG_PRESENCE, broadcastNetBuffer, NET_BUFFER_LENGTH-1);
  		bytesSent = sendto(hBroadcast, broadcastNetBuffer, formattedLength, 0, (struct sockaddr *)&broadcast_addr, sizeof(struct sockaddr_in));

  		if (bytesSent <= 0)
  			break;

  		osDelayUntil(&previousTimeStamp, BROAD_SEND_DELAY);
  	}
  	close(hBroadcast);
  	hBroadcast = -1;
  }
}

void NetworkClient(void const * argument)
{
	static char clientNetBuffer[NET_BUFFER_LENGTH] = {0};
	int32_t hSocket = -1;
	int32_t status;
	uint8_t retries;
	struct sockaddr_in addrServer = {0};
	int8_t currentNode = -1;

	addrServer.sin_family = AF_INET;
	addrServer.sin_port = htons(SERVER_LISTEN_PORT);

  logMessage(TID_CLIENT "Client ready." ENDL);
  osMutexWait(nodesMutexHandle, osWaitForever);
  while (1)
  {
  	while (hSocket < 0)
    {
    	if (ethernetLinkState != ETH_LINK_UP)
    	{
    		logMessage(TID_CLIENT "Client paused." ENDL);
    	  osMutexRelease(nodesMutexHandle);
    		osSignalWait(SIG_LINK_UP, osWaitForever);
    		osMutexWait(nodesMutexHandle, osWaitForever);
    		logMessage(TID_CLIENT "Client resumed." ENDL);
    	}

  		currentNode = getNextRegisteredNode(currentNode);
  		if (currentNode == -1)
  		{
  			logMessage(TID_CLIENT "Looped on all %d registered nodes." ENDL, nodeCount);
  			osMutexRelease(nodesMutexHandle);
  			osDelay(CLIENT_LOOP_DELAY);
  			osMutexWait(nodesMutexHandle, osWaitForever);
  			continue;
  		}
  		addrServer.sin_addr.s_addr = getNode(currentNode)->node_addr;

  		logMessage(TID_CLIENT "Attempting to open a socket..." ENDL);
  		hSocket = tcpSocket(SOCK_NONBLOCK, NULL, 0);
  		if (hSocket == -1)
  		{
  			logMessage(TID_CLIENT "Unable to open a TCP socket - errno %03d" ENDL, errno);
  			osDelay(SOCK_ERROR_DELAY);
  		}
    }

    logMessage(TID_CLIENT "Attempting to connect to node %d..." ENDL, currentNode);

    retries = 0;
		while ((retries < CLIENT_MAX_RETRIES) && (ethernetLinkState == ETH_LINK_UP))
		{
			status = connectWithTimeout(hSocket, (struct sockaddr *)&addrServer, sizeof(struct sockaddr_in));
			if (status == 0)
				break;

			retries += 1;
		}

		if ((retries == CLIENT_MAX_RETRIES) || (ethernetLinkState != ETH_LINK_UP))
		{
			logMessage(TID_CLIENT "Unable to connect to node %d." ENDL, currentNode);
		  logMessage(TID_CLIENT "%s" ENDL, removeNode(currentNode)?"Unable to remove node.":"Node removed.");
			close(hSocket);
			hSocket = -1;
			continue;
		}

		logMessage(TID_CLIENT "Connected to node %d - %s:%d." ENDL, currentNode, getNode(currentNode)->node_ipstr, ntohs(addrServer.sin_port));
		formatNetMessage(NET_MSG_DATA, clientNetBuffer, NET_BUFFER_LENGTH);
		status = send(hSocket, clientNetBuffer, strlen(clientNetBuffer), CLIENT_FLAGS);
		if (status < 0)
		{
			logMessage(TID_CLIENT "Unable to send data - errno %03d" ENDL, errno);
			close(hSocket);
			hSocket = -1;
			osDelay(SOCK_ERROR_DELAY);
			continue;
		}
		osDelay(CLIENT_EOC_DELAY);
		close(hSocket);
		hSocket = -1;
  }
}

void NetworkListener(void const * argument)
{
	static char listenerNetBuffer[NET_BUFFER_LENGTH] = {0};
	static char messageType[NET_MSGTYPE_LEN] = {0};
	static char addressString[INET_ADDRSTRLEN] = {0};
	static char nodeId[NODE_ID_LEN] = {0};
  int32_t hListener = -1;
  int32_t status;
  struct sockaddr_in addressListener = {0};
	uint32_t addressInteger;
	struct sockaddr_in addressRemote = {0};
	socklen_t sizeRemote;

	addressListener.sin_family = AF_INET;
	addressListener.sin_port = htons(BROAD_PORT);
	addressListener.sin_addr.s_addr = IPADDR_ANY;

	logMessage(TID_LISTENER "Network listener ready." ENDL);
	while (1)
	{
		while (hListener < 0)
		{
			if (ethernetLinkState != ETH_LINK_UP)
			{
				logMessage(TID_LISTENER "Listener paused." ENDL);
	 			osSignalWait(SIG_LINK_UP, osWaitForever);
	 			logMessage(TID_LISTENER "Listener resumed." ENDL);
			}

			logMessage(TID_LISTENER "Attempting to open a socket..." ENDL);
			hListener = udpSocket(SOCK_BROADCAST | SOCK_NONBLOCK | SOCK_BIND, (struct sockaddr *)&addressListener, sizeof(struct sockaddr_in));
	    if (hListener == -1)
	    {
	      logMessage(TID_BROAD "Unable to create a socket - ERRNO:%d." ENDL, errno);
	      osDelay(SOCK_ERROR_DELAY);
	    }
	  }

		logMessage(TID_LISTENER "Going to listen for broadcast messages..." ENDL);
    while ((ethernetLinkState == ETH_LINK_UP) && (hListener >= 0))
    {
  		status = recvfromWithTimeout(hListener, listenerNetBuffer, NET_BUFFER_LENGTH, 0, (struct sockaddr *)&addressRemote, &sizeRemote);
    	if (status < 0)
    	{
    		logMessage(TID_LISTENER "Unable to receive datagram - errno %03d" ENDL, errno);
        close(hListener);
        hListener = -1;
        break;
    	}

    	if (status == 0)
    		continue;

      if (!jsonGetValue(listenerNetBuffer, "type", messageType, NET_MSGTYPE_LEN))
      {
      	logMessage(TID_LISTENER "Invalid message - field \"type\" not found." ENDL);
      	continue;
      }

			if (strcmp(messageType, "presence") != 0)
			{
				logMessage(TID_LISTENER "Message type not supported: %s" ENDL, messageType);
				continue;
			}

			logMessage(TID_LISTENER "Message received - type: %s" ENDL, messageType);

			if (!jsonGetValue(listenerNetBuffer, "ip", addressString, INET_ADDRSTRLEN))
			{
				logMessage(TID_LISTENER "Invalid message - field \"ip\" not found." ENDL);
				continue;
			}

			if (!jsonGetValue(listenerNetBuffer, "id", nodeId, NODE_ID_LEN))
			{
				logMessage(TID_LISTENER "Invalid message - field \"id\" not found." ENDL);
			  continue;
			}

			if(inet_pton(AF_INET, addressString, &addressInteger) == 0)
			{
				logMessage(TID_LISTENER "Value is not valid IPv4 : %s" ENDL, addressString);
			}

			logMessage(TID_LISTENER "Message originated from %s @ %s" ENDL, nodeId, addressString);
			osMutexWait(nodesMutexHandle, osWaitForever);
			status = registerNode(addressInteger, addressString, nodeId);
			osMutexRelease(nodesMutexHandle);

			switch (status)
      {
			case NODES_ALREADY:
				logMessage(TID_LISTENER "Node already registered." ENDL);
				break;
      case NODES_ERROR:
      	logMessage(TID_LISTENER "Unable to register a node." ENDL);
      	break;
      default:
      	logMessage(TID_LISTENER "Node registered at index %d." ENDL, status);
      }
		}

    if (hListener >= 0)
    {
    	close(hListener);
    	hListener = -1;
    }
	}
}

void NetworkServer(void const * argument)
{
  static char serverNetBuffer[NET_BUFFER_LENGTH] = {0};
  static char valueBuffer[16] = {0};
  static char remoteAddress[INET_ADDRSTRLEN] = {0};
	int32_t hListen = -1;
	int32_t hService = -1;
	int32_t bytesTransceived;
  struct sockaddr_in addrListen = {0};
  struct sockaddr_in addrService = {0};
  uint32_t remoteAddressLen;

  addrListen.sin_family = AF_INET;
  addrListen.sin_port = htons(SERVER_LISTEN_PORT);
  addrListen.sin_addr.s_addr = INADDR_ANY;

	logMessage(TID_SERVER "Server ready." ENDL);
  while (1)
  {
  	while (hListen < 0)
  	{
  		if (ethernetLinkState != ETH_LINK_UP)
  		{
  			logMessage(TID_SERVER "Server paused." ENDL);
  			osSignalWait(SIG_LINK_UP, osWaitForever);
  			logMessage(TID_SERVER "Server resumed." ENDL);
  		}

  		logMessage(TID_SERVER "Attempting to open a socket..." ENDL);
      hListen = tcpSocket(SOCK_NONBLOCK | SOCK_BIND, (struct sockaddr *)&addrListen, sizeof(struct sockaddr_in));
  		if (hListen == -1)
  		{
  			logMessage(TID_SERVER "Unable to open a TCP socket - errno %03d" ENDL, errno);
  			osDelay(SOCK_ERROR_DELAY);
  			continue;
  		}
  		logMessage(TID_SERVER "Socket opened and bound to address " SERVER_LISTEN_ADDR ":%d" ENDL, SERVER_LISTEN_PORT);
  	}

  	while ((hListen >= 0) && (ethernetLinkState == ETH_LINK_UP))
    {
			if (listen(hListen, SERVER_BACKLOG))
			{
				logMessage(TID_SERVER "Unable to listen for connections - errno %03d" ENDL, errno);
				close(hListen);
				hListen = -1;
				osDelay(SOCK_ERROR_DELAY);
				continue;
			}

			logMessage(TID_SERVER "Waiting for remote client to connect..." ENDL);
			while ((hListen >= 0) && (hService < 0) && (ethernetLinkState == ETH_LINK_UP))
			{
				hService = accept(hListen, (struct sockaddr *)&addrService, &remoteAddressLen);
				if ((hService == -1) && ((errno == EWOULDBLOCK) || (errno == EAGAIN)))
				{
					osDelay(SERVER_ACCEPT_POLLING);
					continue;
				}

				if (hService == -1)
				{
					logMessage(TID_SERVER "Unable to accept a connection - errno %03d" ENDL, errno);
					close(hListen);
					hListen = -1;
					osDelay(SOCK_ERROR_DELAY);
					continue;
				}

				if (hService >= 0)
				{
					// At this point inet_ntop cannot fail; no use checking return value.
					(void) inet_ntop(AF_INET, &(addrService.sin_addr.s_addr), (char *)remoteAddress, INET_ADDRSTRLEN);
					logMessage(TID_SERVER "Client connected - %s:%d" ENDL, remoteAddress, ntohs(addrService.sin_port));
				}
			}

			while ((hService >= 0) && (ethernetLinkState == ETH_LINK_UP))
			{
				bytesTransceived = recv(hService, serverNetBuffer, NET_BUFFER_LENGTH, SERVER_FLAGS);
				if (bytesTransceived == -1)
				{
					logMessage(TID_SERVER "Unable to receive data - errno %03d" ENDL, errno);
					close(hService);
					hService = -1;
					osDelay(SOCK_ERROR_DELAY);
					continue;
				}

				if (bytesTransceived == 0)
				{
					logMessage(TID_SERVER "Remote disconnected normally." ENDL);
					close(hService);
					hService = -1;
					continue;
				}

				logMessage(TID_SERVER "Message received : %s" ENDL, serverNetBuffer);
				jsonGetValue(serverNetBuffer, "type", valueBuffer, 16);
				logMessage(TID_SERVER "Message type: %s" ENDL, valueBuffer);
			}

			if (ethernetLinkState != ETH_LINK_UP)
			{
				logMessage(TID_SERVER "Closing the listening socket - link is not up." ENDL);
				close(hListen);
				hListen = -1;
			}
    }
  }
}

int32_t connectWithTimeout(int32_t hSocket, struct sockaddr *remoteAddress, socklen_t socksize)
{
  int32_t status;
  fd_set fdSet;
  struct timeval tv = {.tv_sec = 0, .tv_usec = CLIENT_CONNECT_TIMEOUT};

  status = connect(hSocket, remoteAddress, socksize);
  if (status == 0)
  	return 0;

  if (errno != EINPROGRESS)
  	return -1;

  FD_ZERO(&fdSet);
  FD_SET(hSocket, &fdSet);

  status = select(hSocket+1, NULL, &fdSet, NULL, &tv);
  if (status == 0)
  	return -1;
  if (status < 0)
  	return -1;

  int32_t so_error;
  socklen_t length = sizeof(so_error);
  if (getsockopt(hSocket, SOL_SOCKET, SO_ERROR, &so_error, &length) == -1)
  	return -1;

  if (so_error != 0)
  	return -1;

  return 0;
}

ssize_t formatNetMessage(NetMessageType type, char *messageBuffer, size_t bufferSize)
{
	if (messageBuffer == NULL)
		return -1;

	// TODO: fetch timestamp with external RTC (step-3)
	const char *timestamp = "2025-12-26T12:31:42Z";

	switch (type)
	{
	case NET_MSG_PRESENCE:
		return snprintf(messageBuffer, bufferSize, net_presence_format, timestamp);
	case NET_MSG_DATA:
		return snprintf(messageBuffer, bufferSize, net_data_format, timestamp, 0.13f, -0.20f, 0.97f, "normal");
	default:
		logMessage("<ERROR>  Net message type invalid - 0x%08X" ENDL, type);
		return -1;
	}
}

int32_t genericSocket(
		int32_t domain,
		int32_t type,
		int32_t protocol,
		uint32_t flags,
		struct sockaddr *address,
		socklen_t addressLength)
{
	int32_t hSocket = socket(domain, type, protocol);

	if ((flags & SOCK_NONBLOCK) && (hSocket >= 0))
	  hSocket = setNonBlocking(hSocket);

	if ((flags & SOCK_BROADCAST) && (hSocket >= 0))
	{
	  if (type == SOCK_STREAM)
	  {
	  	close(hSocket);
	    return -1;
	  }

	  int32_t enable = 0x01;
		if (setsockopt(hSocket, SOL_SOCKET, SO_BROADCAST, &enable, sizeof(enable)))
		{
		  close(hSocket);
			return -1;
		}
	}

	if ((flags & SOCK_BIND) && (hSocket >= 0))
	{
		if (bind(hSocket, address, addressLength) == -1)
		{
			close(hSocket);
      return -1;
		}
	}

	return hSocket;
}

/// !!! NEEDS TO BE UNDER MUTEX WAIT !!!
ssize_t getNextRegisteredNode(ssize_t currentNode)
{
  ssize_t result = -1;
	int8_t nextNode = currentNode+1;
  while (nextNode < nodeCount)
  {
  	if (nodeRegister[nextNode].node_addr != 0)
  	{
  		result = nextNode;
  		break;
  	}
  	nextNode++;
  }

  return result;
}

/// !!! NEEDS TO BE UNDER MUTEX WAIT !!!
inline NodeEntry *getNode(uint32_t nodeIndex)
{
  return &nodeRegister[nodeIndex];
}

char *jsonGetValue(const char *__restrict json, const char *__restrict key, char *__restrict value, size_t value_length)
{
  if ((json == NULL) || (key == NULL) || (value == NULL))
    return NULL;

  value[0] = '\0';

  char *match = NULL;
  uint32_t formatted_key_len;
  {
   uint32_t key_len = strlen(key);

    // Using Variable Length Array since malloc() is not available
    formatted_key_len = key_len+3;
    char formatted_key[formatted_key_len];
    snprintf(formatted_key, formatted_key_len, "\"%s\"", key);

    // points to a key, if it exists
    match = strstr(json, formatted_key);
    if (match == NULL)
      return NULL;
  }

  // offset the pointer and search the key-value separator
  match = strchr(match+(formatted_key_len-1), ':');
  if (match == NULL)
	  return NULL;

  // consume all characters that are not part of the value
  do
    match += 1;
  while ((*match == ' ') || (*match == '\t') || (*match == '\n') || (*match == '\r') || (*match == '"'));

  if (*match == '\0')
    return NULL;

  // if value is an object, decapsulate the whole object as a string
  // if value is a primitive type, return the value as a string
  int length = 0;
  if (*match == '{')
  {
    while ((length < value_length) && (match[length] != '\0') && (match[length] != '}'))
      length += 1;

    if (match[length] == '\0')
      return NULL;

    length += 1;
  }
  else
  {
    while ((length < value_length) && (match[length] != '\0') && (match[length] != '"') && (match[length] != ',') && (match[length] != '}'))
      length += 1;

    if (match[length] == '\0')
      return NULL;
  }

  strncpy(value, match, length);
  value[length] = '\0';
  return value;
}

int32_t recvfromWithTimeout(int32_t hSocket, void *buffer, size_t bufferLenght, int32_t flags, struct sockaddr *address, socklen_t *addressLength)
{
  int32_t status;
  fd_set fdSet;
  struct timeval tv = {.tv_sec = 0, .tv_usec = NET_RECVFROM_TIMEOUT};

  FD_ZERO(&fdSet);
  FD_SET(hSocket, &fdSet);

  status = select(hSocket+1, &fdSet, NULL, NULL, &tv);
  if (status < 0)
  	return -1;

  if (status == 0)
  	return 0;

  if (!FD_ISSET(hSocket, &fdSet))
  	return -1;

  status = recvfrom(hSocket, buffer, bufferLenght-1, flags, address, addressLength);
  if (status < 0)
  	return -1;

  ((char *)buffer)[status] = '\0';
  return status;
}

/// !!! NEEDS TO BE UNDER MUTEX WAIT !!!
/// return value: NODES_ERROR   - unable to register node (no space left)
///               NODES_ALREADY - node already registered
///               >= 0          - index of registered node
int32_t registerNode(uint32_t address, const char *__restrict ipstr, const char *__restrict id)
{
  int8_t freeNode = -1;
  ssize_t nodeIndex = 0;

	if (nodeCount+1 >= NET_MAX_NODES)
		return NODES_ERROR;

  while (nodeIndex < nodeCount+1)
  {
  	// Node already in register.
  	if (getNode(nodeIndex)->node_addr == address)
  		return NODES_ALREADY;

  	if ((freeNode == -1) && (getNode(nodeIndex)->node_addr == 0))
  		freeNode = nodeIndex;

    nodeIndex += 1;
  }

  // No space found (should not happen here)
  if (freeNode == -1)
  {
    return -1;
  }

  nodeCount += 1;
  nodeRegister[freeNode].node_addr = address;
  strncpy(nodeRegister[freeNode].node_ipstr, ipstr, INET_ADDRSTRLEN);
  strncpy(nodeRegister[freeNode].node_id, id, NODE_ID_LEN);
  return freeNode;
}

/// !!! NEEDS TO BE UNDER MUTEX WAIT !!!
/// return value: NODES_ERROR - unable to remove node
///               NODES_ALREADY - node already removed
///               NODES_SUCCESS - node removed successfully
int32_t removeNode(uint32_t idx)
{
	if (idx >= nodeCount)
	  return NODES_ERROR;

	if (nodeRegister[idx].node_addr == 0)
		return NODES_ALREADY;

	int32_t neighbour = idx+1;
	while (neighbour < nodeCount)
	{
    getNode(idx)->node_addr = getNode(neighbour)->node_addr;
    strncpy(getNode(idx)->node_ipstr, getNode(neighbour)->node_ipstr, INET_ADDRSTRLEN);
    strncpy(getNode(idx)->node_id, getNode(neighbour)->node_id, NODE_ID_LEN);
    idx += 1;
    neighbour += 1;
	}

	getNode(idx)->node_addr = 0;
	getNode(idx)->node_ipstr[0] = '\0';
	getNode(idx)->node_id[0] = '\0';

	nodeCount -= 1;
	return NODES_SUCCESS;
}

/// !!! NEEDS TO BE UNDER MUTEX WAIT !!!
/// return value:  NODES_ERROR   - node count is invalid (should not happen)
///                NODES_ALREADY - nodes already cleared
///                NODES_SUCCESS - nodes successfully cleared
int32_t resetNodes(void)
{
  if (nodeCount < 0)
  	return NODES_ERROR;

  if (nodeCount == 0)
  	return NODES_ALREADY;

  nodeCount = 0;
  memset(nodeRegister, 0, sizeof(NodeEntry)*NET_MAX_NODES);
  return NODES_SUCCESS;
}

inline int32_t tcpSocket(uint32_t flags, struct sockaddr *address, socklen_t addressLength)
{
	return genericSocket(AF_INET, SOCK_STREAM, IPPROTO_TCP, flags, address, addressLength);
}

inline int32_t udpSocket(uint32_t flags, struct sockaddr *address, socklen_t addressLength)
{
	return genericSocket(AF_INET, SOCK_DGRAM, IPPROTO_UDP, flags, address, addressLength);
}

int32_t setNonBlocking(int32_t hSocket)
{
	int32_t  socketFlags;
	socketFlags = fcntl(hSocket, F_GETFL, 0);
	if (socketFlags == -1)
	{
		close(hSocket);
		return -1;
	}

	socketFlags |= O_NONBLOCK;
	if (fcntl(hSocket, F_SETFL, socketFlags) == -1)
	{
		close(hSocket);
    return -1;
	}

	return hSocket;
}
