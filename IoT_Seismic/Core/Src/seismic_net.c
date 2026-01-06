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
#define CLIENT_MAX_REMOTES       (20)
#define CLIENT_MAX_RETRIES       (4)
#define CLIENT_NEXT_DELAY        (500)
#define CLIENT_POLL_REMOTES      (2000)
#define CLIENT_REMOTE_ADDRESS    "192.168.1.174"
#define ETH_LINK_DOWN            (0)
#define ETH_LINK_PAUSED          (2)
#define ETH_LINK_POLLING_DELAY   (200)
#define ETH_LINK_STARTUP_DELAY   (2000)
#define ETH_LINK_UP              (1)
#define NET_BUFFER_LENGTH        (144)
#define NET_RECVFROM_TIMEOUT     (500000) // microseconds
#define SERVER_ACCEPT_POLLING    (200)
#define SERVER_BACKLOG           (1)
#define SERVER_FLAGS             (0)
#define SERVER_LISTEN_ADDR       "0.0.0.0"
#define SERVER_LISTEN_PORT       (12345)
#define SOCK_BROADCAST           (0x00000002)
#define SOCK_BIND                (0x00000004)
#define SOCK_ERROR_DELAY         (1000)
#define SOCK_NONBLOCK            (0x00000001)

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
static uint8_t remoteCount = 0;
static ip4_addr_t remoteNodes[CLIENT_MAX_REMOTES] = {0};

static int32_t connectWithTimeout(int32_t hSocket, struct sockaddr *remoteAddress, socklen_t socksize);
static ssize_t formatNetMessage(NetMessageType type, char *messageBuffer, size_t bufferSize);
static int8_t getNextRegisteredRemote(int8_t currentRemote);
static char *jsonGetValue(const char *__restrict json, const char *__restrict key, char *__restrict value, size_t value_length);
static int32_t genericSocket(int32_t domain, int32_t type, int32_t protocol, uint32_t flags, struct sockaddr *address, socklen_t addressLength);
static int32_t recvfromWithTimeout(int32_t hSocket, void *buffer, size_t bufferLenght, int32_t flags, struct sockaddr *address, socklen_t *addressLength);
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
  inet_pton(AF_INET, CLIENT_REMOTE_ADDRESS, &remoteNodes[0]);
  remoteCount = 1;

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
  		logMessage(TID_ETH "Network resumed." ENDL);
  		break;
  	}
  }
}

void NetworkBroadcast(void const * argument)
{
  int32_t hBroadcast = -1;
  ssize_t bytesSent;
  uint32_t previousTimeStamp;
  struct sockaddr_in broadcast_addr = {0};
  broadcast_addr.sin_family = AF_INET;
  broadcast_addr.sin_port = htons(BROAD_PORT);
  broadcast_addr.sin_addr.s_addr = IPADDR_BROADCAST;

  char payload[NET_BUFFER_LENGTH];
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
  		formattedLength = formatNetMessage(NET_MSG_PRESENCE, payload, NET_BUFFER_LENGTH-1);
  		bytesSent = sendto(hBroadcast, payload, formattedLength, 0, (struct sockaddr *)&broadcast_addr, sizeof(struct sockaddr_in));

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
	int32_t hSocket = -1;
	int32_t status;
	uint8_t retries;
	struct sockaddr_in addrServer = {0};
	char remoteAddress[INET_ADDRSTRLEN] = {0};
	char txBuffer[NET_BUFFER_LENGTH] = {0};
	int8_t currentRemote = -1;

	addrServer.sin_family = AF_INET;
	addrServer.sin_port = htons(SERVER_LISTEN_PORT);

  logMessage(TID_CLIENT "Client ready." ENDL);
  while (1)
  {
  	while (hSocket < 0)
    {
    	if (ethernetLinkState != ETH_LINK_UP)
    	{
    		logMessage(TID_CLIENT "Client paused." ENDL);
    		osSignalWait(SIG_LINK_UP, osWaitForever);
    		logMessage(TID_CLIENT "Client resumed." ENDL);
    	}

  		currentRemote = getNextRegisteredRemote(currentRemote);
  		if (currentRemote == -1)
  		{
  			logMessage(TID_CLIENT "Looped on all registered nodes." ENDL);
  			osDelay(CLIENT_LOOP_DELAY);
  			continue;
  		}
  		addrServer.sin_addr.s_addr = remoteNodes[currentRemote].addr;

  		logMessage(TID_CLIENT "Attempting to open a socket..." ENDL);
  		hSocket = tcpSocket(SOCK_NONBLOCK, NULL, 0);
  		if (hSocket == -1)
  		{
  			logMessage(TID_CLIENT "Unable to open a TCP socket - errno %03d" ENDL, errno);
  			osDelay(SOCK_ERROR_DELAY);
  		}
    }

    logMessage(TID_CLIENT "Attempting to connect to node %d/%d..." ENDL, currentRemote+1, remoteCount);

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
			logMessage(TID_CLIENT "Unable to connect to node %d/%d." ENDL, currentRemote+1, remoteCount);
			close(hSocket);
			hSocket = -1;
			continue;
		}

		inet_ntop(AF_INET, &addrServer.sin_addr, remoteAddress, INET_ADDRSTRLEN);
		logMessage(TID_CLIENT "Connected to node %d/%d - %s:%d." ENDL, currentRemote+1, remoteCount, remoteAddress, ntohs(addrServer.sin_port));
		formatNetMessage(NET_MSG_DATA, txBuffer, NET_BUFFER_LENGTH);
		status = send(hSocket, txBuffer, strlen(txBuffer), CLIENT_FLAGS);
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
		osDelay(CLIENT_NEXT_DELAY);
  }
}

void NetworkListener(void const * argument)
{
  int32_t hListener = -1;
  int32_t status;
  struct sockaddr_in addressListener = {0};
	char listenerNetBuffer[NET_BUFFER_LENGTH] = {0};
	char valueBuffer[16] = {0};
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

    while ((ethernetLinkState == ETH_LINK_UP) && (hListener >= 0))
    {
  		//logMessage(TID_LISTENER "Waiting for a message with a timeout..." ENDL);
    	status = recvfromWithTimeout(hListener, listenerNetBuffer, NET_BUFFER_LENGTH, 0, (struct sockaddr *)&addressRemote, &sizeRemote);
    	if (status < 0)
    	{
    		logMessage(TID_LISTENER "Unable to receive datagram - errno %03d" ENDL, errno);
    		close(hListener);
    		hListener = -1;
    		continue;
    	}

    	if (status == 0)
    	//{
    		//logMessage(TID_LISTENER "Timeout." ENDL);
    		continue;
    	//}

      logMessage(TID_LISTENER "Message received." ENDL);
      jsonGetValue(listenerNetBuffer, "type", valueBuffer, 16);
			logMessage(TID_LISTENER "Message type: %s" ENDL, valueBuffer);
    }
	}
}

void NetworkServer(void const * argument)
{
	int32_t hListen = -1;
	int32_t hService = -1;
	int32_t bytesTransceived;
  struct sockaddr_in addrListen = {0};
  struct sockaddr_in addrService = {0};
  uint8_t remoteAddress[INET_ADDRSTRLEN] = {0};
  uint32_t remoteAddressLen;
  char ioBuffer[NET_BUFFER_LENGTH] = {0};
  char valueBuffer[16] = {0};

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
				bytesTransceived = recv(hService, ioBuffer, NET_BUFFER_LENGTH, SERVER_FLAGS);
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

				logMessage(TID_SERVER "Message received : %s" ENDL, ioBuffer);
				jsonGetValue(ioBuffer, "type", valueBuffer, 16);
				logMessage(TID_SERVER "Message type: %s" ENDL, valueBuffer);

				strncpy((char *)ioBuffer, "ACK", NET_BUFFER_LENGTH);
				bytesTransceived = send(hService, ioBuffer, 3, 0);
				if (bytesTransceived == -1)
				{
					logMessage(TID_SERVER "Unable to send data - errno %03d" ENDL, errno);
					close(hService);
					hService = -1;
					osDelay(SOCK_ERROR_DELAY);
					continue;
				}
				logMessage(TID_SERVER "Data sent successfully." ENDL);
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

int8_t getNextRegisteredRemote(int8_t currentRemote)
{
	int8_t nextRemote = currentRemote+1;
  while (nextRemote < remoteCount)
  {
  	if (remoteNodes[nextRemote].addr != 0)
  		return nextRemote;
  	nextRemote++;
  }

  return -1;
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

  return strncpy(value, match, length);
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
