#ifndef __SEISMIC_LOG_H__
#define __SEISMIC_LOG_H__

#include "seismic.h"

#define MESSAGE_TEXT_MAXLEN      (192UL)
#define MAILQ_LENGTH             (0x10)

typedef struct message
{
	uint32_t length;
	char text[MESSAGE_TEXT_MAXLEN];
} Message;

extern osThreadId loggerHandle;
extern osMailQId mailQueueHandle;
extern osMutexId uartMutexHandle;

void initLogger(UART_HandleTypeDef *huart);
void Logger(void const * argument);

osStatus logMessage(const char *__restrict format, ...);

#endif /* __SEISMIC_LOG_H__ */
