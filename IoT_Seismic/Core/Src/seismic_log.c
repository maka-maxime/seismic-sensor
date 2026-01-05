#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#include "seismic_log.h"

#define MAILQ_GET_TIMEOUT (osWaitForever)
#define UART_TIMEOUT (100)

static UART_HandleTypeDef *__huart = NULL;

osThreadId loggerHandle;
osMailQId mailQueueHandle;
osMutexId uartMutexHandle;

void initLogger(UART_HandleTypeDef *huart)
{
  __huart = huart;

  osMutexDef(uartMutex);
  uartMutexHandle = osMutexCreate(osMutex(uartMutex));

  osMailQDef(mailQueue, MAILQ_LENGTH, Message);
  mailQueueHandle = osMailCreate(osMailQ(mailQueue), NULL);

  osThreadDef(logger, Logger, osPriorityNormal, 0, 256);
  loggerHandle = osThreadCreate(osThread(logger), NULL);
}

void Logger(void const * argument)
{
	osEvent event = {0x00};
	Message *message;
	char timestamp[16];
	uint32_t ticks;
	div_t div_result;
	while (1)
	{
		event = osMailGet(mailQueueHandle, MAILQ_GET_TIMEOUT);
		if (osEventMail != event.status)
			continue;

		ticks = osKernelSysTick();
		div_result = div(ticks, osKernelSysTickFrequency);
		snprintf(timestamp, 16, "[%8d.%03d] ", div_result.quot, div_result.rem);
		message = (Message *)event.value.p;
		osMutexWait(uartMutexHandle, osWaitForever);
		HAL_UART_Transmit(__huart, (uint8_t*)timestamp, 16, UART_TIMEOUT);
		HAL_UART_Transmit(__huart, (uint8_t*)message->text, message->length, UART_TIMEOUT);
		osMutexRelease(uartMutexHandle);
		osMailFree(mailQueueHandle, event.value.p);
	}
}

osStatus logMessage(const char *__restrict format, ...)
{
	// osMailAlloc() is never blocking, millisec is discarded in v1 implementation
	Message *message = (Message *)osMailAlloc(mailQueueHandle, 0);
	if (message == NULL)
		return osErrorNoMemory;

	va_list args;

	va_start(args, format);
	vsnprintf(message->text, MESSAGE_TEXT_MAXLEN, format, args);
	va_end(args);

	message->length = strlen(message->text);

	return osMailPut(mailQueueHandle, message);
}
