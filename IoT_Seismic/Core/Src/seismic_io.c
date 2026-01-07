#include <math.h>
#include "seismic_log.h"
#include "seismic_io.h"

#define ACCEL_FSR                (3330)
#define ACCEL_SENSITIVITY        (333)
#define ACCEL_X_BIAS             (1670)
#define ACCEL_Y_BIAS             (1655)
#define ACCEL_Z_BIAS             (1715)
#define ADC_MAX_VALUE            (4095)

static TIM_HandleTypeDef *__htimAccelerometer;
static TIM_HandleTypeDef *__htimHeartbeat;

osSemaphoreId accelerometerSemHandle;
osThreadId accelerometerHandle;
osThreadId heartbeatHandle;

void initIOs(TIM_HandleTypeDef *htimHeartbeat, TIM_HandleTypeDef *htimAccelerometer)
{
	__htimHeartbeat = htimHeartbeat;
	__htimAccelerometer = htimAccelerometer;
  osSemaphoreDef(accelerometerSem);
  accelerometerSemHandle = osSemaphoreCreate(osSemaphore(accelerometerSem), 1);

  osThreadDef(heartbeat, Heartbeat, osPriorityNormal, 0, 128);
  heartbeatHandle = osThreadCreate(osThread(heartbeat), NULL);

  osThreadDef(accelerometer, Accelerometer, osPriorityNormal, 0, 256);
  accelerometerHandle = osThreadCreate(osThread(accelerometer), NULL);

  if (tasksState == SYS_TASKS_PAUSED)
  {
  	// take the binary semaphore to block the task after initialisation.
  	osSemaphoreWait(accelerometerSemHandle, osWaitForever);
  }
}

void Accelerometer(void const * argument)
{
	uint32_t sumX = 0;
	uint32_t sumY = 0;
	uint32_t sumZ = 0;
	float rmsX;
	float rmsY;
	float rmsZ;
	uint8_t bufferIndex = 0;

	logMessage(TID_ACCEL "Accelerometer ready." ENDL);
	while (1)
	{
		osSemaphoreWait(accelerometerSemHandle, osWaitForever);

		if (tasksState == SYS_TASKS_PAUSED)
		{
			HAL_TIM_Base_Stop_IT(__htimAccelerometer);
			logMessage(TID_ACCEL "Task suspended." ENDL);
			continue;
		}

		// Only restart the timer once after resume.
		if (__htimAccelerometer->State == HAL_TIM_STATE_READY)
		{
			HAL_TIM_Base_Start_IT(__htimAccelerometer);
			logMessage(TID_ACCEL "Task resumed." ENDL);
			// Wait until the next interrupt, DMA transfer may not be done after resume.
			osSemaphoreWait(accelerometerSemHandle, osWaitForever);
		}

		while (bufferIndex < 10)
		{
			sumX += accelDmaBuffer[ACCEL_AXES * bufferIndex] * accelDmaBuffer[ACCEL_AXES * bufferIndex];
			sumY += accelDmaBuffer[ACCEL_AXES * bufferIndex + 1] * accelDmaBuffer[ACCEL_AXES * bufferIndex + 1];
			sumZ += accelDmaBuffer[ACCEL_AXES * bufferIndex + 2] * accelDmaBuffer[ACCEL_AXES * bufferIndex + 1];
			bufferIndex++;
		}
		bufferIndex = 0;

		rmsX = (float)sqrt((double)sumX / ACCEL_SAMPLES_PER_AXIS);
		rmsY = (float)sqrt((double)sumY / ACCEL_SAMPLES_PER_AXIS);
		rmsZ = (float)sqrt((double)sumZ / ACCEL_SAMPLES_PER_AXIS);
		logMessage(TID_ACCEL "x=%6.2f, y=%6.2f, z=%6.2f" ENDL, rmsX, rmsY, rmsZ);
		sumX = sumY = sumZ = 0;
	}
}

void Heartbeat(void const * argument)
{
	logMessage(TID_HEART "Heartbeat ready." ENDL);
	while (1)
	{
		osSignalWait(SIG_RESUME, osWaitForever);
		HAL_TIM_OC_Start(__htimHeartbeat, TIM_CHANNEL_2);
		logMessage(TID_HEART "Task resumed." ENDL);

		osSignalWait(SIG_PAUSE, osWaitForever);
		HAL_TIM_OC_Stop(__htimHeartbeat, TIM_CHANNEL_2);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
		logMessage(TID_HEART "Task suspended." ENDL);
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	osSemaphoreRelease(accelerometerSemHandle);
}

