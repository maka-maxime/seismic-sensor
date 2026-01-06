#ifndef __SEISMIC_IO_H__
#define __SEISMIC_IO_H__

#include "seismic.h"

#define ACCEL_AXES               (3)
#define ACCEL_SAMPLES_PER_AXIS   (10)
#define ACCEL_SAMPLES            (ACCEL_AXES * ACCEL_SAMPLES_PER_AXIS)

extern osSemaphoreId accelerometerSemHandle;
extern osThreadId accelerometerHandle;
extern osThreadId heartbeatHandle;

extern uint16_t accelDmaBuffer[ACCEL_SAMPLES];
extern uint8_t tasksState;

void initIOs(TIM_HandleTypeDef *htimHeartbeat, TIM_HandleTypeDef *htimAccelerometer);
void Accelerometer(void const * argument);
void Heartbeat(void const * argument);

#endif /* __SEISMIC_IO_H__ */
