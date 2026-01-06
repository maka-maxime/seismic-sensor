#ifndef __SEISMIC_H__
#define __SEISMIC_H__

#include "stm32f7xx_hal.h"
#include "cmsis_os.h"

#define ENDL                     "\r\n"
#define SIG_BUTTON               (0x00000001)
#define SIG_LWIP                 (0x00000002)
#define SIG_LINK_UP              (0x00000004)
#define SIG_PAUSE                (0x00000010)
#define SIG_RESUME               (0x00000100)
#define SYS_TASKS_RUNNING        (0x00)
#define SYS_TASKS_PAUSED         (0x01)
#define NODE_ID                  "nucleo-3"
#define NODE_IP                  "192.168.1.183"
#define TID_ACCEL                "<ACCEL>  "
#define TID_BROAD                "<BROAD>  "
#define TID_CLIENT               "<CLIENT> "
#define TID_DEBUG                "<MEMORY> "
#define TID_ETH                  "<ETHNET> "
#define TID_HEART                "<HEART>  "
#define TID_SERVER               "<SERVER> "
#define TID_SYS                  "<SYSTEM> "

#endif /* __SEISMIC_H__ */
