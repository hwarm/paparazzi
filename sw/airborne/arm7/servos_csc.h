#ifndef SERVOS_CSC_H
#define SERVOS_CSC_H

#include "LPC21xx.h"
#include "airframe.h"
#include "actuators.h"
#include "sys_time.h"
#include "csc_ap_link.h"
#include "csc_msg_def.h"

#define SERVOS_TICS_OF_USEC(s) SYS_TICS_OF_USEC(s)
#define ChopServo(x,a,b) Chop(x, a, b)
#define SERVO_COUNT 4

#define Actuator(i) actuators[i]

static inline void ActuatorsCommit(void)
{
  csc_ap_send_msg(CSC_SERVO_CMD_ID, (uint8_t *)&actuators, sizeof(struct CscServoCmd));
}

#endif /* SERVOS_CSC_H */