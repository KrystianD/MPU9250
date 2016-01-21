#ifndef __MPUDMP_INTERNAL_H__
#define __MPUDMP_INTERNAL_H__

#include <stdio.h>
#include <inttypes.h>

#ifdef MPU9250_DEBUG
#define DEBUG_PRINT(x,...) printf(x, ##__VA_ARGS__);
#define DEBUG_PRINTLN(x,...) printf(x "\r\n", ##__VA_ARGS__);
#else
#define DEBUG_PRINT(x,...)
#define DEBUG_PRINTLN(x,...)
#endif

extern uint8_t *dmpPacketBuffer;
extern uint16_t dmpPacketSize;

#endif
