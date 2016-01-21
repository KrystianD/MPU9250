#ifndef __MPUDMP_INTERNAL_H__
#define __MPUDMP_INTERNAL_H__

#include <stdio.h>
#include <inttypes.h>

#ifdef DEBUG
#define DEBUG_PRINT(x) printf(x)
#define DEBUG_PRINTi(x) printf("%d",x)
#define DEBUG_PRINTF(x, y) printf(x, y)
#define DEBUG_PRINTLN(x,...) printf(x "\r\n", ##__VA_ARGS__);
#define DEBUG_PRINTLNi(x) printf("%d\r\n",x);
#define DEBUG_PRINTLNF(x, y) printf("%d\r\n",x);
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTi(x)
#define DEBUG_PRINTF(x, y)
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINTLNi(x)
#define DEBUG_PRINTLNF(x, y)
#endif

extern uint8_t *dmpPacketBuffer;
extern uint16_t dmpPacketSize;

static int min(int a, int b)
{
	return a < b ? a : b;
}

#endif
