#ifndef __RING_BUFFER_H
#define __RING_BUFFER_H

#include "sys.h"
#include <stdbool.h>

#define RING_BUFFER_SIZE 256

typedef struct {
    uint8_t buffer[RING_BUFFER_SIZE];
    uint16_t pOft;  // read offset (pop pointer)
    uint16_t pWrt;  // write offset (push pointer)
} RingBuffer_TypeDef;

void RingBuffer_Init(RingBuffer_TypeDef *rb);
bool RingBuffer_Pop(RingBuffer_TypeDef *rb, uint8_t *data);
void RingBuffer_Push(RingBuffer_TypeDef *rb, uint8_t data);
uint16_t RingBuffer_GetByteUsed(RingBuffer_TypeDef *rb);
uint8_t RingBuffer_Peek(RingBuffer_TypeDef *rb, uint16_t offset);
bool RingBuffer_IsEmpty(RingBuffer_TypeDef *rb);

#endif
