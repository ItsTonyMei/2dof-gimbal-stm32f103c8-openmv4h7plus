#include "ring_buffer.h"

void RingBuffer_Init(RingBuffer_TypeDef *rb)
{
    if (rb == NULL) {
        return;
    }
    rb->pOft = 0;
    rb->pWrt = 0;
}

bool RingBuffer_Pop(RingBuffer_TypeDef *rb, uint8_t *data)
{
    if (rb == NULL || data == NULL) {
        return false;
    }
    if (rb->pOft == rb->pWrt) {
        return false;  // buffer empty
    }
    *data = rb->buffer[rb->pOft];
    rb->pOft = (rb->pOft + 1) % RING_BUFFER_SIZE;
    return true;
}

void RingBuffer_Push(RingBuffer_TypeDef *rb, uint8_t data)
{
    if (rb == NULL) {
        return;
    }
    rb->buffer[rb->pWrt] = data;
    rb->pWrt = (rb->pWrt + 1) % RING_BUFFER_SIZE;
    // If write pointer catches up to read pointer, discard oldest data
    if (rb->pWrt == rb->pOft) {
        rb->pOft = (rb->pOft + 1) % RING_BUFFER_SIZE;
    }
}

uint16_t RingBuffer_GetByteUsed(RingBuffer_TypeDef *rb)
{
    if (rb == NULL) {
        return 0;
    }
    if (rb->pWrt >= rb->pOft) {
        return rb->pWrt - rb->pOft;
    } else {
        return RING_BUFFER_SIZE - rb->pOft + rb->pWrt;
    }
}

uint8_t RingBuffer_Peek(RingBuffer_TypeDef *rb, uint16_t offset)
{
    uint16_t idx;
    if (rb == NULL) {
        return 0;
    }
    if (offset >= RingBuffer_GetByteUsed(rb)) {
        return 0;
    }
    idx = (rb->pOft + offset) % RING_BUFFER_SIZE;
    return rb->buffer[idx];
}

bool RingBuffer_IsEmpty(RingBuffer_TypeDef *rb)
{
    if (rb == NULL) {
        return true;
    }
    return rb->pOft == rb->pWrt;
}
