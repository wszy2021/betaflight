#pragma once

#include "follow/follow_config.h"

typedef struct {
    uint8_t *buffer;
    uint32_t size;
    uint32_t in;
    uint32_t out;
    uint32_t mask;
} kfifo_t;

static inline bool kfifo_init(kfifo_t *fifo, uint8_t *buffer, uint32_t size)
{
    if (!fifo || !buffer || size < MIN_KFIFO_SIZE || size > MAX_KFIFO_SIZE || !IS_POWER_OF_2(size)) {
        return false;
    }

    fifo->buffer = buffer;
    fifo->size = size;
    fifo->in = 0;
    fifo->out = 0;
    fifo->mask = KFIFO_MASK(size);
    return true;
}

static inline void kfifo_reset(kfifo_t *fifo)
{
    if (!fifo) {
        return;
    }

    ATOMIC_BLOCK(KFIFO_ATOMIC_PRIO) {
        fifo->in = fifo->out = 0;
    }
}

static inline uint32_t kfifo_input(const kfifo_t *fifo)
{
    return fifo ? fifo->in : 0;
}

static inline uint32_t kfifo_output(const kfifo_t *fifo)
{
    return fifo ? fifo->out : 0;
}

static inline uint32_t kfifo_len(const kfifo_t *fifo)
{
    uint32_t in;
    uint32_t out;

    if (!fifo) {
        return 0;
    }

    ATOMIC_BLOCK(KFIFO_ATOMIC_PRIO) {
        in = fifo->in;
        out = fifo->out;
    }

    return in - out;
}

static inline uint32_t kfifo_avail(const kfifo_t *fifo)
{
    if (!fifo) {
        return 0;
    }

    return fifo->size - kfifo_len(fifo);
}

static inline bool kfifo_is_empty(const kfifo_t *fifo)
{
    return !fifo || fifo->in == fifo->out;
}

static inline bool kfifo_is_full(const kfifo_t *fifo)
{
    return !fifo || kfifo_len(fifo) == fifo->size;
}

static inline uint32_t kfifo_in(kfifo_t *fifo, const uint8_t *buffer, uint32_t len)
{
    uint32_t first;
    uint32_t offset;
    uint32_t size;

    if (!fifo || !buffer || !len) {
        return 0;
    }

    size = kfifo_avail(fifo);
    len = len > size ? size : len;
    if (!len) {
        return 0;
    }

    offset = fifo->in & fifo->mask;
    first = fifo->size - offset;

    if (len <= first) {
        memcpy(fifo->buffer + offset, buffer, len);
    } else {
        memcpy(fifo->buffer + offset, buffer, first);
        memcpy(fifo->buffer, buffer + first, len - first);
    }

    ATOMIC_BLOCK(KFIFO_ATOMIC_PRIO) {
        fifo->in += len;
    }

    return len;
}

static inline uint32_t kfifo_out(kfifo_t *fifo, uint8_t *buffer, uint32_t len)
{
    uint32_t first;
    uint32_t offset;
    uint32_t size;

    if (!fifo || !buffer || !len) {
        return 0;
    }

    size = kfifo_len(fifo);
    len = len > size ? size : len;
    if (!len) {
        return 0;
    }

    offset = fifo->out & fifo->mask;
    first = fifo->size - offset;

    if (len <= first) {
        memcpy(buffer, fifo->buffer + offset, len);
    } else {
        memcpy(buffer, fifo->buffer + offset, first);
        memcpy(buffer + first, fifo->buffer, len - first);
    }

    ATOMIC_BLOCK(KFIFO_ATOMIC_PRIO) {
        fifo->out += len;
    }

    return len;
}

static inline uint32_t kfifo_peek(const kfifo_t *fifo, uint8_t *buffer, uint32_t len)
{
    uint32_t first;
    uint32_t offset;
    uint32_t size;

    if (!fifo || !buffer || !len) {
        return 0;
    }

    size = kfifo_len(fifo);
    len = len > size ? size : len;
    if (!len) {
        return 0;
    }

    offset = fifo->out & fifo->mask;
    first = fifo->size - offset;

    if (len <= first) {
        memcpy(buffer, fifo->buffer + offset, len);
    } else {
        memcpy(buffer, fifo->buffer + offset, first);
        memcpy(buffer + first, fifo->buffer, len - first);
    }

    return len;
}

static inline uint32_t kfifo_skip(kfifo_t *fifo, uint32_t len)
{
    uint32_t size;

    if (!fifo || !len) {
        return 0;
    }

    size = kfifo_len(fifo);
    len = len > size ? size : len;
    if (!len) {
        return 0;
    }

    ATOMIC_BLOCK(KFIFO_ATOMIC_PRIO) {
        fifo->out += len;
    }

    return len;
}
