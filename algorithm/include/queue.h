#pragma once

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "CAN_info.h"

/*
 * By current Queue definition it should be smaller than 255.
 * Also note that it can store elements with size that divide this buffer.
 * For example for 128: 2, 4, 8, ...
 */
#define QUEUE_DATA_SIZE_IN_BYTES 128

typedef struct Queue_s
{
    u8 end;
    u8 start;
    u8 elementSize;
    u8 size;
    u8 data[QUEUE_DATA_SIZE_IN_BYTES];
} Queue_s;

static inline void queueInit(Queue_s *queue_p, const u8 elementSize)
{
    queue_p->start = 0;
    queue_p->end = 0;
    queue_p->size = 0;
    queue_p->elementSize = elementSize;
}

static inline bool isFull(const Queue_s *queue_p)
{
    u8 nextEnd = (queue_p->end + queue_p->elementSize) % QUEUE_DATA_SIZE_IN_BYTES;
    return nextEnd == queue_p->start;
}

static inline bool isEmpty(const Queue_s *queue_p)
{
    return queue_p->start == queue_p->end;
}

static inline void pop(Queue_s *queue_p, void *dest_p)
{
    const u8 startIdx = queue_p->start;
    u8 *data_p = queue_p->data;
    const u8 *start_p = data_p + startIdx;

    if (isEmpty(queue_p))
    {
        fprintf(stderr, "Not enough data in queue!\n");
        exit(1);
    }

    memcpy(dest_p, start_p, queue_p->elementSize);

    queue_p->start = (startIdx + queue_p->elementSize) % QUEUE_DATA_SIZE_IN_BYTES;
    queue_p->size--;
}

static inline void push(Queue_s *queue_p, const void *element_p)
{
    u8 endIdx = queue_p->end;
    u8 *data_p = queue_p->data;
    u8 *end_p = data_p + endIdx;

    if (isFull(queue_p))
    {
        fprintf(stderr, "Queue overflow\n");
        assert(!isFull(queue_p));
        exit(1);
    }

    memcpy(end_p, element_p, queue_p->elementSize);

    queue_p->end = (endIdx + queue_p->elementSize) % QUEUE_DATA_SIZE_IN_BYTES;
    queue_p->size++;
}

static inline void peek(Queue_s *queue_p, void *dest_p)
{
    const u8 startIdx = queue_p->start;
    u8 *data_p = queue_p->data;
    const u8 *start_p = data_p + startIdx;

    if (isEmpty(queue_p))
    {
        fprintf(stderr, "Not enough data in queue!\n");
        exit(1);
    }

    memcpy(dest_p, start_p, queue_p->elementSize);
}
