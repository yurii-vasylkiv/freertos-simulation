#ifndef AVIONICS_MOVING_BUFFER_H
#define AVIONICS_MOVING_BUFFER_H

#include <inttypes.h>
#include <assert.h>
#include <memory.h>

#ifdef __cplusplus
extern "C" {
#endif

#if (userconf_USE_COTS_DATA == 1)
#define MOVING_BUFFER_RANGE    100
#else
#define MOVING_BUFFER_RANGE    10
#endif

#define REAL_BUFFER_CAPACITY    MOVING_BUFFER_RANGE + 1
#define MOVING_BUFFER_DATA_TYPE float

typedef struct
{
    MOVING_BUFFER_DATA_TYPE buffer [REAL_BUFFER_CAPACITY];          // data buffer
    size_t head;                                                    // pointer to the head
    size_t capacity;                                                // maximum number of items in the buffer
    size_t sz;                                                      // size of each item in the buffer
    MOVING_BUFFER_DATA_TYPE *head_ptr;                              // pointer to head
    int first_lap;
    MOVING_BUFFER_DATA_TYPE linear_repr [REAL_BUFFER_CAPACITY];     // linear buffer
} moving_data_buffer;

static inline void moving_buffer_init      ( moving_data_buffer *queue )
{
    assert(queue != NULL);

    queue->capacity = REAL_BUFFER_CAPACITY;
    queue->sz = sizeof ( MOVING_BUFFER_DATA_TYPE );
    memset( queue->buffer, 0, queue->sz * queue->capacity);
    queue->head = 0;
    queue->first_lap = 1;
}

void moving_buffer_linearize( moving_data_buffer * queue)
{
    assert (queue  != NULL);

    memset(queue->linear_repr, 0, sizeof( MOVING_BUFFER_DATA_TYPE ) * queue->capacity);

    if(queue->first_lap)
    {
        memcpy(&queue->linear_repr[0], & queue->buffer[0], queue->head * sizeof(MOVING_BUFFER_DATA_TYPE));
    }
    else
    {
        memcpy(&queue->linear_repr[0], &queue->buffer[queue->head], (queue->capacity - queue->head) * sizeof(MOVING_BUFFER_DATA_TYPE));
        memcpy(&queue->linear_repr[queue->capacity - queue->head], & queue->buffer[0], queue->head* sizeof(MOVING_BUFFER_DATA_TYPE));
    }
}

static inline void moving_buffer_insert    ( moving_data_buffer  *queue, MOVING_BUFFER_DATA_TYPE * item)
{
    assert(queue != NULL);
    assert(item != NULL);

    if(queue->head < queue->capacity)
    {
        queue->buffer[ queue->head++ ] = * item;
    }
    else
    {
        queue->first_lap = 0;
        queue->head = 0;
        queue->buffer[ queue->head++ ] = * item;
    }

    moving_buffer_linearize(queue);
}



#ifdef __cplusplus
}
#endif







#endif //AVIONICS_MOVING_BUFFER_H
