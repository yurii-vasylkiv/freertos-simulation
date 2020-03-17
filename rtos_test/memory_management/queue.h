#ifndef MEMORY_MANAGER_QUEUE_H
#define MEMORY_MANAGER_QUEUE_H

#include <inttypes.h>
#include <malloc.h>
#include <memory.h>

#define BUFFER_QUEUE_CAPACITY 25
#define INT_FLOAT_QUEUE_CAPACITY    250
typedef struct
{
    int8_t type;
    uint8_t data [256];
} buffer_item;


typedef struct
{
    uint8_t buffer [sizeof (buffer_item) * BUFFER_QUEUE_CAPACITY ]; // data buffer
    uint8_t *buffer_end;                                            // end of data buffer
    size_t capacity;                                                // maximum number of items in the buffer
    size_t count;                                                   // number of items in the buffer
    size_t sz;                                                      // size of each item in the buffer
    uint8_t *head;                                                  // pointer to head
    uint8_t *tail;                                                  // pointer to tail
} buffer_queue;


typedef struct
{
    int32_t buffer [INT_FLOAT_QUEUE_CAPACITY];                            // data buffer
    uint32_t buffer_end;                                            // end of data buffer
    size_t capacity;                                                // maximum number of items in the buffer
    size_t count;                                                   // number of items in the buffer
    size_t sz;                                                      // size of each item in the buffer
    uint32_t head;                                                  // pointer to head
    uint32_t tail;                                                  // pointer to tail
    uint8_t first_round;
    uint32_t averaging_buffer_count;

} int_averaging_queue;

typedef struct
{
    int16_t buffer [INT_FLOAT_QUEUE_CAPACITY];                              // data buffer
    uint32_t buffer_end;                                            // end of data buffer
    size_t capacity;                                                // maximum number of items in the buffer
    size_t count;                                                   // number of items in the buffer
    size_t sz;                                                      // size of each item in the buffer
    uint32_t head;                                                  // pointer to head
    uint32_t tail;                                                  // pointer to tail
    uint8_t first_round;
    uint32_t averaging_buffer_count;
} float_averaging_queue;

void buffer_queue_init          ( buffer_queue      *queue );
void int_averaging_queue_init   ( int_averaging_queue   *queue, uint32_t average_over_count );
void float_averaging_queue_init ( float_averaging_queue *queue, uint32_t average_over_count );

int buffer_queue_push_back ( buffer_queue  *cb, buffer_item *item );
int buffer_queue_pop_front ( buffer_queue  *cb, buffer_item *item );

int int_averaging_queue_add ( int_averaging_queue   *cb, int16_t item  );
int int_averaging_queue_get ( int_averaging_queue   *cb, int16_t *item );

int float_averaging_queue_add ( float_averaging_queue   *cb, float item  );
int float_averaging_queue_get ( float_averaging_queue   *cb, float *item );


#endif //MEMORY_MANAGER_QUEUE_H
