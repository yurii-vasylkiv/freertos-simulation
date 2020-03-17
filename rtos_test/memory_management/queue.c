#include "queue.h"
#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>


void buffer_queue_init(buffer_queue *cb)
{
    cb->capacity = BUFFER_QUEUE_CAPACITY;
    cb->count = 0;
    cb->sz = sizeof( buffer_item );
    cb->head = cb->buffer;
    cb->tail = cb->buffer;
    cb->buffer_end = &cb->buffer[cb->capacity * cb->sz];
}

int buffer_queue_push_back(buffer_queue *cb, buffer_item *item)
{
    if(cb->count == cb->capacity)
    {
        return 1;
    }

    taskENTER_CRITICAL();
    {
        memcpy( cb->head, item, cb->sz );
        cb->head = ( uint8_t * ) cb->head + cb->sz;
        if ( cb->head == cb->buffer_end )
            cb->head = cb->buffer;
        cb->count++;
    }
    taskEXIT_CRITICAL();

    return 0;
}

int buffer_queue_pop_front(buffer_queue *cb, buffer_item *item)
{
    if(cb->count == 0)
    {
        return 0;
    }

    taskENTER_CRITICAL();
    {
        memcpy( item, cb->tail, cb->sz );

        cb->tail = ( uint8_t * ) cb->tail + cb->sz;
        if ( cb->tail == cb->buffer_end )
            cb->tail = cb->buffer;
        cb->count--;
    }
    taskEXIT_CRITICAL();

    return 1;
}

void int_averaging_queue_init( int_averaging_queue *queue, uint32_t average_over_count)
{
    queue->capacity = INT_FLOAT_QUEUE_CAPACITY;
    queue->count = 0;
    queue->sz = sizeof(int16_t);
    queue->head = 0;
    queue->tail = 0;
    queue->buffer_end = INT_FLOAT_QUEUE_CAPACITY;
    queue->first_round = 1;
    queue->averaging_buffer_count = average_over_count;
}


int int_averaging_queue_add( int_averaging_queue *cb, int16_t item)
{
    taskENTER_CRITICAL();
    {
        cb->buffer[ cb->head ] = item;
        cb->head++;

        if ( cb->head > cb->averaging_buffer_count )
        {
            cb->tail++;
            cb->first_round = 0;
        }

        if ( cb->head == cb->buffer_end )
        {
            cb->head = 0;
        }

        if ( cb->tail == cb->buffer_end )
        {
            cb->tail = 0;
        }

        if ( cb->head < cb->tail )
        {
            cb->tail++;
        }

    }
    taskEXIT_CRITICAL();
    return 0;
}

int int_averaging_queue_get( int_averaging_queue *cb, int16_t * item)
{
    taskENTER_CRITICAL();
    {
        float sum    = 0;

        if ( cb->head > cb->tail)
        {
            for ( int i = cb->tail ; i < cb->head ; i++ )
            {
                sum += cb->buffer[ i ];
            }
        }
        else
        {
            for ( int i = cb->tail ; i < cb->buffer_end ; i++ )
            {
                sum += cb->buffer[ i ];
            }

            for ( int i = 0 ; i <= cb->head ; i++ )
            {
                sum += cb->buffer[ i ];
            }
        }

        if(cb->first_round && cb->head < cb->capacity)
        {
            if(cb->head == 0)
            {
                ( *item ) = 0;
            }

            ( *item ) =  sum / ( cb->head - cb->tail );
        }
        else
        {
            ( *item ) = sum / cb->averaging_buffer_count;
        }
    }
    taskEXIT_CRITICAL();

//    printf("IMU Average : %d\n", *item);


    return 1;
}




void float_averaging_queue_init( float_averaging_queue *queue, uint32_t average_over_count )
{
    queue->capacity = INT_FLOAT_QUEUE_CAPACITY;
    queue->count = 0;
    queue->sz = sizeof(float);
    queue->head = 0;
    queue->tail = 0;
    queue->buffer_end = INT_FLOAT_QUEUE_CAPACITY;
    queue->first_round = 1;
    queue->averaging_buffer_count = average_over_count;
}


int float_averaging_queue_add( float_averaging_queue *cb, float item)
{
    taskENTER_CRITICAL();
    {
        cb->buffer[ cb->head ] = item;
        cb->head++;

        if ( cb->head > cb->averaging_buffer_count )
        {
            cb->tail++;
            cb->first_round = 0;
        }

        if ( cb->head == cb->buffer_end )
        {
            cb->head = 0;
        }

        if ( cb->tail == cb->buffer_end )
        {
            cb->tail = 0;
        }

        if ( cb->head < cb->tail )
        {
            cb->tail++;
        }

    }
    taskEXIT_CRITICAL();

    return 0;
}

int float_averaging_queue_get( float_averaging_queue *cb, float * item)
{
    taskENTER_CRITICAL();
    {
        float sum    = 0;

        if ( cb->head > cb->tail)
        {
            for ( int i = cb->tail ; i < cb->head ; i++ )
            {
                sum += cb->buffer[ i ];
            }
        }
        else
        {
            for ( int i = cb->tail ; i < cb->buffer_end ; i++ )
            {
                sum += cb->buffer[ i ];
            }

            for ( int i = 0 ; i <= cb->head ; i++ )
            {
                sum += cb->buffer[ i ];
            }
        }

        if(cb->first_round && cb->head < cb->capacity)
        {
            ( *item ) =  sum / ( cb->head - cb->tail );
        }
        else
        {
            ( *item ) = sum / cb->averaging_buffer_count;
        }
    }
    taskEXIT_CRITICAL();

//    printf("Pressure Average : %f\n", *item);

    return 1;
}