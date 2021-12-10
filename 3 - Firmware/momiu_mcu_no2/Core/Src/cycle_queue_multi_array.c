/*
 * cycle_queue_multi_array.c
 *
 *  Created on: Jul 12, 2021
 *      Author: Nuntawat Maliwan
 */
/******************************************************************************

                    Cycle Queue with multidimensional array

*******************************************************************************/

#include <stdio.h>
#include <string.h>
#include "cycle_queue_multi_array.h"

void init_cycle_queue(cycle_queue *queue)
{
    queue -> front = -1;
    queue -> rear  = -1;
}

void put_cycle_queue(cycle_queue *queue, float elemant[9][4])
{
    if((queue->front==0&&queue->rear==max-1)||(queue->rear+1==queue->front))
    {
        printf("Queu is overflow\n");
    }
    else
    {
        if(queue->rear==-1)
        {
            queue->front=0,queue->rear=0;
        }
        else if(queue->rear==max-1)
        {
            queue->rear=0;
        }
        else
        {
            queue->rear++;

        }
        memcpy(queue->array[queue->rear],elemant,500);
    }
}

void get_cycle_queue(cycle_queue *queue, array_of_4_double *temp)
{
    if(queue->front ==-1)
    {
        printf("Queue is underflow\n");
    }
    else
    {
        memcpy(temp,queue->array[queue->front],500);
        if(queue->front==queue->rear)
        {
            queue->front=-1 ,queue->rear=-1;
        }
        else if(queue->front==max-1)
        {
            queue->front=0;
        }
        else
        {
            queue->front++;
        }
    }
}

void next_cycle_queue(cycle_queue *queue)
{
    if(queue->front ==-1)
    {
        printf("Queue is underflow\n");
    }
    else
    {
        if(queue->front==queue->rear)
        {
            queue->front=-1 ,queue->rear=-1;
        }
        else if(queue->front==max-1)
        {
            queue->front=0;
        }
        else
        {
            queue->front++;
        }
    }
}

int check_cycle_queue_empty(cycle_queue *queue)
{
       if(queue->front ==-1)
        {
            printf("Queue is Empty\n");
            return 1;
        }
        else
        {
            return 0;
        }
}


