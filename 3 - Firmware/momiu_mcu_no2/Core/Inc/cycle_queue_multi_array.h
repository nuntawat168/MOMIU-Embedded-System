/*
 * cycle_queue_multi_array.h
 *
 *  Created on: Jul 12, 2021
 *      Author: Nuntawat Maliwan
 */
/******************************************************************************

                    Cycle Queue with multidimensional array

*******************************************************************************/


#ifndef INC_CYCLE_QUEUE_MULTI_ARRAY_H_
#define INC_CYCLE_QUEUE_MULTI_ARRAY_H_

#ifdef __cplusplus
  extern "C" {
#endif

#include <stdio.h>
#include <string.h>

#define max 20

typedef struct{
	float array[max][9][4];
    int front;
    int rear;
}cycle_queue;

typedef float array_of_4_double[4];

void init_cycle_queue(cycle_queue *queue);

void put_cycle_queue(cycle_queue *queue, float elemant[9][4]);

void get_cycle_queue(cycle_queue *queue, array_of_4_double *temp);

void next_cycle_queue(cycle_queue *queue);

int check_cycle_queue_empty(cycle_queue *queue);

#ifdef __cplusplus
  }
#endif

#endif /* INC_CYCLE_QUEUE_MULTI_ARRAY_H_ */
