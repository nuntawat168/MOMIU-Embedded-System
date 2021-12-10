/*
 * momiu.h
 *
 *  Created on: Jul 5, 2021
 *      Author: ohm
 */

#ifndef INC_MOMIU_H_
#define INC_MOMIU_H_

#ifdef __cplusplus
  extern "C" {
#endif

#include <stdio.h>
#include <string.h>

const float DEFAULT_SPEED_MOTOR_ARRAY[] = {0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50};
//float current_position_of_all_motor[8];
//float current_velocity_of_all_motor[8];
float current_pose;
float next_pose;

typedef struct {
	float position[8];
  	float speed[8];
}via_point;

typedef float array_of_4_double[4];
typedef float matrix_8_4_double[8][4];

typedef struct {
  via_point default_m;
  via_point mainternance1;
  via_point mainternance2;
  via_point sad;
  via_point happy[2];
  via_point hi;
  via_point angry[2];
  via_point dance[3];
  via_point thinking;
  via_point shocked;
}pose_t;

extern pose_t pose;

void drive_all_Dynamixel(float pos_motor[8], float speed_motor[8]);

void default_pose();

void mainternance1_pose();

void trajectory_genaration(via_point viapoint1, via_point viapoint2, uint32_t duration, int priority, array_of_4_double *coeffcient);

float cubic_spline_position(float coeffcient[4], uint32_t t);
float cubic_spline_velocity(float coeffcient[4], uint32_t t);

#ifdef __cplusplus
  }
#endif

#endif /* INC_MOMIU_H_ */
