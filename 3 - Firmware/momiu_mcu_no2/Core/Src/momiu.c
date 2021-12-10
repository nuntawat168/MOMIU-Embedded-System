/*
 * momiu.c
 *
 *  Created on: Jul 5, 2021
 *      Author: Nuntawat Maliwan
 */
#define MX28_DEG_PER_UNIT 0.088
#define AX12_DEG_PER_UNIT 0.29
#define MX28_RPM_PER_UNIT 0.114
#define AX12_RPM_PER_UNIT 0.111
#define DEFAULT_SPEED_MOTOR 0x50

#include <stdio.h>
#include <string.h>
#include "momiu.h"
#include "Dynamixel_Serial.h"

pose_t pose = {
		.default_m.position		= {175, 180, 180, 180, 180, 180,  40,  40}, .default_m.speed		= { 5,  5,  5,  5,  5,  5,  5,  5},
		.mainternance1.position = {175, 210, 180, 180, 180, 180,  40,  40}, .mainternance1.speed 	= { 0,  0,  0,  0,  0,  0,  0,  0},
		.mainternance2.position = {175, 220, 180, 180, 180, 180,  40,  40}, .mainternance2.speed	= { 0,  0,  0,  0,  0,  0,  0,  0},
};

void drive_all_Dynamixel(float pos_motor[8], float speed_motor[8]){
	  Dynamixel_servo((0x01),(pos_motor[0]/MX28_DEG_PER_UNIT),(speed_motor[0]/MX28_RPM_PER_UNIT));
	  Dynamixel_servo((0x02),(pos_motor[1]/MX28_DEG_PER_UNIT),(speed_motor[1]/MX28_RPM_PER_UNIT));
	  Dynamixel_servo((0x03),(pos_motor[2]/MX28_DEG_PER_UNIT),(speed_motor[2]/MX28_RPM_PER_UNIT));
	  Dynamixel_servo((0x04),(pos_motor[3]/MX28_DEG_PER_UNIT),(speed_motor[3]/MX28_RPM_PER_UNIT));
	  Dynamixel_servo((0x05),(pos_motor[4]/MX28_DEG_PER_UNIT),(speed_motor[4]/MX28_RPM_PER_UNIT));
	  Dynamixel_servo((0x06),(pos_motor[5]/MX28_DEG_PER_UNIT),(speed_motor[5]/MX28_RPM_PER_UNIT));
	  Dynamixel_servo((0x07),(pos_motor[6]/AX12_DEG_PER_UNIT),(speed_motor[6]/AX12_RPM_PER_UNIT));
	  Dynamixel_servo((0x08),(pos_motor[7]/AX12_DEG_PER_UNIT),(speed_motor[7]/AX12_RPM_PER_UNIT));
}

void default_pose(){
	drive_all_Dynamixel(pose.default_m.position,pose.default_m.speed);
}

void mainternance1_pose(){
	drive_all_Dynamixel(pose.mainternance1.position, pose.mainternance1.speed);
}

void mainternance2_pose(){
	drive_all_Dynamixel(pose.mainternance2.position, pose.mainternance2.speed);
}

void trajectory_genaration(via_point viapoint_initial, via_point viapoint_final, uint32_t duration, int priority, array_of_4_double *coeffcient)
{

	coeffcient[0][0] = priority;
	coeffcient[0][1] = duration;

	for (int  i = 1; i <= 8; ++i)
	{
		float c[5];
		float qi = viapoint_initial.position[i-1];
		//double vi = viapoint_initial.speed[i-1];
		float vi = 0;

		float qf = viapoint_final.position[i-1];
		//double vf = viapoint_final.speed[i-1];
		float vf = 0;
		uint32_t T = duration;

		c[0] = qi;
		c[1] = vi;

		c[2] = ((3*(qf-qi))/(T*T)) - ((vf+(2*vi))/T);
		c[3] = ((vf+vi)/(T*T))     - ((2*(qf-qi))/(T*T*T));

		coeffcient[i][0] = c[0];
		coeffcient[i][1] = c[1];
		coeffcient[i][2] = c[2];
		coeffcient[i][3] = c[3];
	}
}

float cubic_spline_position(float coeffcient[4], uint32_t t){

	return coeffcient[0] + (coeffcient[1]*t) + (coeffcient[2] * (t*t)) + (coeffcient[3]*(t*t*t));
}

float cubic_spline_velocity(float coeffcient[4], uint32_t t){

	return (coeffcient[1] + (2*coeffcient[2]*t) + (3*coeffcient[3]*(t*t)))* 95.49297;
}
