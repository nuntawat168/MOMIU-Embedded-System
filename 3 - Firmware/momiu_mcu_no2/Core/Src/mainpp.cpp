/*
 * cycle_queue_multi_array.c
 *
 *  Created on: Jul 12, 2021
 *      Author: Nuntawat Maliwan
 */
#include <main.h>
#include <mainpp.h>
#include <stdio.h>
#include <string.h>

#include <ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <momiu_msg/IR_message.h>
#include <momiu_msg/momiu_posMotor.h>
#include <momiu_msg/pose_message.h>
#include <momiu_msg/SelectMode.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include "Dynamixel_Serial.h"
#include "bno055_stm32.h"
#include "VL53L0x.h"
#include "momiu.h"
#include "cycle_queue_multi_array.h"

#define MX28_DEG_PER_UNIT 0.088
#define AX12_DEG_PER_UNIT 0.29
#define MX28_RPM_PER_UNIT 0.114
#define AX12_RPM_PER_UNIT 0.111
#define DEFAULT_SPEED_MOTOR 0x50

#define threshold_IR_chin 58
#define VL53L0x_add_chin 0x52
#define VL53L0x_add_forehead 0x52

struct VL53L0xData VL53L0x_Data_chin;
struct VL53L0xData VL53L0x_Data_forehead;

ros::NodeHandle nh;

char a[1000];
char pose_name_list[9][50] = {
								"hello"		,
								"tickle"	,
								"happy"		,
								"angry"		,
								"surprise"	,
								"sad"		,
								"natural"	,
								"confuse"   ,
								"dance"
											};

float pose_list[9][10][8] =
						  {
							{{4,250,1,0.7,0.5},{175, 180, 180, 180, 180, 180,  40,  40},{175, 145, 120, 250, 250, 240,  40,  40},{175, 180, 180, 180, 180, 180,  40,  40}},
							{{4,100,1,0.7,0.5},{175, 210, 180, 180, 180, 180,  40,  40},{175, 320, 180, 180, 180, 180,  40,  40},{175, 180, 180, 180, 180, 180,  40,  40}},
							{{4,120,1,0.7,0.5},{175, 180, 180, 180, 180, 180,  40,  40},{175, 180, 250, 180, 180, 110,   0,   0},{175, 180, 180, 180, 180, 180,  40,  40}},
							{{4,300,1,0.7,0.5},{175, 180, 180, 180, 180, 180,  40,  40},{145, 180, 180, 180, 180, 180, 140, 140},{215, 180, 180, 180, 180, 180, 140, 140},{145, 180, 180, 180, 180, 180, 140, 140},{215, 180, 180, 180, 180, 180, 140, 140},{175, 180, 180, 180, 180, 180,  40,  40}},
							{{4,200,1,0.7,0.5},{175, 210, 180, 180, 180, 180,  40,  40},{175, 290, 180, 180, 180, 180,  40,  40},{175, 180, 180, 180, 180, 180,  40,  40}},
							{{4,200,1,0.7,0.5},{175, 180, 180, 180, 180, 180,  40,  40},{175, 140, 160, 180, 180, 200, 120, 120},{175, 180, 180, 180, 180, 180,  80,  80}},
							{{4,200,1,0.7,0.5},{175, 210, 180, 180, 180, 180,  40,  40},{175, 290, 180, 180, 180, 180,  40,  40},{175, 180, 180, 180, 180, 180,  40,  40}},
							{{4,400,1,0.7,0.5},{175, 180, 180, 180, 180, 180,  40,  40},{175, 155, 150, 240, 180, 180,  40,  40},{175, 155, 150, 240, 180, 180,  40,  40},{175, 155, 150, 240, 180, 180,  40,  40},{175, 155, 150, 240, 180, 180,  40,  40},{175, 180, 180, 180, 180, 180,  40,  40}},
							{{4,241,1,0.7,0.5},{175, 180, 180, 180, 180, 180,  40,  40},{200, 220, 220, 180, 180, 220,  40, 120},{175, 180, 180, 180, 180, 180,  40,  40},{150, 220, 140, 180, 180, 140, 120,  40},{175, 180, 180, 180, 180, 180,  40,  40}}
								};

/*double pose_list[8][10][8] = {
	pose_No	= 0					{{pose_priority,time},{via_point_0},{via_point_1},{via_point_2},..,{via_point_n}},
	pose_No	= 1					{{pose_priority,time},{via_point_0},{via_point_1},{via_point_2},..,{via_point_n}},
	pose_No	= 2					{{pose_priority,time},{via_point_0},{via_point_1},{via_point_2},..,{via_point_n}},
			 ...
	pose_No	= n					{{pose_priority,time},{via_point_0},{via_point_1},{via_point_2},..,{via_point_n}},m7m8
								};

double pose_table[n][n][n] = {
	pose_No	= 0	->				{{initial_duration,gain1,gain2,gain3},{via_point_0},{via_point_1},{via_point_2},..,{via_point_n}},
	pose_No	= 1	->				{{initial_duration,gain1,gain2,gain3},{via_point_0},{via_point_1},{via_point_2},..,{via_point_n}},
	pose_No	= 2	->				{{initial_duration,gain1,gain2,gain3},{via_point_0},{via_point_1},{via_point_2},..,{via_point_n}},
			 ...
	pose_No	= n	->				{{initial_duration,gain1,gain2,gain3},{via_point_0},{via_point_1},{via_point_2},..,{via_point_n}},
								};

 	 example: via_point_0 = {175, 155, 150, 240, 240, 210,  40,  40}

 #time unit is milliseconds
 #via_point_n is array of via point each of motor
  like this -> {via_point_of_motor#1 ,via_point_of_motor#0, via_point_of_motor#2, via_point_of_motor#3, via_point_of_motor#4, via_point_of_motor#5 ,via_point_of_motor#6, via_point_of_motor#8}
  example: via_point_0 = {175, 155, 150, 240, 240, 210,  40,  40}
*/

cycle_queue trajectory_queue;
uint32_t start_action_time;

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c3;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim11;

void momiu_posMotor_cb(const momiu_p::momiu_posMotor& msg);
void pose_message_cb(const momiu_p::pose_message& msg);
void display_queue();

sensor_msgs::Imu imu1;
ros::Publisher pub_imu1("imu1", &imu1);
sensor_msgs::Imu imu2;
ros::Publisher pub_imu2("imu2", &imu2);
std_msgs::Bool IR_chin;
ros::Publisher pub_IR_chin("IR_chin", &IR_chin);
std_msgs::Float32 IR_face;
ros::Publisher pub_IR_face("IR_face", &IR_face);

ros::Subscriber<momiu_p::momiu_posMotor> momiu_posMotor_sub_n("momiu_posMotor_t", &momiu_posMotor_cb);
ros::Subscriber<momiu_p::pose_message> pose_message_sub_n("pose", &pose_message_cb);

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->reset_rbuf();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim10)
	{
		uint32_t delta_time = HAL_GetTick() - start_action_time;
		uint32_t duration_on_queue = (trajectory_queue.array[trajectory_queue.front][0][1])*10;

		if(delta_time <= duration_on_queue)
		{
			unsigned int t = delta_time/10;

			double q_m1 = cubic_spline_position(trajectory_queue.array[trajectory_queue.front][1], t);
			double qq_m1 = cubic_spline_velocity(trajectory_queue.array[trajectory_queue.front][1], t);

			double q_m2 = cubic_spline_position(trajectory_queue.array[trajectory_queue.front][2], t);
			double qq_m2 = cubic_spline_velocity(trajectory_queue.array[trajectory_queue.front][2], t);

			double q_m3 = cubic_spline_position(trajectory_queue.array[trajectory_queue.front][3], t);
			double qq_m3 = cubic_spline_velocity(trajectory_queue.array[trajectory_queue.front][3], t);

			double q_m4 = cubic_spline_position(trajectory_queue.array[trajectory_queue.front][4], t);
			double qq_m4 = cubic_spline_velocity(trajectory_queue.array[trajectory_queue.front][4], t);

			double q_m5 = cubic_spline_position(trajectory_queue.array[trajectory_queue.front][5], t);
			double qq_m5 = cubic_spline_velocity(trajectory_queue.array[trajectory_queue.front][5], t);

			double q_m6 = cubic_spline_position(trajectory_queue.array[trajectory_queue.front][6], t);
			double qq_m6 = cubic_spline_velocity(trajectory_queue.array[trajectory_queue.front][6], t);

			double q_m7 = cubic_spline_position(trajectory_queue.array[trajectory_queue.front][7], t);
			double qq_m7 = cubic_spline_velocity(trajectory_queue.array[trajectory_queue.front][7], t);

			double q_m8 = cubic_spline_position(trajectory_queue.array[trajectory_queue.front][8], t);
			double qq_m8 = cubic_spline_velocity(trajectory_queue.array[trajectory_queue.front][8], t);

			Dynamixel_servo((0x01),(q_m1/MX28_DEG_PER_UNIT),(qq_m1/MX28_RPM_PER_UNIT));
			Dynamixel_servo((0x02),(q_m2/MX28_DEG_PER_UNIT),(qq_m2/MX28_RPM_PER_UNIT));
			Dynamixel_servo((0x03),(q_m3/MX28_DEG_PER_UNIT),(qq_m3/MX28_RPM_PER_UNIT));
			Dynamixel_servo((0x04),(q_m4/MX28_DEG_PER_UNIT),(qq_m4/MX28_RPM_PER_UNIT));
			Dynamixel_servo((0x05),(q_m5/MX28_DEG_PER_UNIT),(qq_m5/MX28_RPM_PER_UNIT));
			Dynamixel_servo((0x06),(q_m6/MX28_DEG_PER_UNIT),(qq_m6/MX28_RPM_PER_UNIT));
			Dynamixel_servo((0x07),(q_m7/AX12_DEG_PER_UNIT),(qq_m7/AX12_RPM_PER_UNIT));
			Dynamixel_servo((0x08),(q_m8/AX12_DEG_PER_UNIT),(qq_m8/AX12_RPM_PER_UNIT));

			/*snprintf(a, 1000,"delta_time=%ld,duration_on_queue=%ld\n", delta_time,duration_on_queue);
			char *TransmitBuffer = a;
			HAL_UART_Transmit(&huart2, (uint8_t*)TransmitBuffer, strlen(TransmitBuffer), 0xFFFF);*/
		}
		else if(delta_time > duration_on_queue)
		{
			/*snprintf(a, 1000,"delta_time=%ld,duration_on_queue=%ld\n", delta_time,duration_on_queue);
			char *TransmitBuffer = a;
			HAL_UART_Transmit(&huart2, (uint8_t*)TransmitBuffer, strlen(TransmitBuffer), 0xFFFF);*/

			//check_cycle_queue_empty() = 0 is cycle queue is "not empty"
			//check_cycle_queue_empty() = 1 is cycle queue is "empty"
			if(check_cycle_queue_empty(&trajectory_queue)==0)
			{
				next_cycle_queue(&trajectory_queue);
				start_action_time=HAL_GetTick();
			}
			else if(check_cycle_queue_empty(&trajectory_queue)==1)
			{

			}
		}
	}
}

void setup(void)
{
  init_cycle_queue(&trajectory_queue);

  nh.initNode();
  nh.advertise(pub_imu1);
  nh.advertise(pub_imu2);
  nh.advertise(pub_IR_chin);
  nh.advertise(pub_IR_face);
  nh.subscribe(momiu_posMotor_sub_n);
  nh.subscribe(pose_message_sub_n);

  bno055_assignI2C(&hi2c2, 0x28);
  bno055_setup();
  bno055_setOperationModeNDOF();

  HAL_Delay(10);
  bno055_assignI2C(&hi2c1, 0x29);
  bno055_setup();
  bno055_setOperationModeNDOF();

  default_pose();
}

void loop(void)
{
	bno055_assignI2C(&hi2c2, 0x28);
	bno055_vector_t imu1_orientation = bno055_getVectorEuler();
	bno055_vector_t imu1_angular_velocity = bno055_getVectorGyroscope();
	bno055_vector_t imu1_linear_acceleration = bno055_getVectorAccelerometer();
	imu1.orientation.x=imu1_orientation.x;
	imu1.orientation.y=imu1_orientation.y;
	imu1.orientation.z=imu1_orientation.z;
	imu1.angular_velocity.x=imu1_angular_velocity.x;
	imu1.angular_velocity.y=imu1_angular_velocity.y;
	imu1.angular_velocity.z=imu1_angular_velocity.z;
	imu1.linear_acceleration.x=imu1_linear_acceleration.x;
	imu1.linear_acceleration.y=imu1_linear_acceleration.y;
	imu1.linear_acceleration.z=imu1_linear_acceleration.z;
	imu1.header.frame_id="base_imu1";
	pub_imu1.publish(&imu1);

	//HAL_Delay(1);
	bno055_assignI2C(&hi2c1, 0x29);
	bno055_vector_t imu2_orientation = bno055_getVectorEuler();
	bno055_vector_t imu2_angular_velocity = bno055_getVectorGyroscope();
	bno055_vector_t imu2_linear_acceleration = bno055_getVectorAccelerometer();
	imu2.orientation.x=imu2_orientation.x;
	imu2.orientation.y=imu2_orientation.y;
	imu2.orientation.z=imu2_orientation.z;
	imu2.angular_velocity.x=imu2_angular_velocity.x;
	imu2.angular_velocity.y=imu2_angular_velocity.y;
	imu2.angular_velocity.z=imu2_angular_velocity.z;
	imu2.linear_acceleration.x=imu2_linear_acceleration.x;
	imu2.linear_acceleration.y=imu2_linear_acceleration.y;
	imu2.linear_acceleration.z=imu2_linear_acceleration.z;
	imu2.header.frame_id="base_imu2";
	pub_imu2.publish(&imu2);

	VL53L0x_StartConversion(&hi2c3, VL53L0x_add_chin);
	VL53L0x_ReadDistance(&hi2c3, &VL53L0x_Data_chin, VL53L0x_add_chin);
	if(VL53L0x_Data_chin.distValidFinal < threshold_IR_chin)
	{
		IR_chin.data = true;
	}
	else
	{
		IR_chin.data = false;
	}
	pub_IR_chin.publish(&IR_chin);

	//HAL_Delay(1);
	VL53L0x_StartConversion(&hi2c2, VL53L0x_add_forehead);
	VL53L0x_ReadDistance(&hi2c2, &VL53L0x_Data_forehead, VL53L0x_add_forehead);
	IR_face.data = VL53L0x_Data_forehead.distValidFinal;
	pub_IR_face.publish(&IR_face);

	/*snprintf(a, 1000,"dist_VL_chin: %d \n",VL53L0x_Data_chin.distValidFinal);
	char *TransmitBuffer = a;
	HAL_UART_Transmit(&huart2, (uint8_t*)TransmitBuffer, strlen(TransmitBuffer), 0xFFFF);

	snprintf(a, 1000,"dist_VL_forehead: %d \n",VL53L0x_Data_forehead.distValidFinal);
	TransmitBuffer = a;
	HAL_UART_Transmit(&huart2, (uint8_t*)TransmitBuffer, strlen(TransmitBuffer), 0xFFFF);*/

	nh.spinOnce();
	HAL_Delay(10);
}

void momiu_posMotor_cb(const momiu_p::momiu_posMotor& msg)
{

	int16_t moter1_pos = msg.motor1_pos;
	int16_t moter2_pos = msg.motor2_pos;
	int16_t moter3_pos = msg.motor3_pos;
	int16_t moter4_pos = msg.motor4_pos;
	int16_t moter5_pos = msg.motor5_pos;
	int16_t moter6_pos = msg.motor6_pos;
	int16_t moter7_pos = msg.motor7_pos;
	int16_t moter8_pos = msg.motor8_pos;

	Dynamixel_servo((0x01),(moter1_pos/0.088),(0x50));
	Dynamixel_servo((0x02),(moter2_pos/0.088),(0x50));
	Dynamixel_servo((0x03),(moter3_pos/0.088),(0x50));
	Dynamixel_servo((0x04),(moter4_pos/0.088),(0x50));
	Dynamixel_servo((0x05),(moter5_pos/0.088),(0x50));
	Dynamixel_servo((0x06),(moter6_pos/0.088),(0x50));
	Dynamixel_servo((0x07),(moter7_pos/0.29),(0x50));
	Dynamixel_servo((0x08),(moter8_pos/0.29),(0x50));

	/*snprintf(a, 1000,"motor1_pos : %d ,motor2_pos : %d ,motor3_pos : %d ,motor4_pos : %d ,motor5_pos : %d ,motor6_pos : %d ,motor7_pos : %d ,motor8_pos : %d \n",moter1_pos ,moter2_pos ,moter3_pos ,moter4_pos ,moter5_pos ,moter6_pos ,moter7_pos ,moter8_pos);
	char *TransmitBuffer = a;
	HAL_UART_Transmit(&huart2, (uint8_t*)TransmitBuffer, strlen(TransmitBuffer), 0xFFFF);*/

}

void pose_message_cb(const momiu_p::pose_message& msg)
{
	char pose_name[50];
	snprintf(pose_name,50,"%s",msg.pose);
	int pose_No ;

	//--mapping "pose name" to "pose list array index"--
	char pose_name_list[9][50] = {"hello","tickle","happy","angry","surprise","sad","natural","confuse","dance"};
		 if(!strcmp ( pose_name, pose_name_list[0]))	{ pose_No =  0; }
	else if(!strcmp ( pose_name, pose_name_list[1]))	{ pose_No =  1; }
	else if(!strcmp ( pose_name, pose_name_list[2]))	{ pose_No =  2; }
	else if(!strcmp ( pose_name, pose_name_list[3]))	{ pose_No =  3; }
	else if(!strcmp ( pose_name, pose_name_list[4]))	{ pose_No =  4; }
	else if(!strcmp ( pose_name, pose_name_list[5]))	{ pose_No =  5; }
	else if(!strcmp ( pose_name, pose_name_list[6]))	{ pose_No =  6; }
	else if(!strcmp ( pose_name, pose_name_list[7]))	{ pose_No =  7; }
	else if(!strcmp ( pose_name, pose_name_list[8]))	{ pose_No =  8; }
	else{ pose_No =  99; }

	//get priority and duration of pose from pose list
	int priority_of_pose = pose_list[pose_No][0][0]; //now, priority of pose not yet implemented
	int duration_of_pose = pose_list[pose_No][0][1];

	uint32_t duration_per_subTrajectory;
	int num_viapoint_of_pose;
	for(int i=1;i<=10;i++)
	{
		//Get number of via point of pose
		int check_via_point = pose_list[pose_No][i+1][0] + pose_list[pose_No][i+1][1] + pose_list[pose_No][i+1][2] + pose_list[pose_No][i+1][3] + pose_list[pose_No][i+1][4] + pose_list[pose_No][i+1][5] + pose_list[pose_No][i+1][6] + pose_list[pose_No][i+1][7];
		if(check_via_point < 1)
		{
			num_viapoint_of_pose = i ;
			break;
		}
	}

	//find duration of sub trajectory by average from total duration of pose
	duration_per_subTrajectory = (duration_of_pose/(num_viapoint_of_pose-1));

	for(int j = 1 ; j <= msg.loop;j++)
	{
		for(int i=1;i<=num_viapoint_of_pose-1;i++)
		{
			//is trajectory queue empty?
			//if trajectory queue "is empty"     -> generate trajectory of pose then put coefficient of trajectory to cycle queue and assign start_action_time = HAL_GetTick() for start drive motor to follow trajectory in timer interrupt function(HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim))
			//if trajectory queue "is not empty" -> generate trajectory of pose then put coefficient of trajectory to cycle queue
			if(check_cycle_queue_empty(&trajectory_queue)==1)
			{
				via_point via_point_init;
				via_point via_point_final;
				memcpy(via_point_init.position,pose_list[pose_No][i],100);
				memcpy(via_point_final.position,pose_list[pose_No][i+1],100);

				float temp_coeff_cubic_spline[9][4];

				trajectory_genaration(via_point_init, via_point_final, duration_per_subTrajectory, priority_of_pose, temp_coeff_cubic_spline);

				put_cycle_queue(&trajectory_queue, temp_coeff_cubic_spline);
				start_action_time=HAL_GetTick();
				/*display_queue();

				snprintf(a, 1000,"trajectory_no_%d -> init:{%f %f %f %f %f %f %f %f}  \n",i,via_point_init.position[0],via_point_init.position[1],via_point_init.position[2],via_point_init.position[3],via_point_init.position[4],via_point_init.position[5],via_point_init.position[6],via_point_init.position[7]);
				char *TransmitBuffer = a;
				HAL_UART_Transmit(&huart2, (uint8_t*)TransmitBuffer, strlen(TransmitBuffer), 0xFFFF);

				snprintf(a, 1000,"trajectory_no_%d -> final:{%f %f %f %f %f %f %f %f}  \n",i,via_point_final.position[0],via_point_final.position[1],via_point_final.position[2],via_point_final.position[3],via_point_final.position[4],via_point_final.position[5],via_point_final.position[6],via_point_final.position[7]);
				TransmitBuffer = a;
				HAL_UART_Transmit(&huart2, (uint8_t*)TransmitBuffer, strlen(TransmitBuffer), 0xFFFF);

				snprintf(a, 1000,"coeff-m1: prirority=%f, duration=%f\n",temp_coeff_cubic_spline[0][0], temp_coeff_cubic_spline[0][1]);
				TransmitBuffer = a;
				HAL_UART_Transmit(&huart2, (uint8_t*)TransmitBuffer, strlen(TransmitBuffer), 0xFFFF);

				snprintf(a, 1000,"coeff-m1: c0=%f, c1=%f, c2=%f, c3=%f\n",temp_coeff_cubic_spline[1][0],temp_coeff_cubic_spline[1][1],temp_coeff_cubic_spline[1][2],temp_coeff_cubic_spline[1][3]);
				TransmitBuffer = a;
				HAL_UART_Transmit(&huart2, (uint8_t*)TransmitBuffer, strlen(TransmitBuffer), 0xFFFF);

				snprintf(a, 1000,"coeff-m2: c0=%f, c1=%f, c2=%f, c3=%f\n",temp_coeff_cubic_spline[2][0],temp_coeff_cubic_spline[2][1],temp_coeff_cubic_spline[2][2],temp_coeff_cubic_spline[2][3]);
				TransmitBuffer = a;
				HAL_UART_Transmit(&huart2, (uint8_t*)TransmitBuffer, strlen(TransmitBuffer), 0xFFFF);

				snprintf(a, 1000,"***************************************************\n");
				TransmitBuffer = a;
				HAL_UART_Transmit(&huart2, (uint8_t*)TransmitBuffer, strlen(TransmitBuffer), 0xFFFF);*/
			}
			else if(check_cycle_queue_empty(&trajectory_queue)==0)
			{
				via_point via_point_init;
				via_point via_point_final;
				memcpy(via_point_init.position,pose_list[pose_No][i],100);
				memcpy(via_point_final.position,pose_list[pose_No][i+1],100);

				float temp_coeff_cubic_spline[9][4];

				trajectory_genaration(via_point_init, via_point_final, duration_per_subTrajectory, priority_of_pose, temp_coeff_cubic_spline);

				put_cycle_queue(&trajectory_queue, temp_coeff_cubic_spline);
				/*display_queue();

				snprintf(a, 1000,"trajectory_no_%d -> init:{%f %f %f %f %f %f %f %f}  \n",i,via_point_init.position[0],via_point_init.position[1],via_point_init.position[2],via_point_init.position[3],via_point_init.position[4],via_point_init.position[5],via_point_init.position[6],via_point_init.position[7]);
				char *TransmitBuffer = a;
				HAL_UART_Transmit(&huart2, (uint8_t*)TransmitBuffer, strlen(TransmitBuffer), 0xFFFF);

				snprintf(a, 1000,"trajectory_no_%d -> final:{%f %f %f %f %f %f %f %f}  \n",i,via_point_final.position[0],via_point_final.position[1],via_point_final.position[2],via_point_final.position[3],via_point_final.position[4],via_point_final.position[5],via_point_final.position[6],via_point_final.position[7]);
				TransmitBuffer = a;
				HAL_UART_Transmit(&huart2, (uint8_t*)TransmitBuffer, strlen(TransmitBuffer), 0xFFFF);

				snprintf(a, 1000,"coeff-m1: prirority=%f, duration=%f\n",temp_coeff_cubic_spline[0][0], temp_coeff_cubic_spline[0][1]);
				TransmitBuffer = a;
				HAL_UART_Transmit(&huart2, (uint8_t*)TransmitBuffer, strlen(TransmitBuffer), 0xFFFF);

				snprintf(a, 1000,"coeff-m1: c0=%f, c1=%f, c2=%f, c3=%f\n",temp_coeff_cubic_spline[1][0],temp_coeff_cubic_spline[1][1],temp_coeff_cubic_spline[1][2],temp_coeff_cubic_spline[1][3]);
				TransmitBuffer = a;
				HAL_UART_Transmit(&huart2, (uint8_t*)TransmitBuffer, strlen(TransmitBuffer), 0xFFFF);

				snprintf(a, 1000,"coeff-m2: c0=%f, c1=%f, c2=%f, c3=%f\n",temp_coeff_cubic_spline[2][0],temp_coeff_cubic_spline[2][1],temp_coeff_cubic_spline[2][2],temp_coeff_cubic_spline[2][3]);
				TransmitBuffer = a;
				HAL_UART_Transmit(&huart2, (uint8_t*)TransmitBuffer, strlen(TransmitBuffer), 0xFFFF);

				snprintf(a, 1000,"***************************************************\n");
				TransmitBuffer = a;
				HAL_UART_Transmit(&huart2, (uint8_t*)TransmitBuffer, strlen(TransmitBuffer), 0xFFFF);*/
			}
		}
	}
}
