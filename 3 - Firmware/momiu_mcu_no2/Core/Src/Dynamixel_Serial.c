#include <Dynamixel_Serial.h>
#include "usart.h"
#include "gpio.h"

extern UART_HandleTypeDef huart6;

void Dynamixel_servo(int Dynamixel_ID, uint16_t position, uint16_t speed) //(position, speed)
{
	uint8_t position_H = position >> 8 ;
	uint8_t position_L = position ;

	uint8_t speed_H = speed >> 8 ;
	uint8_t speed_L = speed ;

	uint8_t packet[11];
	packet[0] = 0xff; // Start communicating byte 1
	packet[1] = 0xff; // Start communicating byte 2
	packet[2] = Dynamixel_ID; // ID
	packet[3] = 0x07; // length : #num of parameter +2 = 7
	packet[4] = 0x03; // instruction : write
	packet[5] = 0x1e; // start write at address : goal position(L)(0x1e)
	packet[6] = position_L; // write on address 0x1e
	packet[7] = position_H; // write on address 0x1e+1
	packet[8] = speed_L;	// write on address 0x1e+2
	packet[9] = speed_H;	// write on address 0x1e+3
	uint8_t checksum = (~(packet[2]+packet[3]+packet[4]+packet[5]+packet[6]+packet[7]+packet[8]+packet[9]) & 0xff );
	packet[10] = checksum;

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET); // setup pin dir_data_ax12a high for transmit data
	HAL_UART_Transmit(&huart6, packet, 11, 10);
}
