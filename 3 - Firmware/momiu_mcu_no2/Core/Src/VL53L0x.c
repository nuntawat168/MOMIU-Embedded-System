#include "VL53L0x.h"
//#include <math.h>
//#include "PersonalMath.h"

 struct VL53L0xData VL53L0x_Bottom; //Data of VL53L0x connected to IIC2 at the bottom


void VL53L0x_init(I2C_HandleTypeDef *hi2c, uint16_t VL53L0x_add)
{
	uint8_t VL53L0x_SendData[2] = {0x01};
	uint8_t VL53L0x_RecData[5] ;

/*	//Revision ID:
	HAL_I2C_Mem_Read(hi2c, VL53L0x_add, VL53L0X_REG_IDENTIFICATION_REVISION_ID, I2C_MEMADD_SIZE_8BIT, VL53L0x_RecData, 1, 10);
	//Device ID:
	HAL_I2C_Mem_Read(hi2c, VL53L0x_add, VL53L0X_REG_IDENTIFICATION_MODEL_ID, I2C_MEMADD_SIZE_8BIT, VL53L0x_RecData+1, 1, 10);
	//PRE_RANGE_CONFIG_VCSEL_PERIOD =
	HAL_I2C_Mem_Read(hi2c, VL53L0x_add, VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD, I2C_MEMADD_SIZE_8BIT, VL53L0x_RecData+2, 1, 10);
	//FINAL_RANGE_CONFIG_VCSEL_PERIOD=
	HAL_I2C_Mem_Read(hi2c, VL53L0x_add, VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD, I2C_MEMADD_SIZE_8BIT, VL53L0x_RecData+3, 1, 10);
	*/
	HAL_I2C_Mem_Write(hi2c, VL53L0x_add, VL53L0X_REG_SYSRANGE_START, I2C_MEMADD_SIZE_8BIT, VL53L0x_SendData, 1, 1);
	//HAL_Delay(500);

	VL53L0x_SendData[1] = 100;
	while(VL53L0x_SendData[1]--)
	{
		HAL_Delay(1);
		HAL_I2C_Mem_Read(hi2c, VL53L0x_add, VL53L0X_REG_RESULT_RANGE_STATUS, I2C_MEMADD_SIZE_8BIT, VL53L0x_RecData+4, 1, 1);
			if (VL53L0x_RecData[4] & 0x01) break;
	}
}

 //Check if VL53L0x is normal
 //Return 0 if check is successful
 //Return 1 if the check fails
uint8_t VL53L0x_Check(I2C_HandleTypeDef *hi2c, uint16_t VL53L0x_add)
{

	uint8_t VL53L0x_SendData[2] = {0x01};
	uint8_t VL53L0x_RecData[5];

	//Revision ID:
	HAL_I2C_Mem_Read(hi2c, VL53L0x_add, VL53L0X_REG_IDENTIFICATION_REVISION_ID, I2C_MEMADD_SIZE_8BIT, VL53L0x_RecData, 1, 1);
	//Device ID:
	HAL_I2C_Mem_Read(hi2c, VL53L0x_add, VL53L0X_REG_IDENTIFICATION_MODEL_ID, I2C_MEMADD_SIZE_8BIT, VL53L0x_RecData+1, 1, 1);
	//PRE_RANGE_CONFIG_VCSEL_PERIOD =
	HAL_I2C_Mem_Read(hi2c, VL53L0x_add, VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD, I2C_MEMADD_SIZE_8BIT, VL53L0x_RecData+2, 1, 1);
	//FINAL_RANGE_CONFIG_VCSEL_PERIOD=
	HAL_I2C_Mem_Read(hi2c, VL53L0x_add, VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD, I2C_MEMADD_SIZE_8BIT, VL53L0x_RecData+3, 1, 1);

	HAL_I2C_Mem_Write(hi2c, VL53L0x_add, VL53L0X_REG_SYSRANGE_START, I2C_MEMADD_SIZE_8BIT, VL53L0x_SendData, 1, 1);
	HAL_Delay(500);

	VL53L0x_SendData[1] = 100;
	while(VL53L0x_SendData[1]--)
	{
		HAL_Delay(10);
		HAL_I2C_Mem_Read(hi2c, VL53L0x_add, VL53L0X_REG_RESULT_RANGE_STATUS, I2C_MEMADD_SIZE_8BIT, VL53L0x_RecData+4, 1, 1);
			if (VL53L0x_RecData[4] & 0x01) break;
	}

	if (VL53L0x_RecData[4] & 0x01)
	 return 0; //return 0 if check is successful
	 else return 1; //return 1 if check fails
}

 //VL53L0x convert once
void VL53L0x_StartConversion(I2C_HandleTypeDef *hi2c, uint16_t VL53L0x_add)
{
	uint8_t VL53L0x_SendData[1] = {0x01};
	HAL_I2C_Mem_Write(hi2c, VL53L0x_add, VL53L0X_REG_SYSRANGE_START, I2C_MEMADD_SIZE_8BIT, VL53L0x_SendData, 1, 1);
}

uint16_t makeuint16(int lsb, int msb)
{
    return ((msb & 0xFF) << 8) | (lsb & 0xFF);
}


 //VL53L0x reading distance and other data feedback information
void VL53L0x_ReadDistance(I2C_HandleTypeDef *hi2c, struct VL53L0xData *VL_temp, uint16_t VL53L0x_add)
{
	 //Record historical valid values
	VL_temp->dist_last = VL_temp->distValid;

	HAL_I2C_Mem_Read(hi2c, VL53L0x_add, VL53L0X_REG_RESULT_RANGE_STATUS, I2C_MEMADD_SIZE_8BIT, VL_temp->vtemp, 12, 1);

	VL_temp->acnt = makeuint16(VL_temp->vtemp[7], VL_temp->vtemp[6]);
	VL_temp->scnt = makeuint16(VL_temp->vtemp[9], VL_temp->vtemp[8]);
	VL_temp->dist = makeuint16(VL_temp->vtemp[11], VL_temp->vtemp[10]);
	VL_temp->DeviceRangeStatusInternal = ((VL_temp->vtemp[0] & 0x78) >> 3);


	 //Extract valid value
	 if(VL_temp->dist <= 0x0014) //Distance data is invalid
		VL_temp->distValid = VL_temp->dist_last;
	 else //valid
		VL_temp->distValid = VL_temp->dist;

 /* The code below has been forgotten for a long time. The function is to get the altitude of the aircraft based on the inclination to the ground. You can delete it directly */

	uint8_t i,j;

	for(i = 14;i>0;i--)
	{
		VL_temp->dist_buff[i] = VL_temp->dist_buff[i-1];
	}
	VL_temp->dist_buff[0] = VL_temp->distValid;

	i =0;
	j=0;
	for(i=0;i<6;i++)
	{
		if(VL_temp->dist_buff[i] >=2000)
			j++;
	}

	 if(j >= 2) // 1/3 overflow rate
		VL_temp->distValidFinal = 2000;
	 else //Filter the most recent valid value from the array
	{
		i=0;
		for(i=0;i<15;i++)
		{
			if(VL_temp->dist_buff[i] <2000)
				break;
		}
		VL_temp->distValidFinal = VL_temp->dist_buff[i];
	}

	VL_temp->GroundDis_last = VL_temp->GroundDis;
	VL_temp->GroundDis = (float)VL_temp->distValidFinal /1000;
	//VL_temp->GroundDis = VL_temp->GroundDis *cos(ABS((PostureAngle.Pitch /180 *3.14159))) *cos(ABS((PostureAngle.Roll /180 *3.14159)));
}
