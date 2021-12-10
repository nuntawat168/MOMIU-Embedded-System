#ifndef _ROS_momiu_p_momiu_posMotor_h
#define _ROS_momiu_p_momiu_posMotor_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace momiu_p
{

  class momiu_posMotor : public ros::Msg
  {
    public:
      typedef int16_t _motor1_pos_type;
      _motor1_pos_type motor1_pos;
      typedef int16_t _motor2_pos_type;
      _motor2_pos_type motor2_pos;
      typedef int16_t _motor3_pos_type;
      _motor3_pos_type motor3_pos;
      typedef int16_t _motor4_pos_type;
      _motor4_pos_type motor4_pos;
      typedef int16_t _motor5_pos_type;
      _motor5_pos_type motor5_pos;
      typedef int16_t _motor6_pos_type;
      _motor6_pos_type motor6_pos;
      typedef int16_t _motor7_pos_type;
      _motor7_pos_type motor7_pos;
      typedef int16_t _motor8_pos_type;
      _motor8_pos_type motor8_pos;

    momiu_posMotor():
      motor1_pos(0),
      motor2_pos(0),
      motor3_pos(0),
      motor4_pos(0),
      motor5_pos(0),
      motor6_pos(0),
      motor7_pos(0),
      motor8_pos(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_motor1_pos;
      u_motor1_pos.real = this->motor1_pos;
      *(outbuffer + offset + 0) = (u_motor1_pos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_motor1_pos.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->motor1_pos);
      union {
        int16_t real;
        uint16_t base;
      } u_motor2_pos;
      u_motor2_pos.real = this->motor2_pos;
      *(outbuffer + offset + 0) = (u_motor2_pos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_motor2_pos.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->motor2_pos);
      union {
        int16_t real;
        uint16_t base;
      } u_motor3_pos;
      u_motor3_pos.real = this->motor3_pos;
      *(outbuffer + offset + 0) = (u_motor3_pos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_motor3_pos.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->motor3_pos);
      union {
        int16_t real;
        uint16_t base;
      } u_motor4_pos;
      u_motor4_pos.real = this->motor4_pos;
      *(outbuffer + offset + 0) = (u_motor4_pos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_motor4_pos.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->motor4_pos);
      union {
        int16_t real;
        uint16_t base;
      } u_motor5_pos;
      u_motor5_pos.real = this->motor5_pos;
      *(outbuffer + offset + 0) = (u_motor5_pos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_motor5_pos.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->motor5_pos);
      union {
        int16_t real;
        uint16_t base;
      } u_motor6_pos;
      u_motor6_pos.real = this->motor6_pos;
      *(outbuffer + offset + 0) = (u_motor6_pos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_motor6_pos.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->motor6_pos);
      union {
        int16_t real;
        uint16_t base;
      } u_motor7_pos;
      u_motor7_pos.real = this->motor7_pos;
      *(outbuffer + offset + 0) = (u_motor7_pos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_motor7_pos.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->motor7_pos);
      union {
        int16_t real;
        uint16_t base;
      } u_motor8_pos;
      u_motor8_pos.real = this->motor8_pos;
      *(outbuffer + offset + 0) = (u_motor8_pos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_motor8_pos.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->motor8_pos);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_motor1_pos;
      u_motor1_pos.base = 0;
      u_motor1_pos.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_motor1_pos.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->motor1_pos = u_motor1_pos.real;
      offset += sizeof(this->motor1_pos);
      union {
        int16_t real;
        uint16_t base;
      } u_motor2_pos;
      u_motor2_pos.base = 0;
      u_motor2_pos.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_motor2_pos.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->motor2_pos = u_motor2_pos.real;
      offset += sizeof(this->motor2_pos);
      union {
        int16_t real;
        uint16_t base;
      } u_motor3_pos;
      u_motor3_pos.base = 0;
      u_motor3_pos.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_motor3_pos.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->motor3_pos = u_motor3_pos.real;
      offset += sizeof(this->motor3_pos);
      union {
        int16_t real;
        uint16_t base;
      } u_motor4_pos;
      u_motor4_pos.base = 0;
      u_motor4_pos.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_motor4_pos.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->motor4_pos = u_motor4_pos.real;
      offset += sizeof(this->motor4_pos);
      union {
        int16_t real;
        uint16_t base;
      } u_motor5_pos;
      u_motor5_pos.base = 0;
      u_motor5_pos.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_motor5_pos.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->motor5_pos = u_motor5_pos.real;
      offset += sizeof(this->motor5_pos);
      union {
        int16_t real;
        uint16_t base;
      } u_motor6_pos;
      u_motor6_pos.base = 0;
      u_motor6_pos.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_motor6_pos.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->motor6_pos = u_motor6_pos.real;
      offset += sizeof(this->motor6_pos);
      union {
        int16_t real;
        uint16_t base;
      } u_motor7_pos;
      u_motor7_pos.base = 0;
      u_motor7_pos.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_motor7_pos.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->motor7_pos = u_motor7_pos.real;
      offset += sizeof(this->motor7_pos);
      union {
        int16_t real;
        uint16_t base;
      } u_motor8_pos;
      u_motor8_pos.base = 0;
      u_motor8_pos.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_motor8_pos.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->motor8_pos = u_motor8_pos.real;
      offset += sizeof(this->motor8_pos);
     return offset;
    }

    virtual const char * getType() override { return "momiu_p/momiu_posMotor"; };
    virtual const char * getMD5() override { return "d0c783b9220b33d08756c95682928665"; };

  };

}
#endif
