#ifndef _ROS_SERVICE_SelectMode_h
#define _ROS_SERVICE_SelectMode_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace momiu_p
{

static const char SELECTMODE[] = "momiu_p/SelectMode";

  class SelectModeRequest : public ros::Msg
  {
    public:
      typedef bool _SelectMode_type;
      _SelectMode_type SelectMode;

    SelectModeRequest():
      SelectMode(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_SelectMode;
      u_SelectMode.real = this->SelectMode;
      *(outbuffer + offset + 0) = (u_SelectMode.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->SelectMode);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_SelectMode;
      u_SelectMode.base = 0;
      u_SelectMode.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->SelectMode = u_SelectMode.real;
      offset += sizeof(this->SelectMode);
     return offset;
    }

    virtual const char * getType() override { return SELECTMODE; };
    virtual const char * getMD5() override { return "b1265bbb66b88637ede62c0463b5c0e6"; };

  };

  class SelectModeResponse : public ros::Msg
  {
    public:
      typedef const char* _respond_msg_type;
      _respond_msg_type respond_msg;

    SelectModeResponse():
      respond_msg("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_respond_msg = strlen(this->respond_msg);
      varToArr(outbuffer + offset, length_respond_msg);
      offset += 4;
      memcpy(outbuffer + offset, this->respond_msg, length_respond_msg);
      offset += length_respond_msg;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_respond_msg;
      arrToVar(length_respond_msg, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_respond_msg; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_respond_msg-1]=0;
      this->respond_msg = (char *)(inbuffer + offset-1);
      offset += length_respond_msg;
     return offset;
    }

    virtual const char * getType() override { return SELECTMODE; };
    virtual const char * getMD5() override { return "45a2bd61f0ae079e3089a6c84ffed3e8"; };

  };

  class SelectMode {
    public:
    typedef SelectModeRequest Request;
    typedef SelectModeResponse Response;
  };

}
#endif
