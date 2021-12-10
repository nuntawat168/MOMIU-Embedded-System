#ifndef _ROS_momiu_p_pose_message_h
#define _ROS_momiu_p_pose_message_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace momiu_p
{

  class pose_message : public ros::Msg
  {
    public:
      typedef const char* _pose_type;
      _pose_type pose;
      typedef uint8_t _intensity_type;
      _intensity_type intensity;
      typedef uint8_t _loop_type;
      _loop_type loop;
      typedef const char* _face_type;
      _face_type face;

    pose_message():
      pose(""),
      intensity(0),
      loop(0),
      face("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_pose = strlen(this->pose);
      varToArr(outbuffer + offset, length_pose);
      offset += 4;
      memcpy(outbuffer + offset, this->pose, length_pose);
      offset += length_pose;
      *(outbuffer + offset + 0) = (this->intensity >> (8 * 0)) & 0xFF;
      offset += sizeof(this->intensity);
      *(outbuffer + offset + 0) = (this->loop >> (8 * 0)) & 0xFF;
      offset += sizeof(this->loop);
      uint32_t length_face = strlen(this->face);
      varToArr(outbuffer + offset, length_face);
      offset += 4;
      memcpy(outbuffer + offset, this->face, length_face);
      offset += length_face;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_pose;
      arrToVar(length_pose, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_pose; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_pose-1]=0;
      this->pose = (char *)(inbuffer + offset-1);
      offset += length_pose;
      this->intensity =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->intensity);
      this->loop =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->loop);
      uint32_t length_face;
      arrToVar(length_face, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_face; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_face-1]=0;
      this->face = (char *)(inbuffer + offset-1);
      offset += length_face;
     return offset;
    }

    virtual const char * getType() override { return "momiu_p/pose_message"; };
    virtual const char * getMD5() override { return "3b993a6445fc8aa1faa13f3e6553ef3a"; };

  };

}
#endif
