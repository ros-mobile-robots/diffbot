#ifndef _ROS_diffbot_msgs_AngularVelocities_h
#define _ROS_diffbot_msgs_AngularVelocities_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace diffbot_msgs
{

  class AngularVelocities : public ros::Msg
  {
    public:
      uint32_t joint_length;
      typedef float _joint_type;
      _joint_type st_joint;
      _joint_type * joint;

    AngularVelocities():
      joint_length(0), st_joint(), joint(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->joint_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joint_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joint_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joint_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_length);
      for( uint32_t i = 0; i < joint_length; i++){
      union {
        float real;
        uint32_t base;
      } u_jointi;
      u_jointi.real = this->joint[i];
      *(outbuffer + offset + 0) = (u_jointi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_jointi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_jointi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_jointi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t joint_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      joint_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      joint_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      joint_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->joint_length);
      if(joint_lengthT > joint_length)
        this->joint = (float*)realloc(this->joint, joint_lengthT * sizeof(float));
      joint_length = joint_lengthT;
      for( uint32_t i = 0; i < joint_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_joint;
      u_st_joint.base = 0;
      u_st_joint.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_joint.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_joint.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_joint.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_joint = u_st_joint.real;
      offset += sizeof(this->st_joint);
        memcpy( &(this->joint[i]), &(this->st_joint), sizeof(float));
      }
     return offset;
    }

    virtual const char * getType() override { return "diffbot_msgs/AngularVelocities"; };
    virtual const char * getMD5() override { return "edecb4b6fff50f927a908742515e167a"; };

  };

}
#endif
