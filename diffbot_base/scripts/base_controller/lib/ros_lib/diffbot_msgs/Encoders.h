#ifndef _ROS_diffbot_msgs_Encoders_h
#define _ROS_diffbot_msgs_Encoders_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace diffbot_msgs
{

  class Encoders : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      int32_t ticks[2];

    Encoders():
      header(),
      ticks()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      for( uint32_t i = 0; i < 2; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_ticksi;
      u_ticksi.real = this->ticks[i];
      *(outbuffer + offset + 0) = (u_ticksi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ticksi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ticksi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ticksi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ticks[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      for( uint32_t i = 0; i < 2; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_ticksi;
      u_ticksi.base = 0;
      u_ticksi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ticksi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ticksi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ticksi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ticks[i] = u_ticksi.real;
      offset += sizeof(this->ticks[i]);
      }
     return offset;
    }

    virtual const char * getType() override { return "diffbot_msgs/Encoders"; };
    virtual const char * getMD5() override { return "8ae22cff29caeebdd5f3ad8360764a8a"; };

  };

}
#endif
