#ifndef _ROS_diffbot_msgs_WheelsCmdStamped_h
#define _ROS_diffbot_msgs_WheelsCmdStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "diffbot_msgs/WheelsCmd.h"

namespace diffbot_msgs
{

  class WheelsCmdStamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef diffbot_msgs::WheelsCmd _wheels_cmd_type;
      _wheels_cmd_type wheels_cmd;

    WheelsCmdStamped():
      header(),
      wheels_cmd()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->wheels_cmd.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->wheels_cmd.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "diffbot_msgs/WheelsCmdStamped"; };
    virtual const char * getMD5() override { return "0a7314ac2bc5413723ab6ac388330f3c"; };

  };

}
#endif
