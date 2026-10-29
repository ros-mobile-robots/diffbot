#ifndef _ROS_diffbot_msgs_AngularVelocitiesStamped_h
#define _ROS_diffbot_msgs_AngularVelocitiesStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "diffbot_msgs/AngularVelocities.h"

namespace diffbot_msgs
{

  class AngularVelocitiesStamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef diffbot_msgs::AngularVelocities _angular_velocities_type;
      _angular_velocities_type angular_velocities;

    AngularVelocitiesStamped():
      header(),
      angular_velocities()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->angular_velocities.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->angular_velocities.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "diffbot_msgs/AngularVelocitiesStamped"; };
    virtual const char * getMD5() override { return "62b2cae704bd641c35dea902c2913994"; };

  };

}
#endif
