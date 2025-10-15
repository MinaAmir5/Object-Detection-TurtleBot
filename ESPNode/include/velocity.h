#ifndef _ROS_your_package_velocity_h
#define _ROS_your_package_velocity_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ArduinoIncludes.h"
#include "std_msgs/Float64.h"


namespace your_package
{

  class velocity : public ros::Msg
  {
    public:
      typedef std_msgs::Float64 _right_wheel_velocity_type;
      _right_wheel_velocity_type right_wheel_velocity;
      typedef std_msgs::Float64 _left_wheel_velocity_type;
      _left_wheel_velocity_type left_wheel_velocity;

    velocity():
      right_wheel_velocity(),
      left_wheel_velocity()
    {
    }

   virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->right_wheel_velocity.serialize(outbuffer + offset);
      offset += this->left_wheel_velocity.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->right_wheel_velocity.deserialize(inbuffer + offset);
      offset += this->left_wheel_velocity.deserialize(inbuffer + offset);
     return offset;
    }
     

    const char * getType() override { return "your_package/velocity"; };
    const char * getMD5() override { return "d06c4e4b7ff480e04a1e8a5e9b1be44b"; };

  };

}
#endif