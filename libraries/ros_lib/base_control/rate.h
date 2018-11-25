#ifndef _ROS_base_control_rate_h
#define _ROS_base_control_rate_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace base_control
{

  class rate : public ros::Msg
  {
    public:
      float rate;

    rate():
      rate(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->rate);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->rate));
     return offset;
    }

    const char * getType(){ return "base_control/rate"; };
    const char * getMD5(){ return "4910f3d55cbb29566b6c8f8f16528adf"; };

  };

}
#endif