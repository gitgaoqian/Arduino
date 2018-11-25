#ifndef _ROS_base_control_addtion_h
#define _ROS_base_control_addtion_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace base_control
{

  class addtion : public ros::Msg
  {
    public:
      float x;
      float y;

    addtion():
      x(0),
      y(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->x);
      offset += serializeAvrFloat64(outbuffer + offset, this->y);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->x));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->y));
     return offset;
    }

    const char * getType(){ return "base_control/addtion"; };
    const char * getMD5(){ return "209f516d3eb691f0663e25cb750d67c1"; };

  };

}
#endif