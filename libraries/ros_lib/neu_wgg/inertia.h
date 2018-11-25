#ifndef _ROS_neu_wgg_inertia_h
#define _ROS_neu_wgg_inertia_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace neu_wgg
{

  class inertia : public ros::Msg
  {
    public:
      float angle_x;
      float angle_y;
      float angle_z;

    inertia():
      angle_x(0),
      angle_y(0),
      angle_z(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->angle_x);
      offset += serializeAvrFloat64(outbuffer + offset, this->angle_y);
      offset += serializeAvrFloat64(outbuffer + offset, this->angle_z);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->angle_x));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->angle_y));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->angle_z));
     return offset;
    }

    const char * getType(){ return "neu_wgg/inertia"; };
    const char * getMD5(){ return "ec276285911cbb4ca8337124cafa807e"; };

  };

}
#endif