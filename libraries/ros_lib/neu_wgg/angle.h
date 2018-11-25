#ifndef _ROS_neu_wgg_angle_h
#define _ROS_neu_wgg_angle_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace neu_wgg
{

  class angle : public ros::Msg
  {
    public:
      float leftk;
      float lefth;
      float rightk;
      float righth;

    angle():
      leftk(0),
      lefth(0),
      rightk(0),
      righth(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->leftk);
      offset += serializeAvrFloat64(outbuffer + offset, this->lefth);
      offset += serializeAvrFloat64(outbuffer + offset, this->rightk);
      offset += serializeAvrFloat64(outbuffer + offset, this->righth);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->leftk));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->lefth));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->rightk));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->righth));
     return offset;
    }

    const char * getType(){ return "neu_wgg/angle"; };
    const char * getMD5(){ return "749b5e31e1f49ff0b2d9774f8a10ff1d"; };

  };

}
#endif