#ifndef _ROS_neu_wgg_location_h
#define _ROS_neu_wgg_location_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace neu_wgg
{

  class location : public ros::Msg
  {
    public:
      float longitude;
      float latitude;

    location():
      longitude(0),
      latitude(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->longitude);
      offset += serializeAvrFloat64(outbuffer + offset, this->latitude);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->longitude));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->latitude));
     return offset;
    }

    const char * getType(){ return "neu_wgg/location"; };
    const char * getMD5(){ return "fd6c3d0b911e124b1f0b5a2ade9c1a01"; };

  };

}
#endif