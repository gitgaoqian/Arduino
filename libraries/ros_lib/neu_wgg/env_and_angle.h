#ifndef _ROS_neu_wgg_env_and_angle_h
#define _ROS_neu_wgg_env_and_angle_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace neu_wgg
{

  class env_and_angle : public ros::Msg
  {
    public:
      float leftk;
      float lefth;
      float rightk;
      float righth;
      float temp;
      float hum;
      float atmo;
      float longitude;
      float latitude;

    env_and_angle():
      leftk(0),
      lefth(0),
      rightk(0),
      righth(0),
      temp(0),
      hum(0),
      atmo(0),
      longitude(0),
      latitude(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->leftk);
      offset += serializeAvrFloat64(outbuffer + offset, this->lefth);
      offset += serializeAvrFloat64(outbuffer + offset, this->rightk);
      offset += serializeAvrFloat64(outbuffer + offset, this->righth);
      offset += serializeAvrFloat64(outbuffer + offset, this->temp);
      offset += serializeAvrFloat64(outbuffer + offset, this->hum);
      offset += serializeAvrFloat64(outbuffer + offset, this->atmo);
      offset += serializeAvrFloat64(outbuffer + offset, this->longitude);
      offset += serializeAvrFloat64(outbuffer + offset, this->latitude);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->leftk));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->lefth));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->rightk));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->righth));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->temp));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->hum));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->atmo));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->longitude));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->latitude));
     return offset;
    }

    const char * getType(){ return "neu_wgg/env_and_angle"; };
    const char * getMD5(){ return "520b31be016734af5220a77b89cb3678"; };

  };

}
#endif