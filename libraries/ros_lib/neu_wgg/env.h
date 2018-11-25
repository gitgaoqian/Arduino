#ifndef _ROS_neu_wgg_env_h
#define _ROS_neu_wgg_env_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace neu_wgg
{

  class env : public ros::Msg
  {
    public:
      float temp;
      float hum;
      float atmo;

    env():
      temp(0),
      hum(0),
      atmo(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->temp);
      offset += serializeAvrFloat64(outbuffer + offset, this->hum);
      offset += serializeAvrFloat64(outbuffer + offset, this->atmo);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->temp));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->hum));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->atmo));
     return offset;
    }

    const char * getType(){ return "neu_wgg/env"; };
    const char * getMD5(){ return "e1c56a32ed8910ed1e2abbeaae156fb5"; };

  };

}
#endif