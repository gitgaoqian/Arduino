#ifndef _ROS_neu_wgg_sensor_h
#define _ROS_neu_wgg_sensor_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace neu_wgg
{

  class sensor : public ros::Msg
  {
    public:
      float temp;
      float hum;
      float atmo;
      float angle_x;
      float angle_y;
      float angle_z;
      float longitude;
      float latitude;

    sensor():
      temp(0),
      hum(0),
      atmo(0),
      angle_x(0),
      angle_y(0),
      angle_z(0),
      longitude(0),
      latitude(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->temp);
      offset += serializeAvrFloat64(outbuffer + offset, this->hum);
      offset += serializeAvrFloat64(outbuffer + offset, this->atmo);
      offset += serializeAvrFloat64(outbuffer + offset, this->angle_x);
      offset += serializeAvrFloat64(outbuffer + offset, this->angle_y);
      offset += serializeAvrFloat64(outbuffer + offset, this->angle_z);
      offset += serializeAvrFloat64(outbuffer + offset, this->longitude);
      offset += serializeAvrFloat64(outbuffer + offset, this->latitude);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->temp));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->hum));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->atmo));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->angle_x));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->angle_y));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->angle_z));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->longitude));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->latitude));
     return offset;
    }

    const char * getType(){ return "neu_wgg/sensor"; };
    const char * getMD5(){ return "93aaeca2f7449bbd83b670bcb5c6ec10"; };

  };

}
#endif