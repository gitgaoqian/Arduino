#ifndef _ROS_base_control_wheelspeed_h
#define _ROS_base_control_wheelspeed_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace base_control
{

  class wheelspeed : public ros::Msg
  {
    public:
      float n1;
      float n2;
      float n3;

    wheelspeed():
      n1(0),
      n2(0),
      n3(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_n1;
      u_n1.real = this->n1;
      *(outbuffer + offset + 0) = (u_n1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_n1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_n1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_n1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->n1);
      union {
        float real;
        uint32_t base;
      } u_n2;
      u_n2.real = this->n2;
      *(outbuffer + offset + 0) = (u_n2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_n2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_n2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_n2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->n2);
      union {
        float real;
        uint32_t base;
      } u_n3;
      u_n3.real = this->n3;
      *(outbuffer + offset + 0) = (u_n3.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_n3.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_n3.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_n3.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->n3);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_n1;
      u_n1.base = 0;
      u_n1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_n1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_n1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_n1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->n1 = u_n1.real;
      offset += sizeof(this->n1);
      union {
        float real;
        uint32_t base;
      } u_n2;
      u_n2.base = 0;
      u_n2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_n2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_n2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_n2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->n2 = u_n2.real;
      offset += sizeof(this->n2);
      union {
        float real;
        uint32_t base;
      } u_n3;
      u_n3.base = 0;
      u_n3.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_n3.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_n3.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_n3.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->n3 = u_n3.real;
      offset += sizeof(this->n3);
     return offset;
    }

    const char * getType(){ return "base_control/wheelspeed"; };
    const char * getMD5(){ return "d8b5fc3e1dc1e96e761730666e64d9f6"; };

  };

}
#endif