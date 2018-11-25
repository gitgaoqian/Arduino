#ifndef _ROS_base_control_carspeed_h
#define _ROS_base_control_carspeed_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace base_control
{

  class carspeed : public ros::Msg
  {
    public:
      float vx;
      float vy;
      float vth;

    carspeed():
      vx(0),
      vy(0),
      vth(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_vx;
      u_vx.real = this->vx;
      *(outbuffer + offset + 0) = (u_vx.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_vx.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_vx.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_vx.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->vx);
      union {
        float real;
        uint32_t base;
      } u_vy;
      u_vy.real = this->vy;
      *(outbuffer + offset + 0) = (u_vy.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_vy.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_vy.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_vy.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->vy);
      union {
        float real;
        uint32_t base;
      } u_vth;
      u_vth.real = this->vth;
      *(outbuffer + offset + 0) = (u_vth.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_vth.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_vth.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_vth.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->vth);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_vx;
      u_vx.base = 0;
      u_vx.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_vx.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_vx.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_vx.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->vx = u_vx.real;
      offset += sizeof(this->vx);
      union {
        float real;
        uint32_t base;
      } u_vy;
      u_vy.base = 0;
      u_vy.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_vy.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_vy.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_vy.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->vy = u_vy.real;
      offset += sizeof(this->vy);
      union {
        float real;
        uint32_t base;
      } u_vth;
      u_vth.base = 0;
      u_vth.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_vth.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_vth.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_vth.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->vth = u_vth.real;
      offset += sizeof(this->vth);
     return offset;
    }

    const char * getType(){ return "base_control/carspeed"; };
    const char * getMD5(){ return "2ed912b0fd0de10da7b827f569b8b385"; };

  };

}
#endif