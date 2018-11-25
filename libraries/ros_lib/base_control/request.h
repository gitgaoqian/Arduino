#ifndef _ROS_SERVICE_request_h
#define _ROS_SERVICE_request_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace base_control
{

static const char REQUEST[] = "base_control/request";

  class requestRequest : public ros::Msg
  {
    public:
      const char* req;

    requestRequest():
      req("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_req = strlen(this->req);
      memcpy(outbuffer + offset, &length_req, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->req, length_req);
      offset += length_req;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_req;
      memcpy(&length_req, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_req; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_req-1]=0;
      this->req = (char *)(inbuffer + offset-1);
      offset += length_req;
     return offset;
    }

    const char * getType(){ return REQUEST; };
    const char * getMD5(){ return "b8dc53fbc3707f169aa5dbf7b19d2567"; };

  };

  class requestResponse : public ros::Msg
  {
    public:
      const char* rep;

    requestResponse():
      rep("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_rep = strlen(this->rep);
      memcpy(outbuffer + offset, &length_rep, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->rep, length_rep);
      offset += length_rep;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_rep;
      memcpy(&length_rep, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_rep; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_rep-1]=0;
      this->rep = (char *)(inbuffer + offset-1);
      offset += length_rep;
     return offset;
    }

    const char * getType(){ return REQUEST; };
    const char * getMD5(){ return "9668706e171011f89a41f8bb5fa19076"; };

  };

  class request {
    public:
    typedef requestRequest Request;
    typedef requestResponse Response;
  };

}
#endif
