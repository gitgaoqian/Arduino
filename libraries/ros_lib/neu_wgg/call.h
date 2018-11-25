#ifndef _ROS_SERVICE_call_h
#define _ROS_SERVICE_call_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace neu_wgg
{

static const char CALL[] = "neu_wgg/call";

  class callRequest : public ros::Msg
  {
    public:
      const char* number;

    callRequest():
      number("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_number = strlen(this->number);
      memcpy(outbuffer + offset, &length_number, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->number, length_number);
      offset += length_number;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_number;
      memcpy(&length_number, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_number; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_number-1]=0;
      this->number = (char *)(inbuffer + offset-1);
      offset += length_number;
     return offset;
    }

    const char * getType(){ return CALL; };
    const char * getMD5(){ return "1390643684d42806cde16cb777d15235"; };

  };

  class callResponse : public ros::Msg
  {
    public:
      const char* info;

    callResponse():
      info("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_info = strlen(this->info);
      memcpy(outbuffer + offset, &length_info, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->info, length_info);
      offset += length_info;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_info;
      memcpy(&length_info, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_info; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_info-1]=0;
      this->info = (char *)(inbuffer + offset-1);
      offset += length_info;
     return offset;
    }

    const char * getType(){ return CALL; };
    const char * getMD5(){ return "c10fc26d5cca9a4b9114f5fc5dea9570"; };

  };

  class call {
    public:
    typedef callRequest Request;
    typedef callResponse Response;
  };

}
#endif
