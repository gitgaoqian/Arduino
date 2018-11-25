#ifndef _ROS_SERVICE_call3_h
#define _ROS_SERVICE_call3_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace neu_wgg
{

static const char CALL3[] = "neu_wgg/call3";

  class call3Request : public ros::Msg
  {
    public:
      const char* number;
      const char* action;

    call3Request():
      number(""),
      action("")
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
      uint32_t length_action = strlen(this->action);
      memcpy(outbuffer + offset, &length_action, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->action, length_action);
      offset += length_action;
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
      uint32_t length_action;
      memcpy(&length_action, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_action; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_action-1]=0;
      this->action = (char *)(inbuffer + offset-1);
      offset += length_action;
     return offset;
    }

    const char * getType(){ return CALL3; };
    const char * getMD5(){ return "e97aeb980498b8863d1b9b7f1a351fe6"; };

  };

  class call3Response : public ros::Msg
  {
    public:
      const char* info;

    call3Response():
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

    const char * getType(){ return CALL3; };
    const char * getMD5(){ return "c10fc26d5cca9a4b9114f5fc5dea9570"; };

  };

  class call3 {
    public:
    typedef call3Request Request;
    typedef call3Response Response;
  };

}
#endif
