#ifndef _ROS_SERVICE_call_h
#define _ROS_SERVICE_call_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace neuro_cloud
{

static const char CALL[] = "neuro_cloud/call";

  class callRequest : public ros::Msg
  {
    public:
      const char* service_name;
      const char* action_name;

    callRequest():
      service_name(""),
      action_name("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_service_name = strlen(this->service_name);
      memcpy(outbuffer + offset, &length_service_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->service_name, length_service_name);
      offset += length_service_name;
      uint32_t length_action_name = strlen(this->action_name);
      memcpy(outbuffer + offset, &length_action_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->action_name, length_action_name);
      offset += length_action_name;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_service_name;
      memcpy(&length_service_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_service_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_service_name-1]=0;
      this->service_name = (char *)(inbuffer + offset-1);
      offset += length_service_name;
      uint32_t length_action_name;
      memcpy(&length_action_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_action_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_action_name-1]=0;
      this->action_name = (char *)(inbuffer + offset-1);
      offset += length_action_name;
     return offset;
    }

    const char * getType(){ return CALL; };
    const char * getMD5(){ return "e0407c0da6e0c34fefc93ce6ef17fb83"; };

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
