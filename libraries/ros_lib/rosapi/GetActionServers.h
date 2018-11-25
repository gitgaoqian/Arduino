#ifndef _ROS_SERVICE_GetActionServers_h
#define _ROS_SERVICE_GetActionServers_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rosapi
{

static const char GETACTIONSERVERS[] = "rosapi/GetActionServers";

  class GetActionServersRequest : public ros::Msg
  {
    public:

    GetActionServersRequest()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return GETACTIONSERVERS; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GetActionServersResponse : public ros::Msg
  {
    public:
      uint8_t action_servers_length;
      char* st_action_servers;
      char* * action_servers;

    GetActionServersResponse():
      action_servers_length(0), action_servers(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = action_servers_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < action_servers_length; i++){
      uint32_t length_action_serversi = strlen(this->action_servers[i]);
      memcpy(outbuffer + offset, &length_action_serversi, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->action_servers[i], length_action_serversi);
      offset += length_action_serversi;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t action_servers_lengthT = *(inbuffer + offset++);
      if(action_servers_lengthT > action_servers_length)
        this->action_servers = (char**)realloc(this->action_servers, action_servers_lengthT * sizeof(char*));
      offset += 3;
      action_servers_length = action_servers_lengthT;
      for( uint8_t i = 0; i < action_servers_length; i++){
      uint32_t length_st_action_servers;
      memcpy(&length_st_action_servers, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_action_servers; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_action_servers-1]=0;
      this->st_action_servers = (char *)(inbuffer + offset-1);
      offset += length_st_action_servers;
        memcpy( &(this->action_servers[i]), &(this->st_action_servers), sizeof(char*));
      }
     return offset;
    }

    const char * getType(){ return GETACTIONSERVERS; };
    const char * getMD5(){ return "46807ba271844ac5ba4730a47556b236"; };

  };

  class GetActionServers {
    public:
    typedef GetActionServersRequest Request;
    typedef GetActionServersResponse Response;
  };

}
#endif
