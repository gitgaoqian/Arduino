#ifndef _ROS_graph_msgs_Edges_h
#define _ROS_graph_msgs_Edges_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace graph_msgs
{

  class Edges : public ros::Msg
  {
    public:
      uint8_t node_ids_length;
      uint32_t st_node_ids;
      uint32_t * node_ids;
      uint8_t weights_length;
      float st_weights;
      float * weights;

    Edges():
      node_ids_length(0), node_ids(NULL),
      weights_length(0), weights(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = node_ids_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < node_ids_length; i++){
      *(outbuffer + offset + 0) = (this->node_ids[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->node_ids[i] >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->node_ids[i] >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->node_ids[i] >> (8 * 3)) & 0xFF;
      offset += sizeof(this->node_ids[i]);
      }
      *(outbuffer + offset++) = weights_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < weights_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->weights[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t node_ids_lengthT = *(inbuffer + offset++);
      if(node_ids_lengthT > node_ids_length)
        this->node_ids = (uint32_t*)realloc(this->node_ids, node_ids_lengthT * sizeof(uint32_t));
      offset += 3;
      node_ids_length = node_ids_lengthT;
      for( uint8_t i = 0; i < node_ids_length; i++){
      this->st_node_ids =  ((uint32_t) (*(inbuffer + offset)));
      this->st_node_ids |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_node_ids |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->st_node_ids |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->st_node_ids);
        memcpy( &(this->node_ids[i]), &(this->st_node_ids), sizeof(uint32_t));
      }
      uint8_t weights_lengthT = *(inbuffer + offset++);
      if(weights_lengthT > weights_length)
        this->weights = (float*)realloc(this->weights, weights_lengthT * sizeof(float));
      offset += 3;
      weights_length = weights_lengthT;
      for( uint8_t i = 0; i < weights_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_weights));
        memcpy( &(this->weights[i]), &(this->st_weights), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "graph_msgs/Edges"; };
    const char * getMD5(){ return "1dcd54afd0b0c0fbebeb59dbdda4c026"; };

  };

}
#endif