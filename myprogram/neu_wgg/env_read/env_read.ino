#include <dht.h>
#include <ros.h>
#include <neu_wgg/env.h>
dht DHT;
#define DH11_PIN 2
ros::NodeHandle nh;
neu_wgg::env env_msg;
ros::Publisher env_pub("env_topic",&env_msg);
void setup()
{
  nh.initNode();
  nh.advertise(env_pub);
  delay(300);
}
void loop()
{
  DHT.read11(DH11_PIN); 
  env_msg.atmo = 1.5;
  env_msg.temp = DHT.temperature;
  env_msg.hum = DHT.humidity;
  env_pub.publish(&env_msg);
  nh.spinOnce();
  delay(1000);            //每1000ms更新一次  
}
