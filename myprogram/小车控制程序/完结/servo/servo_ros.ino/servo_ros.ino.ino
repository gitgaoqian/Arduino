#include <Servo.h> 
#include <ros.h>
#include <std_msgs/Int8.h>

ros::NodeHandle  nh;

Servo servo;
int degree = 90;

void servo_cb( const std_msgs::Int8& cmd_msg){
  if (cmd_msg.data == 0) // the object not detected
  {
     if (degree >= 90  and degree < 180)
     {
        degree = degree+1;
        servo.write(degree);//turn left first
      }
      if (degree == 180)
      {
        degree = 89;
        }
   
      if (degree <90 and degree >0)
      {
        degree = degree-1;
        servo.write(degree);//turn right 
        }
      if (degree == 0)
        {
          degree = 90;
          }
    }
    else 
    {
      servo.detach();
      }
  
}


ros::Subscriber<std_msgs::Int8> sub("servo", servo_cb);

void setup(){
  
  nh.initNode();
  nh.subscribe(sub);
  
  servo.attach(9); //attach it to pin 9
  servo.write(90);
}

void loop(){
  nh.spinOnce();
  delay(1);
}
