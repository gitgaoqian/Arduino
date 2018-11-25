#include <Servo.h> 
Servo myservo1;  //创建一个舵机控制对象谢谢详细信息详细信息 
Servo myservo2; // 使用Servo类最多可以控制8个舵机
int pos = 0;    // 该变量用与存储舵机角度位置
/*~~~~~~~~~~~~~~~~~~~~~~~~~~华丽的分割线~~~~~~~~~~~~~~~~~~~~~~~~~~ */
void setup() 
{ 
  myservo1.attach(9);
  //myservo2.attach(13);
  myservo1.write(0);
  // 该舵机由arduino第九脚控制
 
} 
/*~~~~~~~~~~~~~~~~~~~~~~~~~~华丽的分割线 ~~~~~~~~~~~~~~~~~~~~~~~~~~ */ 
void loop() 
{ 
  for(pos =0; pos < 180; pos += 1 )  // 从0度到180度运动 
  {                                                     // 每次步进一度
    myservo1.write(pos);  
    //myservo2.write(pos); // 指定舵机转向的角度
 
    delay(25);                       // 等待15ms让舵机到达指定位置
  } 
  for(pos = 180; pos>=0; pos-=1)   //从180度到0度运动  
  {                                
    myservo1.write(pos);  
    //myservo2.write(pos); // 指定舵机转向的角度 
    delay(25);                        // 等待15ms让舵机到达指定位置 
  } 
}


