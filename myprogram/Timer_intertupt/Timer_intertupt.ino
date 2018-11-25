// led灯接UNO的13管脚
#include <MsTimer2.h>     //定时器库的 头文件 
float time1;
float time2;
 
void flash()                        //中断处理函数，改变灯的状态
{
  time1 = millis();                       
  static boolean output = HIGH;
  digitalWrite(13, output);
  output = !output;
  time2 = millis();
  Serial.println(time2-time1,DEC);
}
 
void setup() 
{
  pinMode(13, OUTPUT);
  Serial.begin(9600);
  MsTimer2::set(500, flash);        // 中断设置函数，每 500ms 进入一次中断
  MsTimer2::start();                //开始计时
}
 
void loop()
{
 
}
