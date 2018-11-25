 #include <Wire.h>//声明I2C库文件
#define LED 13
byte x = 0;//变量x决定LED的亮灭
//初始化
void setup()
{
  Wire.begin(); // 加入 i2c 总线，作为主机
  pinMode(LED,OUTPUT);//设置数字端口13为输出
  Serial.begin(115200);
}
//主程序
void loop()
{
  Wire.beginTransmission(114); //发送数据到设备号为114的从机
      // 发送字符串"light is "
  Wire.write(x);              // 发送变量x中的一个字节  
  int error = Wire.endTransmission();
     
  Serial.print("Result\t");
  Serial.println(error);   

  x++;//变量x加1
  if(x==2)//如果变量x的值为2，则把x值转为0
  x=0;
  delay(1000);//延时1s
  
  
}
 
