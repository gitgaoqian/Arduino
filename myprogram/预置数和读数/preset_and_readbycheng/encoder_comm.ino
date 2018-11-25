//串口通信编程思路：预置数时，上位机发送７个字节，第一个字节是命令符，后六个分别３个字节给counter1,counter2;
//counter1,counter2的长度必须至少为２４位，此处选择长整形（３２位）;
//串口通信每次只能从缓存区读取１个字节，我们的目的是将３个字节依次放到counter的1-8,9-16,17-24的位置上，（参考c语言指针转换：字符指针访问多字节整形的字节）
//所以需要设置一个字符类型（单子节）的指针，指针指向counter的首地址（即指向该类型的第一个字节）

//设置AB相中断输入引脚
#define MA2 2
#define MB2 3//电机2的A相B相输入引脚的定义:外部中断4 5*/
#define MA1 21
#define MB1 20 //电机1的A相B相输入引脚的定义:外部中断2 3*/
//设置一些变量
long counter1=0;//用来储存电机1的编码器记录的脉冲数*/
long counter2=0;//用来储存电机2的编码器记录的脉冲数，使用long的有符号整型
unsigned char Buffer[8];//缓存上位机发送的字节;此处必须用unsigned char,因为char的范围是-127-128所以Buffer的值就达不到0XFF．
int slot=0;//记录接受上位机发送的字节数
unsigned char * pc1, * pc2;//定义指针，指向char类型，说明每次指针取值只能取一个字节
unsigned long time_stamp;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200,SERIAL_8N1);
  attachInterrupt(2, doEncoderA1, RISING);//电机1的编码器的A相shang sheng yan du qu
  attachInterrupt(0, doEncoderA2, RISING);//电机2的编码器的A相
  pc1= (unsigned char *)&counter1;//指针类型的转换，利用char类型指针访问多字节整形变量的字节
  pc2= (unsigned char *)&counter2;
  time_stamp = millis();
}

void loop() {
    //如果10ms未收到数据，清空缓冲区
    if ((millis()-time_stamp)>10)
      slot=0;
      
    while(Serial.available()>0)
    {
        Buffer[slot++]=Serial.read();
        //增加安全性，防止意外数据输入造成内存泄露
        if (slot>=32)
           slot=0;
        time_stamp = millis();
    }

      
    if(slot>0)
      if(is_cmd_valid())
          preset_and_send();
}

bool is_cmd_valid()//判断命令符的有效性
{
  if((slot==1)&(Buffer[0]==0xFF))
    return true;
    
  if((slot==7)&(Buffer[0]==0x00))
    return true;
   //如果命令码出错或长度过大，清空
  if((Buffer[0]!=0x00)|(slot>7))
    slot=0;
  //如果命令码对，但长度不够，返回false继续等待
  return false;
}
void preset_and_send()
{
      if(Buffer[0]==0x00)
      { 
          //这里根据不同电脑字节顺序进行适配
           pc1[0]=Buffer[3];
           pc1[1]=Buffer[2];
           pc1[2]=Buffer[1];
           //补码符号位扩展，解决long 32位与24位数据之间符号问题，即实现既能预置正数又能预置负数．关于负数要深刻理解计算机中负数的二进制表达方式
           if(Buffer[1]<128)
              pc1[3]=0;
           else
              pc1[3]=0xFF;
            
           pc2[0]=Buffer[6];
           pc2[1]=Buffer[5];
           pc2[2]=Buffer[4];
           if(Buffer[4]<128)
              pc2[3]=0;
           else
              pc2[3]=0xFF;
                                    
           Serial.println(counter1);
           Serial.println(counter2);      
           slot=0;
           return;     
        }
        if(Buffer[0]==0xff)
        {
          Serial.write(pc1,3);
          Serial.write(pc2,3);
          slot=0;
          return;          
        }
}

//第一个编码器计数
void doEncoderA1(){

  // look for a low-to-high on channel A
  if (digitalRead(MB1) == LOW)  
      counter1 = counter1 + 1;         // CW
  else 
      counter1 = counter1 - 1;    // CCW
      
    
  //Serial.println (counter1, DEC); 
          

  
}
//第2个编码器计数
void doEncoderA2(){
  // look for a low-to-high on channel A
  if (digitalRead(MB2) == LOW) 
      counter2 = counter2 + 1;         // CW
  else 
      counter2 = counter2 - 1;         // CCW
 
  
}


