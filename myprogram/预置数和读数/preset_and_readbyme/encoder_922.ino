//设置AB相中断输入引脚
#define MA2 2
#define MB2 3//电机2的A相B相输入引脚的定义:外部中断4 5*/
#define MA1 21
#define MB1 20 //电机1的A相B相输入引脚的定义:外部中断2 3*/
//设置一些变量
long int counter1=0;//用来储存电机1的编码器记录的脉冲数*/
long int counter2=0;//用来储存电机2的编码器记录的脉冲数
unsigned char Buffer[7];
int i=0;
int b1=10,b2=11,b3=12,b4=13,b5=14,b6=15;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200,SERIAL_8N1);
  attachInterrupt(2, doEncoderA1, RISING);//电机1的编码器的A相shang sheng yan du qu
  attachInterrupt(0, doEncoderA2, RISING);//电机2的编码器的A相


}

void loop() {

     preset_and_send();
}

void preset_and_send()
{
  if (Serial.available()==7)
  {
    for(int i=0;i<7;i++)
    {
        Buffer[i]=Serial.read();
    }
    i=0;
         //delay(1);
      if(Buffer[0]==0x00)
      {
           b1=Buffer[1];
           b2=Buffer[2];
           b3=Buffer[3];
           b4=Buffer[4];
           b5=Buffer[5];
           b6=Buffer[6];
           Serial.write(b1);
           Serial.write(b2);
           Serial.write(b3);
           Serial.write(b4);
          Serial.write(b5);
           Serial.write(b6);
           counter1=b1*65536+b2*256+b3;
           counter2=b4*65536+b5*256+b6;//无法实现counter为负值
           Buffer[0]='\0';
           Buffer[1]='\0';
           Buffer[2]='\0';
           Buffer[3]='\0';
           Buffer[4]='\0';
           Buffer[5]='\0';
           Buffer[6]='\0';
           
        }
        if(Buffer[0]==0xff)
        {
        Serial.write(b1);
        Serial.write(b2);
        Serial.write(b3);
        Serial.write(b4);
        Serial.write(b5);
        Serial.write(b6);
   
        }
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


