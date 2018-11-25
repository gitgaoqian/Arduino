//采用定时器中断和外部中断实现的旋转编码器的计数,一个电机为例
/*程序说明:使用micro函数测定A相每记录两个跳边沿的时间间隔为不超过200us;由于是四倍频的关系,这中间
  夹着一个B相跳边沿.所以每记录一个条边沿的时间为不超过100us.电机空载转速是8.3r/s,此时理论上要记录8.3*1560个
脉冲,每记录一个的时间为1/8.3/1560=77us,理论上会有记不到的脉冲*/
//#include <MsTimer2.h>
#define encoder0PinA 19
#define encoder0PinB 18
double encoder0Pos = 0;//设置有符号整形变量
float times;
float newtime;
double counter;

//定时中断函数
/*void flash() 
{
    
  float counter = encoder0Pos;
  float n = counter ;
  Serial.print(n);//'+':正转;'-':反转;
  Serial.println("r/s");
  counter = 0;
  encoder0Pos = 0;

  
}*/

void setup() {
  pinMode(encoder0PinA, INPUT); 
  pinMode(encoder0PinB, INPUT); 

// encoder pin on interrupt 0 (pin 2)
  attachInterrupt(4, doEncoderA, CHANGE);

// encoder pin on interrupt 1 (pin 3)
  attachInterrupt(5, doEncoderB, CHANGE);  

  //timer interrupt
  //MsTimer2::set(100, flash);        // 中断设置函数，每 1s 进入一次定时中断
  //MsTimer2::start();

  Serial.begin (115200);
  
  
}

void loop(){
  counter = encoder0Pos;
  Serial.println(counter);
  delay(100);
  }

void doEncoderA(){
  newtime=times=micros();
 
  // look for a low-to-high on channel A
  if (digitalRead(encoder0PinA) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == LOW) {  
      encoder0Pos = encoder0Pos + 1;         // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder0PinB) == HIGH) {   
      encoder0Pos = encoder0Pos + 1;          // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
  //Serial.println (encoder0Pos, DEC);          
  // use for debugging - remember to comment out
    newtime=micros();
   //Serial.println (newtime - times, DEC); 
  

}

void doEncoderB(){

  // look for a low-to-high on channel B
  if (digitalRead(encoder0PinB) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(encoder0PinA) == HIGH) {  
      encoder0Pos = encoder0Pos + 1;         // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder0PinA) == LOW) {   
      encoder0Pos = encoder0Pos + 1;          // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }

   
}
