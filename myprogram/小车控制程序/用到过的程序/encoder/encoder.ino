//采用定时器中断和一个外部中断实现的旋转编码器的计数,一个电机为例,没有四倍频
#include <MsTimer2.h>
#define encoder0PinA 2
#define encoder0PinB 3
volatile unsigned int encoder0Pos = 0;
float times;
float newtime;

//定时中断函数
void flash() 
{
    
  float counter = encoder0Pos;
  float n = counter ;
  Serial.print(n);
  Serial.println("r/s");
  counter = 0;
  encoder0Pos = 0;

  
}

void setup() {
  pinMode(encoder0PinA, INPUT); 
  pinMode(encoder0PinB, INPUT); 

// encoder pin on interrupt 0 (pin 2)
  attachInterrupt(0, doEncoderA, FALLING);

  //timer interrupt
  MsTimer2::set(1000, flash);  // 中断设置函数，每 1s 进入一次定时中断,类似采样时间
  MsTimer2::start();

  Serial.begin (115200);//波特率设置为115200才可以正确显示数据,
  /*1/115200=8.7us;1/9600=104us*/
  
  
}

void loop(){    }

void doEncoderA(){
  newtime=times=micros();
 
  // look for a low-to-high on channel A
  if (digitalRead(encoder0PinA) == LOW) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == HIGH) {  
      encoder0Pos = encoder0Pos + 1;         // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }
 // Serial.println (encoder0Pos, DEC);          

    newtime=micros();
   //Serial.println (newtime - times, DEC); //88us左右

}

