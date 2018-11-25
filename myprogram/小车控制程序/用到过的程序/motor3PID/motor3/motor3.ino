
//与电机相关设置
int AIN5=38;
int AIN6=40;
int AM3=42;
int BM3=44;
int PWM3=8;
int flagA3=0;
int flagB3=0;//电机3的标志位设定
int valA3=0;
int valB3=0;//用来储存电机3的AB相记录的脉冲数
int valpid;//设置pid输出值
double n3;//存储电机3的转速的变量
unsigned long times;
unsigned long newtime;

//与ＰＩＤ相关设置
unsigned long lasttime;
double Input,Output,Setpoint=3;//设置期望速度为3rad/s
double errorsum,derror;
double lasterror;
double kp=50,kd=0,ki=40;
int SampleTime=100;//设置pid采样时间100ms
int d_time=500;//设定脉冲采样的单位时间






void setup() {
  Serial.begin(9600);
  pinMode(AIN5,OUTPUT);
  pinMode(AIN6,OUTPUT);
  pinMode(AM3,INPUT);
  pinMode(BM3,INPUT);
  pinMode(PWM3,OUTPUT);
 

}

void loop() {

 
  collect();
  pidcompute();
  go();
  

}
void collect()//采集编码器返回的脉冲,并转换为速度值
{
   newtime=times=millis();
  while((newtime-times)<d_time)
  {
     if(digitalRead(AM3)==HIGH && flagA3==0)
     {
       valA3++;
       flagA3=1;
     }
      if(digitalRead(AM3)==LOW && flagA3==1)
       {
         valA3++;
         flagA3=0;
       }
       if(digitalRead(BM3)==HIGH && flagB3==0)
       {
         valB3++;
         flagB3=1; 
       }
       if(digitalRead(BM3)==LOW && flagB3==1)
       {
        valB3++;
        flagB3=0; 
       }//电机3的编码器脉冲计数
       newtime=millis();   
   }
    n3=(valA3+valB3)/(1.56*d_time);//计算电机3转速
    Serial.print("MOTOR3:");
    Serial.print(n3);
    Serial.println("rad/s");//输出电机3转速数值
    valA3=valB3=0;//清零储存脉冲数的变量
    Input=n3;  
 }

void pidcompute()//进行PID控制
{
  unsigned long nowtime=millis();
  int timeChange=(nowtime-lasttime);
  if (timeChange>=SampleTime)
  {
    double error=Setpoint-Input;
    errorsum+= error*0.1;
    derror=(error-lasterror)*10;
    Output=kp*error+ki*errorsum+kd*derror;
    Serial.print("pwm");
    Serial.print(Output);
    lasterror=error;
    lasttime=nowtime;
   }
  
   
 }
   
void go()
{ 
  
    digitalWrite(AIN5,HIGH);
    digitalWrite(AIN6,LOW);
    analogWrite(PWM3,abs(Output));
}
