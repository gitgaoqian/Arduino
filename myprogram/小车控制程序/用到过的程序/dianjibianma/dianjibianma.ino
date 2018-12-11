
int d_time=500;//设定计算编码器脉冲的时间
//定义和设置pid控制中所涉及的参数
unsigned long lasttime;
unsigned long nowtime;
double pi=3.1416;
double Setpoint1,Setpoint2,Setpoint3;//设置期望速度
double Input1,Input2,Input3;
double Output1,Output2,Output3;
double errorsum1,errorsum2,errorsum3;
double derror1,derror2,derror3;
double lasterror1,lasterror2,lasterror3;
double error1,error2,error3;
double kp1=50,kd1=0,ki1=30;
double kp2=50,kd2=0,ki2=30;
double kp3=50,kd3=0,ki3=30;
int SampleTime=100;//设置pid采样时间100ms
int flagA1=0;
int flagB1=0;//电机1的标志位设定*/
int flagA2=0;
int flagB2=0;//电机2的标志位设定
int flagA3=0;
int flagB3=0;//电机3的标志位设定
int AM1=6;
int BM1=7;//电机1的A相B相输入引脚的定义*/
int AM2=12;
int BM2=13;//电机2的AB相的引脚定义
int AM3=42;
int BM3=44;//电机3的AB相的引脚定义
int AIN1=4;
int AIN2=5;
int PWM1=3;//电机1的AIN1、2和PWM1是电机输出引脚的定义*/
int AIN3=11;
int AIN4=10;
int PWM2=9;//电机2的AIN3 4和PWM2是电机输出引脚的定义
int AIN5=38;
int AIN6=40;
int PWM3=8;//电机2的AIN3 4和PWM2是电机输出引脚的定义
int valA1=0;
int valB1=0;//用来储存电机1的A相B相记录的脉冲数*/
int valA2=0;
int valB2=0;//用来储存电机2的AB相记录的脉冲数
int valA3=0;
int valB3=0;//用来储存电机3的AB相记录的脉冲数
double n1;//存储电机1的转速的变量*/
double n2;//存储电机2的转速的变量
double n3;//存储电机2的转速的变量
unsigned long times;
unsigned long newtime;//时间变量

void go(int g1,int g2,int g3);
void back(int b);
void stay();//电机子函数申明

double time1,time2,time3;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(AIN1,OUTPUT);
  pinMode(AIN2,OUTPUT);
  pinMode(PWM1,OUTPUT);
  pinMode(AM1,INPUT);
  pinMode(BM1,INPUT);
  pinMode(AIN3,OUTPUT);
  pinMode(AIN4,OUTPUT);
  pinMode(PWM2,OUTPUT);
  pinMode(AM2,INPUT);
  pinMode(BM2,INPUT);
  pinMode(AIN5,OUTPUT);
  pinMode(AIN6,OUTPUT);
  pinMode(PWM3,OUTPUT);
  pinMode(AM3,INPUT);
  pinMode(BM3,INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  go(0,0,125);//驱动电机1 3,小车直线行走,
  //delay(2000);

// collect();

 delay(400);

 
}
void collect()
{
  newtime=times=millis();
 while((newtime-times)<d_time)
  {
    if(digitalRead(AM1)==HIGH && flagA1==0)
    {
      valA1++;
      flagA1=1;
     
     }
     if(digitalRead(AM1)==LOW && flagA1==1)
     {
        valA1++;
        flagA1=0;
      }
      if(digitalRead(BM1)==HIGH && flagB1==0)
      {
        valB1++;
        flagB1=1; 
       }
       if(digitalRead(BM1)==LOW && flagB1==1)
       {
         valB1++;
         flagB1=0; 
       }//电机1对脉冲数的记录,采用了四分频技术*/
       if(digitalRead(AM2)==HIGH && flagA2==0)
       {
         valA2++;
         flagA2=1;
       }
       if(digitalRead(AM2)==LOW && flagA2==1)
       {
         valA2++;
         flagA2=0;
       }
       if(digitalRead(BM2)==HIGH && flagB2==0)
       {
         valB2++;
         flagB2=1; 
       }
       if(digitalRead(BM2)==LOW && flagB2==1)
       {
        valB2++;
        flagB2=0; 
       }//电机2的编码器脉冲计数
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
    n1=(valA1+valB1)*2*pi/(1.56*d_time);//计算电机1转速*/
    n2=(valA2+valB2)*2*pi/(1.56*d_time);//计算电机2转速
    n3=(valA3+valB3)*2*pi/(1.56*d_time);//计算电机3转速
    Serial.print("MOTOR1:");
    Serial.print(n1);
    Serial.println("rad/s");//输出电机1转速数值*/
    Serial.print("MOTOR2:");
    Serial.print(n2);
    Serial.println("rad/s");//输出电机2转速数值
    Serial.print("MOTOR3:");
    Serial.print(n3);
    Serial.println("rad/s");//输出电机3转速数值*/
    valA1=valB1=0;
    valA2=valB2=0;//清零储存脉冲数的变量
    valA3=valB3=0;//清零储存脉冲数的变量
    pidcompute();
}
void go(int g1,int g2,int g3)
{ 
  if(g1>0)//设置电机1正转
  {
    digitalWrite(AIN1,HIGH);
     digitalWrite(AIN2,LOW);
   }
   if(g1<0)//设置电机1反转
   {
     digitalWrite(AIN1,LOW);
      digitalWrite(AIN2,HIGH);
   }
   if(g2>0)//设置电机2正转
  {
    digitalWrite(AIN3,HIGH);
     digitalWrite(AIN4,LOW);
   }
   if(g2<0)//设置电机2反转
   {
     digitalWrite(AIN3,LOW);
      digitalWrite(AIN4,HIGH);
   }
   if(g3>0)//设置电机3正转
  {
    digitalWrite(AIN5,HIGH);
     digitalWrite(AIN6,LOW);
   }
   if(g3<0)//设置电机1反转
   {
     digitalWrite(AIN5,LOW);
      digitalWrite(AIN6,HIGH);
   }
   analogWrite(PWM1,abs(g1));
   analogWrite(PWM2,abs(g2));
   analogWrite(PWM3,abs(g3));
}

void stay(int s)
{
  if (s==1)//停止电机1
  {
    digitalWrite(AIN1,HIGH);
    digitalWrite(AIN2,HIGH);  
   }
   if(s==2)//停止电机2
   {
    digitalWrite(AIN3,HIGH);
    digitalWrite(AIN4,HIGH);
   }
   if (s==3)//停止电机3
   {
    digitalWrite(AIN5,HIGH);
    digitalWrite(AIN6,HIGH); 
   }
  
}
void pidcompute()
{
     nowtime=millis();
  int timeChange=(nowtime-lasttime);
  if (timeChange>=SampleTime)//开始进行pid调节,
  {
    //pid调节电机1
    Setpoint1=3;
    Input1=n1;
    error1=Setpoint1-Input1;
    errorsum1+= error1;
    derror1=(error1-lasterror1);
    Output1=kp1*error1+ki1*errorsum1+kd1*derror1;
    lasterror1=error1;
    //pid调节电机2
    Setpoint2=3;
    Input2=n2;
    error2=Setpoint2-Input2;
    errorsum2+= error2;
    derror2=(error2-lasterror2);
    Output2=kp2*error2+ki2*errorsum2+kd2*derror2;
    lasterror2=error2;
    //pid调节电机3
    Setpoint3=3;
    Input3=n3;
     error3=Setpoint3-Input3;
    errorsum3+= error3;
    derror3=(error3-lasterror3);
    Output3=kp3*error3+ki3*errorsum3+kd3*derror3;
    lasterror3=error3;
    lasttime=nowtime;
   } 
}
