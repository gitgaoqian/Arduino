
/*#include <ros.h>
//#include <geometry_msgs/Twist.h>//订阅twist消息类型话题，并转换为期望轮速
//#include <smartcar/carspeed.h>//转换编码器采样速度到车身速度,并发布出去
//*/
// 定义和设置pid控制中所涉及的参数
unsigned long lasttime;
unsigned long nowtime;
double pi = 3.14159269297;
double Setpoint1 = 0, Setpoint2 = 0, Setpoint3 = 0.0; // 设置期望速度
double Input1, Input2, Input3;
double Output1, Output2, Output3;
double errorsum1, errorsum2, errorsum3;

double derror1, derror2, derror3;
double lasterror1, lasterror2, lasterror3;
double error1, error2, error3;
double kp1 = 13, kd1 = 2, ki1 = 33;
double kp2 = 8, kd2 = 0, ki2 = 26;
double kp3 = 10, kd3 = 0, ki3 = 28;
int SampleTime = 100; // 设置pid采样时间100ms
// 设置AB相中断输入引脚
#define A2 19
#define B2 18 // 电机2的A相B相输入引脚的定义:外部中断0 1*/
#define A1 21
#define B1 20 // 电机1的A相B相输入引脚的定义:外部中断2 3*/
#define A3 2
#define B3 3 // 电机3的A相B相输入引脚的定义:外部中断4 5*/ //把A2 B2接到19 18;A3 B3接到2　3编码器读数就不行，为什么？
// 设置L298N引脚
int AIN1 = 24;
int AIN2 = 22;
int PWM1 = 4; // 电机1的AIN1、2和PWM1是电机输出引脚的定义*/
int AIN3 = 10;
int AIN4 = 9;
int PWM2 = 8; // 电机2的AIN3 4和PWM2是电机输出引脚的定义
int AIN5 = 7;
int AIN6 = 6;
int PWM3 = 5; // 电机3的AIN3 4和PWM2是电机输出引脚的定义
// 设置一些变量
double counter1; // 用来储存电机1的编码器记录的脉冲数*/
double counter2; // 用来储存电机2的编码器记录的脉冲数
double counter3; // 用来储存电机3的编码器记录的脉冲数
double n1;       // 存储电机1的转速的变量*/
double n2;       // 存储电机2的转速的变量
double n3;       // 存储电机2的转速的变量
unsigned long times;
unsigned long newtime; // 时间变量

// 驱动电机函数声明
void go(int g1, int g2, int g3);
void back(int b);
void stay(); // 电机子函数申明

// double time1,time2,time3;

// 定义车身前进的线速度和角速度
/*double lin_vel_x, lin_vel_y;
double ang_vel;
//定义车轮的转速,同样也是pid控制器中setpoint的输入
double w1,w2,w3;
//定义全向轮小车参数
double R=0.03;
double l=0.115;//车轮到底盘中心的距离m
double fi=pi/6;

ros::NodeHandle nh;
void motor_cb(const geometry_msgs::Twist& vel)
{
   lin_vel_x = vel.linear.x;
   lin_vel_y=vel.linear.y;
   ang_vel = vel.angular.z;
   //参考运动学模型解算出各轮转速
   w1=(lin_vel_x*(-cos(fi))+lin_vel_y*sin(fi)+ang_vel*l)/(2*pi*R);
   w2=(lin_vel_y*(-sin(fi))+ang_vel*l)/(2*pi*R);
   w3=(lin_vel_x*cos(fi)+lin_vel_y*sin(fi)+ang_vel*l)/(2*pi*R);
   go(w1,w2,w3);

}
//定义twist消息订阅者
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", motor_cb);
//定义小车速度的发布者
smartcar::carspeed carspeed_msg;
ros::Publisher carspeed_pub("carspeed",&carspeed_msg);
*/
void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200); // 加上ros程序后,arduino中serial没用了,串口频率的
  // 设定用rosrun serial_python命令,默认是57600
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(A1, INPUT);
  pinMode(B1, INPUT);
  attachInterrupt(2, doEncoderA1, CHANGE);
  attachInterrupt(3, doEncoderB1, CHANGE); // 电机1的编码器的AB相
  pinMode(AIN3, OUTPUT);
  pinMode(AIN4, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(A2, INPUT);
  pinMode(B2, INPUT);
  attachInterrupt(0, doEncoderA3, CHANGE);
  attachInterrupt(1, doEncoderB3, CHANGE); // 电机2的编码器的AB相
  pinMode(AIN5, OUTPUT);
  pinMode(AIN6, OUTPUT);
  pinMode(PWM3, OUTPUT);
  pinMode(A3, INPUT);
  pinMode(B3, INPUT);
  attachInterrupt(4, doEncoderA2, CHANGE);
  attachInterrupt(5, doEncoderB2, CHANGE); // 第三个编码器的AB相
  /*nh.initNode();
  nh.subscribe(sub);// put your setup code here, to run once:
  nh.advertise(carspeed_pub);
  */
}

void loop()
{
  // go(0,0,0);//驱动电机1 3,小车直线行走,50 37 69
  pidcompute();
  Serial.print("MOTOR1:");
  Serial.print(n1);
  Serial.println("r/s");
  Serial.print("MOTOR2:");
  Serial.print(n2);
  Serial.println("r/s");
  Serial.print("MOTOR3:");
  Serial.print(n3);
  Serial.println("r/s");
  counter1 = 0;
  counter2 = 0;
  counter3 = 0;
  go(Output1, Output2, Output3);
  // carspeed_pub.publish(&carspeed_msg);
  // nh.spinOnce();
  delay(100);
}

// 第一个编码器四倍频计数
void doEncoderA1()
{
  newtime = times = micros();

  // look for a low-to-high on channel A
  if (digitalRead(A1) == HIGH)
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(B1) == LOW)
    {
      counter1 = counter1 + 1; // CW
    }
    else
    {
      counter1 = counter1 - 1; // CCW
    }
  }
  else // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(B1) == HIGH)
    {
      counter1 = counter1 + 1; // CW
    }
    else
    {
      counter1 = counter1 - 1; // CCW
    }
  }
  // Serial.println (counter1, DEC);
  //  use for debugging - remember to comment out
  newtime = micros();
  // Serial.println (newtime - times, DEC);
}

void doEncoderB1()
{

  // look for a low-to-high on channel B
  if (digitalRead(B1) == HIGH)
  {
    // check channel A to see which way encoder is turning
    if (digitalRead(A1) == HIGH)
    {
      counter1 = counter1 + 1; // CW
    }
    else
    {
      counter1 = counter1 - 1; // CCW
    }
  }
  // Look for a high-to-low on channel B
  else
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(A1) == LOW)
    {
      counter1 = counter1 + 1; // CW
    }
    else
    {
      counter1 = counter1 - 1; // CCW
    }
  }
}
// 第二个编码器四倍频计数
void doEncoderA2()
{
  newtime = times = micros();

  // look for a low-to-high on channel A
  if (digitalRead(A2) == HIGH)
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(B2) == LOW)
    {
      counter2 = counter2 + 1; // CW
    }
    else
    {
      counter2 = counter2 - 1; // CCW
    }
  }
  else // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(B2) == HIGH)
    {
      counter2 = counter2 + 1; // CW
    }
    else
    {
      counter2 = counter2 - 1; // CCW
    }
  }
  // Serial.println (counter2, DEC);
  //  use for debugging - remember to comment out
  newtime = micros();
  // Serial.println (newtime - times, DEC);
}
void doEncoderB2()
{

  // look for a low-to-high on channel B
  if (digitalRead(B2) == HIGH)
  {
    // check channel A to see which way encoder is turning
    if (digitalRead(A2) == HIGH)
    {
      counter2 = counter2 + 1; // CW
    }
    else
    {
      counter2 = counter2 - 1; // CCW
    }
  }
  // Look for a high-to-low on channel B
  else
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(A2) == LOW)
    {
      counter2 = counter2 + 1; // CW
    }
    else
    {
      counter2 = counter2 - 1; // CCW
    }
  }
}
// 第三个编码器四倍频计数
void doEncoderA3()
{
  newtime = times = micros();

  // look for a low-to-high on channel A
  if (digitalRead(A3) == HIGH)
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(B3) == LOW)
    {
      counter3 = counter3 + 1; // CW
    }
    else
    {
      counter3 = counter3 - 1; // CCW
    }
  }
  else // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(B3) == HIGH)
    {
      counter3 = counter3 + 1; // CW
    }
    else
    {
      counter3 = counter3 - 1; // CCW
    }
  }
  // Serial.println (counter3, DEC);
  //  use for debugging - remember to comment out
  newtime = micros();
  // Serial.println (newtime - times, DEC);
}

void doEncoderB3()
{

  // look for a low-to-high on channel B
  if (digitalRead(B3) == HIGH)
  {
    // check channel A to see which way encoder is turning
    if (digitalRead(A3) == HIGH)
    {
      counter3 = counter3 + 1; // CW
    }
    else
    {
      counter3 = counter3 - 1; // CCW
    }
  }
  // Look for a high-to-low on channel B
  else
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(A3) == LOW)
    {
      counter3 = counter3 + 1; // CW
    }
    else
    {
      counter3 = counter3 - 1; // CCW
    }
  }
}

void go(int g1, int g2, int g3)
{
  if (g1 > 0) // 设置电机1正转
  {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  }
  if (g1 < 0) // 设置电机1反转
  {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  }
  if (g2 > 0) // 设置电机2正转
  {
    digitalWrite(AIN3, HIGH);
    digitalWrite(AIN4, LOW);
  }
  if (g2 < 0) // 设置电机2反转
  {
    digitalWrite(AIN3, LOW);
    digitalWrite(AIN4, HIGH);
  }
  if (g3 > 0) // 设置电机3正转
  {
    digitalWrite(AIN5, HIGH);
    digitalWrite(AIN6, LOW);
  }
  if (g3 < 0) // 设置电机1反转
  {
    digitalWrite(AIN5, LOW);
    digitalWrite(AIN6, HIGH);
  }
  analogWrite(PWM1, abs(g1));
  analogWrite(PWM2, abs(g2));
  analogWrite(PWM3, abs(g3));
}

void stay(int s)
{
  if (s == 1) // 停止电机1
  {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, HIGH);
  }
  if (s == 2) // 停止电机2
  {
    digitalWrite(AIN3, HIGH);
    digitalWrite(AIN4, HIGH);
  }
  if (s == 3) // 停止电机3
  {
    digitalWrite(AIN5, HIGH);
    digitalWrite(AIN6, HIGH);
  }
}
void pidcompute()
{
  nowtime = millis();
  int timeChange = (nowtime - lasttime);
  if (timeChange >= SampleTime) // 开始进行pid调节,
  {
    // pid调节电机1
    n1 = counter1 / 1560 / SampleTime * 1000;
    n2 = counter2 / 1560 / SampleTime * 1000;
    n3 = counter3 / 1560 / SampleTime * 1000;
    Input1 = n1;
    error1 = Setpoint1 - Input1;
    errorsum1 += error1 * SampleTime / 1000;
    derror1 = (error1 - lasterror1) * 1000 / SampleTime;
    Output1 = kp1 * error1 + ki1 * errorsum1 + kd1 * derror1;
    lasterror1 = error1;
    if (abs(Output1) > 255)
      Output1 = 255;

    // pid调节电机2

    Input2 = n2;
    error2 = Setpoint2 - Input2;
    errorsum2 += error2 * SampleTime / 1000;
    derror2 = (error2 - lasterror2) * 1000 / SampleTime;
    Output2 = kp2 * error2 + ki2 * errorsum2 + kd2 * derror2;
    lasterror2 = error2;
    if (abs(Output2) > 255)
      Output2 = 255;
    // pid调节电机3

    Input3 = n3;
    error3 = Setpoint3 - Input3;
    errorsum3 += error3 * SampleTime / 1000;
    derror3 = (error3 - lasterror3) * 1000 / SampleTime;
    Output3 = kp3 * error3 + ki3 * errorsum3 + kd3 * derror3;
    lasterror3 = error3;
    if (abs(Output3) > 255)
      Output3 = 255;
    lasttime = nowtime;
  }
}
