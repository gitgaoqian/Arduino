/******************************************************************
  created on 2018-5-13
******************************************************************/
#include <dht.h>
#include <ros.h>
#include <neu_wgg/sensor.h>
//定义温度传感器变量
#define DH11_PIN 2
dht DHT;

ros::NodeHandle nh;
neu_wgg::sensor sensor_msg;
ros::Publisher sensor_pub("sensor_topic",&sensor_msg);

//GPS变量定义
struct
{
  char GPS_Buffer[80];
  bool isGetData;   //是否获取到GPS数据
  bool isParseData; //是否解析完成
  char UTCTime[11];   //UTC时间
  char latitude[11];    //纬度
  char N_S[2];    //N/S
  char longitude[12];   //经度
  char E_W[2];    //E/W
  bool isUsefull;   //定位信息是否有效
} Save_Data;

const unsigned int gpsRxBufferLength = 600;
char gpsRxBuffer[gpsRxBufferLength];
unsigned int ii = 0;

//惯性变量定义
unsigned char Re_buf[11],counter=0;
unsigned char sign=0;
float a[3],w[3],angle[3],T;



void setup()  //初始化内容
{
  Serial1.begin(9600);      //GPS波特率9600，和我们店铺的GPS模块输出的波特率一致
  Serial2.begin(9600);    //惯性传感器波特率定义,注:其波特率只能通过上位机修改,此处默认为115200
  Save_Data.isGetData = false;
  Save_Data.isParseData = false;
  Save_Data.isUsefull = false;
  nh.initNode();
  nh.advertise(sensor_pub);
  delay(300);
}

void loop()   //主循环
{
  TemRead();//获取温度数据
  gpsRead();  //获取GPS数据
  parseGpsBuffer();//解析GPS数据
  InertiaRead();//获取惯性数据
  sensor_pub.publish(&sensor_msg);
  nh.spinOnce();
}

void TemRead()
{
  DHT.read11(DH11_PIN); 
  sensor_msg.atmo = 1.5;
  sensor_msg.temp = DHT.temperature;
  sensor_msg.hum = DHT.humidity;
  }
  
void InertiaRead()
{
   while (Serial2.available()) {
    
    //char inChar = (char)Serial.read(); Serial.print(inChar); //Output Original Data, use this code 
    Re_buf[counter]=(unsigned char)Serial2.read();
    if(counter==0&&Re_buf[0]!=0x55) return;      //第0号数据不是帧头              
    counter++;       
    if(counter==11)             //接收到11个数据
    {    
       counter=0;               //重新赋值，准备下一帧数据的接收 
       sign=1;
    }
      
  }
  if(sign)
  {  
     sign=0;
     if(Re_buf[0]==0x55)      //检查帧头
     {  
  switch(Re_buf [1])
  {
  case 0x51:
    a[0] = (short(Re_buf [3]<<8| Re_buf [2]))/32768.0*16;
    a[1] = (short(Re_buf [5]<<8| Re_buf [4]))/32768.0*16;
    a[2] = (short(Re_buf [7]<<8| Re_buf [6]))/32768.0*16;
    T = (short(Re_buf [9]<<8| Re_buf [8]))/340.0+36.25;
    break;
  case 0x52:
    w[0] = (short(Re_buf [3]<<8| Re_buf [2]))/32768.0*2000;
    w[1] = (short(Re_buf [5]<<8| Re_buf [4]))/32768.0*2000;
    w[2] = (short(Re_buf [7]<<8| Re_buf [6]))/32768.0*2000;
    T = (short(Re_buf [9]<<8| Re_buf [8]))/340.0+36.25;
    break;
  case 0x53:
          angle[0] = (short(Re_buf [3]<<8| Re_buf [2]))/32768.0*180;
    angle[1] = (short(Re_buf [5]<<8| Re_buf [4]))/32768.0*180;
    angle[2] = (short(Re_buf [7]<<8| Re_buf [6]))/32768.0*180;
    T = (short(Re_buf [9]<<8| Re_buf [8]))/340.0+36.25;
                break;
  } 
    }
  } 
  sensor_msg.angle_x = angle[0];
  sensor_msg.angle_y = angle[1];
  sensor_msg.angle_z = angle[2];
  
}


void errorLog(int num)
{
  //Serial.print("ERROR");
  //Serial.println(num);
  delay(num);
}

void printGpsBuffer()
{
  if (Save_Data.isParseData)
  {
    Save_Data.isParseData = false;
//    
//    Serial.print("Save_Data.UTCTime = ");
//   Serial.println(Save_Data.UTCTime);

    if(Save_Data.isUsefull)
    {
      Save_Data.isUsefull = false;
//      Serial.print("Save_Data.latitude = ");
//      Serial.println(Save_Data.latitude);
//      Serial.print("Save_Data.N_S = ");
//      Serial.println(Save_Data.N_S);
//      Serial.print("Save_Data.longitude = ");
//      Serial.println(Save_Data.longitude);
//      Serial.print("Save_Data.E_W = ");
//      Serial.println(Save_Data.E_W);
    }
    else
    {
//      Serial.println("GPS DATA is not usefull!");
        delay(1);
    }
    
  }
}

void parseGpsBuffer()
{
  char *subString;
  char *subStringNext;
  if (Save_Data.isGetData)
  {
    Save_Data.isGetData = false;
//    Serial.println("**************");
//    Serial.println(Save_Data.GPS_Buffer);

    
    for (int i = 0 ; i <= 6 ; i++)//读取前7个字节
    {
      if (i == 0)
      {
        if ((subString = strstr(Save_Data.GPS_Buffer, ",")) == NULL)
          errorLog(1);  //解析错误
      }
      else
      {
        subString++;
        if ((subStringNext = strstr(subString, ",")) != NULL)
        {
          char usefullBuffer[2]; 
          switch(i)
          {
            case 1:memcpy(Save_Data.UTCTime, subString, subStringNext - subString);break; //获取UTC时间
            case 2:memcpy(usefullBuffer, subString, subStringNext - subString);break; //获取定位是否有效
            case 3:memcpy(Save_Data.latitude, subString, subStringNext - subString);break;  //获取纬度信息
            case 4:memcpy(Save_Data.N_S, subString, subStringNext - subString);break; //获取N/S
            case 5:memcpy(Save_Data.longitude, subString, subStringNext - subString);break; //获取纬度信息
            case 6:memcpy(Save_Data.E_W, subString, subStringNext - subString);break; //获取E/W

            default:break;
          }

          subString = subStringNext;
          Save_Data.isParseData = true;
          if(usefullBuffer[0] == 'A')
            Save_Data.isUsefull = true;
          else if(usefullBuffer[0] == 'V')
            Save_Data.isUsefull = false;

        }
        else
        {
          errorLog(2);  //解析错误
        }
      }


    }
    sensor_msg.longitude = atof(Save_Data.longitude)/100.00;
    sensor_msg.latitude = atof(Save_Data.latitude)/100.00;
  }
}

void gpsRead() {
  while (Serial1.available())
  {
    gpsRxBuffer[ii++] = Serial1.read();
    if (ii == gpsRxBufferLength)clrGpsRxBuffer();
  }

  char* GPS_BufferHead;
  char* GPS_BufferTail;
  if ((GPS_BufferHead = strstr(gpsRxBuffer, "$GPRMC,")) != NULL || (GPS_BufferHead = strstr(gpsRxBuffer, "$GNRMC,")) != NULL )
  {
    if (((GPS_BufferTail = strstr(GPS_BufferHead, "\r\n")) != NULL) && (GPS_BufferTail > GPS_BufferHead))
    {
      memcpy(Save_Data.GPS_Buffer, GPS_BufferHead, GPS_BufferTail - GPS_BufferHead);//把以 GPS_BufferHead为首地址开始后的连续GPS_BufferTail - GPS_BufferHead个字节存放到以Save_Data.GPS_Buffer起始地址的内存空间
      Save_Data.isGetData = true;

      clrGpsRxBuffer();
    }
  }
}

void clrGpsRxBuffer(void)
{
  memset(gpsRxBuffer, 0, gpsRxBufferLength);      //清空
  ii = 0;
}

