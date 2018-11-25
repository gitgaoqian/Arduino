/******************************************************************
  created on 2018-5-11
******************************************************************/
#include <ros.h>
#include <neu_wgg/location.h>
ros::NodeHandle nh;
neu_wgg::location gps_msg;
ros::Publisher gps_pub("gps_topic",&gps_msg);
struct
{
  char GPS_Buffer[80];
  bool isGetData;   //是否获取到GPS数据
  bool isParseData; //是否解析完成
  float lat;
  float lng;
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


void setup()  //初始化内容
{
  Serial1.begin(9600);      //定义波特率9600，和我们店铺的GPS模块输出的波特率一致
  Save_Data.isGetData = false;
  Save_Data.isParseData = false;
  Save_Data.isUsefull = false;
  nh.initNode();
  nh.advertise(gps_pub);
  delay(300);
}

void loop()   //主循环
{
  gpsRead();  //获取GPS数据
  //Serial.println(Save_Data.Gps_Buffer)
  parseGpsBuffer();//解析GPS数据
  gps_pub.publish(&gps_msg);
  nh.spinOnce();
  delay(1000);
  //printGpsBuffer();//输出解析后的数据
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
    gps_msg.longitude = float(*Save_Data.longitude)/100;
    gps_msg.latitude = float(*Save_Data.latitude)/100;
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

