
#include <Wire.h>

int address = 114;
int errcnt;//不知出现的原因
unsigned long countersprev;
unsigned long counters;//定义一个无符号32位整形数组，存储cpld返回的计数

void setup()
{
    
    Wire.begin();//主机arduino加入I2C通信中,作为主机
    Serial.begin(115200);
    Serial.println("Verilog I2C Test\n\n");
  
    reset();
    delay(100);

}

void reset()
{
    
    countersprev = 0;
    counters = 0;
    errcnt = 0;
    countersprev = 0;
    counters = 0;
    Wire.beginTransmission(address);
    Wire.write(1);
    Wire.endTransmission();  
   
}

bool readCounters()
{
    byte valuesread[3];//定义字节数组,读三个字节就好
    unsigned long countersread;//定义数组存放计数器的计数值,其中countread[0]放高16位，counterread[1]放低8位
    byte readcnt = 0;
    bool rslt = false;
  
    Wire.beginTransmission(address);
    Wire.write(0);
    int error = Wire.endTransmission();  
    delay(1000); 

  
    //Read values    
    if (error == 0)//成功发送
    {
        byte count = Wire.requestFrom(address, 3);//从从机中读取3个字节
        
        if (count == 3)
        {
            for (int i = 0; i < 3; i++)
                valuesread[i] = Wire.read();//将每个字节给数组，[0]指最高8位，【1】中间8位，【2】低8位
          
            rslt = true;
        }
        else rslt = false;
    }
    else rslt = false;
  
    if (rslt)
    {
        //Assemble values
        for (int i = 0; i < 3; i++)
        {
          countersread= countersread<<8;
		  countersread= countersread+valuesread[i];
      }
  
      //Validate values
//      for (int i = 0; i < 4; i++)
//      {
//          long iprediff = counters[i] - countersprev[i];
//          word prediff = 0;
//          
//          if (iprediff < 0)
//            prediff = 65535 + iprediff;
//          else prediff = iprediff;
//          
//          long idiff = countersread[i] - counters[i];
//          word diff = 0;
//          
//          if (idiff < 0)
//            diff = 65535 + idiff;
//          else diff = idiff;
//          
//          double diffweight = abs(diff / prediff);
//        
//          if (diffweight > 2)
//            rslt = false; 
//      }
    
      //Copy values across
      //if (rslt)
      //{
          
         
         countersprev = counters;
         counters = countersread;  
      
        
      //}
    }
  
    if (!rslt)
       errcnt++;
  
    return rslt;
}

void loop()
{
    unsigned long time = millis();
    readCounters();
  
    Serial.print("Counts:\n");
    Serial.println(counters);
   
   
    unsigned long time2 = millis();
  
    delay(50 - (time2 - time));
}

