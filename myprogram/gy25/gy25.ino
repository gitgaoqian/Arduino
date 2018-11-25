//GY-25  ARDUINO  测试代码 IICLCD2004显示角度
//注意1：下载程序时候，请先断开GY25的连线!否则将导致下载不成功
//注意2：GY25模块使用时，上电自校正,建议不要用手拿着模块,
int YPR[3];
unsigned char Re_buf[8],counter=0;
unsigned char sign=0;

//-----------------------------------------------------------
void setup()
{
   
  Serial.begin(115200);  
  delay(2000);    
  Serial.write(0XA5); 
  Serial.write(0X52);    //初始化GY25,连续输出模式

}
//-------------------------------------------------------------
void loop() {
  if(sign)
  {  
     sign=0;
     if(Re_buf[0]==0xAA && Re_buf[7]==0x55)        //检查帧头，帧尾
     {           
            YPR[0]=(Re_buf[1]<<8|Re_buf[2])/100;   //合成数据，去掉小数点后2位
            YPR[1]=(Re_buf[3]<<8|Re_buf[4])/100;
            YPR[2]=(Re_buf[5]<<8|Re_buf[6])/100;
            Serial.println(YPR[0],DEC);
            Serial.println(YPR[1],DEC);
            Serial.println(YPR[2],DEC);
            
       
   }
  } 
} 
//---------------------------------------------------------------
void serialEvent() {
  while (Serial.available()) {   
    Re_buf[counter]=(unsigned char)Serial.read();
    if(counter==0&&Re_buf[0]!=0xAA) return;      // 检查帧头         
    counter++;       
    if(counter==8)                //接收到数据
    {    
       counter=0;                 //重新赋值，准备下一帧数据的接收 
       sign=1;
    }      
  }
}

