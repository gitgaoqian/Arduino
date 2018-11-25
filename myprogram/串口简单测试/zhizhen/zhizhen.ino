
//设置一些变量
long value=0;//用来储存电机1的编码器记录的脉冲数*
unsigned char Buffer[4];//缓存上位机发送的字节;此处必须用unsigned char,因为char的范围是-127-128所以Buffer的值就达不到0XFF．
int i=0;//记录接受上位机发送的字节数
unsigned char * p;//定义指针，指向char类型，说明每次指针取值只能取一个字节


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200,SERIAL_8N1);
  p= (unsigned char *)&value;//指针类型的转换，利用char类型指针访问多字节整形变量的字节

}

void loop() {

    while(Serial.available()>0)
    {
        Buffer[i++]=Serial.read();
           
    }
    if(i==4)
    {
      //这里根据不同电脑字节顺序进行适配
           *(p+0)=Buffer[3];
           *(p+1)=Buffer[2];
           *(p+2)=Buffer[1];    
           *(p+3)=Buffer[0];      
           //或者采用下面这种方法:
//           p[0]=Buffer[3];
//           p[1]=Buffer[2];
//           p[2]=Buffer[1];    
//           p[3]=Buffer[0];           
           //Serial.print(value);
           Serial.print(78,OCT);
           
           i=0;     
      }

      
   
}




