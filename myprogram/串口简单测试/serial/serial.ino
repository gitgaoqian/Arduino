String indate="";
String val="";
char value_int;
int led=6;       //变量定义  indate是用来接收pc传来的数据、val暂存indate数据、led定义灯管脚。
void setup() {
  Serial.begin(9600);
  Serial.println("COM is ready");
  Serial.println("Input Command :");
  pinMode(led,OUTPUT);    //初始化设定
}

void loop() {
  while(Serial.available()>0)
  {
    //indate+=char(Serial.read());
    value_int=Serial.read();
    
    if(Serial.available()<=0)
     {
        //Serial.println(indate);
        Serial.println(value_int);
        }
  }    //读取电脑传来的数据
  if(indate.length()>0)
  {
     val=indate;   //将indate暂存到val中
     if(val=="open")   //判断val数据并进行下一步操作
       {
         digitalWrite(led,HIGH);
         Serial.println("Input Command :");
       }
     else if(val=="close")
      {
         digitalWrite(led,LOW);
         Serial.println("Input Command :");
      }
     else
      {
          Serial.println("COMMAND IS ERROR!!! Please input again.");
      }
  }   
    indate="";   //清空indate为下一次输入做准备
}
 
