long value=0;
unsigned char Buffer;//定义接受4个字节的数组
int i=0;
unsigned char * p;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200,SERIAL_8N1);

}
void loop() 

{
   while(Serial.available()>0)
   {
    Buffer=Serial.read();
    Serial.write(Buffer);
    }

}

    
    

