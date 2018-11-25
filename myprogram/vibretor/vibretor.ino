const int vibetorPin =  11;
int value = 0;
int slot = 0;
int Bai = 0;
int Shi = 0;
int Ge = 0;
void setup ()
{
  pinMode(vibetorPin,OUTPUT);
  Serial.begin(115200); 

}
char incomeByte[3];

void loop()
{
         
    if (Serial.available() > 0) 
    {
    	incomeByte[slot] = Serial.read();
    	slot = slot+1;
    }
    else
    {
           if (slot == 1)
        {
          Bai = 0;
          Shi = 0;
          Ge=int(incomeByte[0])-48;
          slot = 0;
        }
    
        if (slot == 2)
        {
          Bai = 0;
          Shi = int(incomeByte[0])-48;
          Ge = int(incomeByte[1])-48;
          slot = 0;
        }
    
        if (slot==3)
       {  
          Bai=int(incomeByte[0])-48;
          Shi=int(incomeByte[1])-48;
          Ge=int(incomeByte[2])-48;
          slot = 0;
        }
          
      }
   
  	value=100*Bai+10*Shi+Ge;
    analogWrite(vibetorPin,value);
    delay(100);                  
                           /*完成一个循环后等待的时间,单位毫秒*/
}
