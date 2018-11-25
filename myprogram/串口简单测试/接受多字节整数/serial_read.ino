long int incomingByte[3] ; 
long int value;
long int data;
int i = 0; 
void setup()  
{  
  Serial.begin(115200);  
  Serial.write(0x80);
}  
void loop()  
{
 
  for (i = 0;i<3;i++)
  {
      incomingByte[i] = Serial.read();
      
  }
  
    value = incomingByte[0]<<16+incomingByte[1]<<8+incomingByte[2];
    Serial.println(value);
 
 
}  
