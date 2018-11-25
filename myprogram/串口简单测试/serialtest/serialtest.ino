
void setup() {
  Serial.begin(115200); //设定串口的通讯波特率为115200BPS.
}
int dir_value;
int v_value;
int Bai;
int Shi;
int Ge;
int slot=0;
unsigned char income[4];
void loop() {
  if (Serial.available() > 0) 
  { 
    
     income[slot] = Serial.read(); 
     slot=slot+1;
     
  }
  if (slot==2 & income[0]=='D')
  {
      dir_value=int(income[1])-48;//int 强制转换只能得到ascii码值
      slot=0;
      Serial.println(dir_value);
    }
   else if (slot==4 & income[0]=='V')
   {
      Bai=int(income[1])-48;
      Shi=int(income[2])-48;
      Ge=int(income[3])-48;
      v_value=100*Bai+10*Shi+Ge;
      slot=0;
      Serial.println(v_value);
    }
}

