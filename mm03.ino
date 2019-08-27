#include <Wire.h>
 
int16_t tempAngles[12] = {90,90,90,90,90,90,90,90,90,90,90,90};
 
void init_pca9685() {
  Wire.beginTransmission(0x40);
  Wire.write(0x0);
  Wire.write(0x80);   //reset device
  Wire.endTransmission();
   
  Wire.beginTransmission(0x40);
  Wire.write(0x0);
  Wire.write(0x10);   //sleep mode
  Wire.endTransmission();
   
  Wire.beginTransmission(0x40);
  Wire.write(0xFE);
  Wire.write(0x65); //set prescaler to 60Hz
  Wire.endTransmission();
   
  Wire.beginTransmission(0x40);
  Wire.write(0x0);
  Wire.write(0x80);   //back to prev mode
  Wire.endTransmission();
   
  Wire.beginTransmission(0x40);
  Wire.write(0x0);
  Wire.write(0xa0);   //enable auto-increment
  Wire.endTransmission();
}
 
void set_angle() {
  int angle;
  Wire.beginTransmission(0x40);
  Wire.write(6);
  for(int i=0;i<12;i++){
    angle = 400 + (tempAngles[i] - 90)*2.8;
    Wire.write(0x0);       //led_on lower byte
    Wire.write(0x0>>8);    //led_on upper byte
    Wire.write(angle);     //led_off lower byte
    Wire.write(angle>>8);  //led_off upper byte
  }
  Wire.endTransmission();
}
 
void setup() {
  // put your setup code here, to run once:
  Wire.begin(21,22);
  Wire.setClock(100000);
 
  init_pca9685();
  set_angle();
   
  pinMode(19, OUTPUT);
  digitalWrite(19,LOW);
}
 
void loop() {
  // put your main code here, to run repeatedly:
 
}
