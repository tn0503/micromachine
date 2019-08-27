#include <Wire.h>
 
int16_t tempAngles[12] = {90,90,90,90,90,90,90,90,90,90,90,90};
int16_t offset[12] =     {10,  4, -5,  8, 0,  0,  0,-10,  4,  0,-12, -2};

int16_t fwrdAngles[8][13] = {
//LHR LHP LAP LAR LSP LSR RSR RSP RAR RAP RHP RHR time
  {90,100,100, 90, 90, 90, 90, 90, 90,100,100, 90, 8},
  {90, 90, 90, 80, 80, 90, 90, 80, 80,100,100, 90, 8}, //左重心
  {90, 90, 90, 80, 70, 90, 90, 70, 90, 90, 90, 90, 4},
  {90, 90, 90, 80, 80, 90, 90, 80, 90, 80, 80, 90, 4}, //右足前
  {90, 80, 80, 90, 90, 90, 90, 90, 90, 80, 80, 90, 8},
  {90, 80, 80,100,100, 90, 90,100,100, 90, 90, 90, 8}, //右重心
  {90, 90, 90, 90,110, 90, 90,110,100, 90, 90, 90, 4},
  {90,100,100, 90,100, 90, 90,100,100, 90, 90, 90, 4}  //左足前
};
uint8_t divCounter;
uint8_t keyFrame;
uint8_t nextKeyFrame;

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
    angle = 400 + (tempAngles[i] + offset[i] - 90)*2.8;
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
  Serial.begin(9600);

  divCounter = 0;
  keyFrame = 0;
  nextKeyFrame = 1;
}
 
void loop() {
  // put your main code here, to run repeatedly:
  divCounter++;
  if(divCounter >= fwrdAngles[nextKeyFrame][12]) {
    divCounter = 0;
    keyFrame = nextKeyFrame;
    nextKeyFrame++;
    if(nextKeyFrame > 7) nextKeyFrame = 0;
  }
  for(int i=0; i<12; i++) {
    tempAngles[i] = fwrdAngles[keyFrame][i] +
              int8_t((fwrdAngles[nextKeyFrame][i] - fwrdAngles[keyFrame][i])
               * divCounter / fwrdAngles[nextKeyFrame][12]);
  }
  set_angle();
  delay(30);
}
