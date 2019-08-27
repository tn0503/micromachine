#include <Wire.h>
 
// remote
const uint8_t interruptPin = 16;
boolean  rmReceived = 0;  //信号受信完了した
uint8_t  i;               //受信データの桁
uint8_t  rmState = 0;     //信号受信状況
uint8_t  dataCode;        //データコード(8bit)
uint8_t  invDataCode;     //反転データコード(8bit)
uint16_t customCode;      //カスタムコード(16bit)
uint32_t rmCode;          //コード全体(32bit)
volatile uint32_t prevMicros = 0; //時間計測用
 
// action
#define STOP  0
#define FWRD  1
#define BWRD  2
#define RTRN  3
#define LTRN  4
#define RGHT  5
#define LEFT  6
uint8_t actionMode = STOP;
int16_t tempAngles[12] = {90,90,90,90,90,90,90,90,90,90,90,90};
int16_t offset[12] =     {10,  4, -7,  8, 0,  0,  0,-10,  2, -2,-12, -2};
 
int16_t stopAngles[12] = {90,90,90,90,90,90,90,90,90,90,90,90};
int16_t leftAngles[6][13] = {
// LHR LHP LAP LAR LSP LSR RSR RSP RAR RAP RHP RHR time 
  { 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 8},
  { 90, 90, 90,100, 90, 90, 90, 90,100, 90, 90, 90, 8},
  {100, 90, 90,100, 90, 90, 90, 90,100, 90, 90, 90, 8},
  { 90, 90, 90, 80, 90, 90, 90, 90, 80, 90, 90, 80, 8},
  { 90, 90, 90, 80, 90, 90, 90, 90, 80, 90, 90, 90, 8},
  { 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 8}
};
int16_t rghtAngles[6][13] = {
// LHR LHP LAP LAR LSP LSR RSR RSP RAR RAP RHP RHR time 
  { 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 8},
  { 90, 90, 90, 80, 90, 90, 90, 90, 80, 90, 90, 90, 8},
  { 90, 90, 90, 80, 90, 90, 90, 90, 80, 90, 90, 80, 8},
  {100, 90, 90,100, 90, 90, 90, 90,100, 90, 90, 90, 8},
  { 90, 90, 90,100, 90, 90, 90, 90,100, 90, 90, 90, 8},
  { 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 8}
};
int16_t rtrnAngles[8][13] = {
// 常に右重心にしました。
//LHR LHP LAP LAR LSP LSR RSR RSP RAR RAP RHP RHR time
  {90,100,100, 90, 90, 90, 90, 90, 90,100,100, 90, 8},
  {90, 90, 90,100, 80, 90, 90, 80, 90,100,100, 90, 8}, //右重心
  {90, 90, 90,100, 70, 90, 90, 70, 90, 90, 90, 90, 4},
  {90, 90, 90,100, 80, 90, 90, 80, 90, 80, 80, 90, 4}, //右足前
  {90, 80, 80, 90, 90, 90, 90, 90, 90, 80, 80, 90, 8},
  {90, 80, 80,100,100, 90, 90,100,100, 90, 90, 90, 8}, //右重心
  {90, 90, 90, 90,110, 90, 90,110,100, 90, 90, 90, 4},
  {90,100,100, 90,100, 90, 90,100,100, 90, 90, 90, 4}  //左足前
};
int16_t ltrnAngles[8][13] = {
// 常に左重心にしました。
//LHR LHP LAP LAR LSP LSR RSR RSP RAR RAP RHP RHR time
  {90,100,100, 90, 90, 90, 90, 90, 90,100,100, 90, 8},
  {90, 90, 90, 80, 80, 90, 90, 80, 80,100,100, 90, 8}, //左重心
  {90, 90, 90, 80, 70, 90, 90, 70, 90, 90, 90, 90, 4},
  {90, 90, 90, 80, 80, 90, 90, 80, 90, 80, 80, 90, 4}, //右足前
  {90, 80, 80, 90, 90, 90, 90, 90, 90, 80, 80, 90, 8},
  {90, 80, 80, 90,100, 90, 90,100,100, 90, 90, 90, 8}, //左重心
  {90, 90, 90, 90,110, 90, 90,110,100, 90, 90, 90, 4},
  {90,100,100, 90,100, 90, 90,100,100, 90, 90, 90, 4}  //左足前
};
int16_t bwrdAngles[8][13] = { //後退
// 順番を逆にしました。
//LHR LHP LAP LAR LSP LSR RSR RSP RAR RAP RHP RHR time
  {90,100,100, 90,100, 90, 90,100,100, 90, 90, 90, 4}, //左足前
  {90, 90, 90, 90,110, 90, 90,110,100, 90, 90, 90, 4},
  {90, 80, 80,100,100, 90, 90,100,100, 90, 90, 90, 8}, //右重心
  {90, 80, 80, 90, 90, 90, 90, 90, 90, 80, 80, 90, 8},
  {90, 90, 90, 80, 80, 90, 90, 80, 90, 80, 80, 90, 4}, //右足前
  {90, 90, 90, 80, 70, 90, 90, 70, 90, 90, 90, 90, 4},
  {90, 90, 90, 80, 80, 90, 90, 80, 80,100,100, 90, 8}, //左重心
  {90,100,100, 90, 90, 90, 90, 90, 90,100,100, 90, 8}
};
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
int16_t motionAngles[8][13];
uint8_t maxRows;
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
    Wire.write(0x0);
    Wire.write(0x0>>8);
    Wire.write(angle);
    Wire.write(angle>>8);
  }
  Wire.endTransmission();
}
 
void rmUpdate() //信号が変化した
{
  uint32_t width; //パルスの幅を計測
  if(rmState != 0){
    width = micros() - prevMicros;  //時間間隔を計算
    if(width > 10000)rmState = 0; //長すぎ
    prevMicros = micros();  //次の計算用
  }
  switch(rmState){
    case 0: //信号未達
    prevMicros = micros();  //現在時刻(microseconds)を記憶
    rmState = 1;  //最初のOFF->ON信号を検出した
    i = 0;
    return;
    case 1: //最初のON状態
      if((width > 9500) || (width < 8500)){ //リーダーコード(9ms)ではない
        rmState = 0;
      }else{
        rmState = 2;  //ON->OFFで9ms検出
      }
      break;
    case 2: //9ms検出した
      if((width > 5000) || (width < 4000)){ //リーダーコード(4.5ms)ではない
        rmState = 0;
      }else{
        rmState = 3;  //OFF->ONで4.5ms検出
      }
      break;
    case 3: //4.5ms検出した
      if((width > 700) || (width < 400)){
        rmState = 0;
      }else{
        rmState = 4;  //ON->OFFで0.56ms検出した
      }
      break;
    case 4: //0.56ms検出した
      if((width > 1800) || (width < 400)){  //OFF期間(2.25-0.56)msより長い or (1.125-0.56)msより短い
        rmState = 0;
      }else{
        if(width > 1000){ //OFF期間長い -> 1
          bitSet(rmCode, (i));
        }else{             //OFF期間短い -> 0
          bitClear(rmCode, (i));
        }
        i++;  //次のbit
        if(i > 31){ //完了
          rmReceived = 1;
          return;
        }
        rmState = 3;  //次のON->OFFを待つ
      }
      break;
    }
}
  
void setup() {
  // put your setup code here, to run once:
  //Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(interruptPin), rmUpdate, CHANGE);
  //pinMode(14, OUTPUT);
  Wire.begin(21,22);
  Wire.setClock(100000);
 
  init_pca9685();
  set_angle();
   
  pinMode(19, OUTPUT);
  digitalWrite(19,LOW);
  Serial.begin(9600);  //追加
 
  divCounter = 0;
  keyFrame = 0;
  nextKeyFrame = 1;
}
  
void loop() {
  // put your main code here, to run repeatedly:
  if(rmReceived){ //リモコン受信した
    detachInterrupt(digitalPinToInterrupt(interruptPin));
    rmReceived = 0;   //初期化
    rmState = 0;      //初期化
    //図とは左右が逆であることに注意
    customCode = rmCode;    //下16bitがcustomCode
    dataCode = rmCode >> 16;  //下16bitを捨てたあとの下8bitがdataCode
    invDataCode = rmCode >> 24; //下24bitを捨てたあとの下8bitがinvDataCode
    if((dataCode + invDataCode) == 0xff){   //反転確認
      //Serial.println(dataCode);
      switch(dataCode){  //<-switch文を追加
        case 24:  //"2"ボタン
          actionMode = FWRD;
          memcpy(motionAngles, fwrdAngles, sizeof(fwrdAngles));
          maxRows = sizeof(fwrdAngles) / sizeof(*fwrdAngles) - 1;
          break;
        case 28:  //"5"ボタン
          actionMode = STOP;
          for(int i=0; i<12; i++) {
            tempAngles[i] = stopAngles[i];
          }
          set_angle();
          break;
        case 82:  //"8"ボタン
          actionMode = BWRD;
          memcpy(motionAngles, bwrdAngles, sizeof(bwrdAngles));
          maxRows = sizeof(bwrdAngles) / sizeof(*bwrdAngles) - 1;
          break;
        case 12:  //"1"ボタン
          actionMode = LTRN;
          memcpy(motionAngles, ltrnAngles, sizeof(ltrnAngles));
          maxRows = sizeof(ltrnAngles) / sizeof(*ltrnAngles) - 1;
          break;
        case 94:  //"3"ボタン
          actionMode = RTRN;
          memcpy(motionAngles, rtrnAngles, sizeof(rtrnAngles));
          maxRows = sizeof(rtrnAngles) / sizeof(*rtrnAngles) - 1;
          break;
        case 8:   //"4"ボタン
          actionMode = LEFT;
          memcpy(motionAngles, leftAngles, sizeof(leftAngles));
          maxRows = sizeof(leftAngles) / sizeof(*leftAngles) - 1;
          break;
        case 90:  //"6"ボタン
          actionMode = RGHT;
          memcpy(motionAngles, rghtAngles, sizeof(rghtAngles));
          maxRows = sizeof(rghtAngles) / sizeof(*rghtAngles) - 1;
          break;
        default:
          break;
      }
      divCounter = 0;
      keyFrame = 0;
      nextKeyFrame = 1;
    }
    attachInterrupt(digitalPinToInterrupt(interruptPin), rmUpdate, CHANGE);
  }
  if(actionMode != STOP){
    divCounter++;
    if(divCounter >= motionAngles[nextKeyFrame][12]) {
      divCounter = 0;
      keyFrame = nextKeyFrame;
      nextKeyFrame++;
      if(nextKeyFrame > maxRows) nextKeyFrame = 0;
    }
    for(int i=0; i<12; i++) {
      tempAngles[i] = motionAngles[keyFrame][i] +
            int8_t((motionAngles[nextKeyFrame][i] - motionAngles[keyFrame][i])
             * divCounter / motionAngles[nextKeyFrame][12]);
    }
    set_angle();
    delay(30);
  }
}
