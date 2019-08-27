#include "audiodata.h"
#include <Wire.h>
 
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
 
// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
 
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
 
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
 
// sound
int wavCounter;
hw_timer_t * timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
 
// another timer
hw_timer_t * timer2 = NULL;
volatile SemaphoreHandle_t timerSemaphore2;
 
// action
#define STOP  0
#define FWRD  1
#define BWRD  2
#define RTRN  3
#define LTRN  4
#define RGHT  5
#define LEFT  6
#define FWDBWD  7
#define RIGHTSTEP  8
#define RIGHTTURN  9
#define LEFTSTEP  10
#define LEFTTURN  11
#define ARM  12
#define LOAD  13

uint8_t actionMode = STOP;
int16_t tempAngles[12] = {90,90,90,90,90,90,90,90,90,90,90,90};
int16_t offset[12] = {10,  4, -7,  8,10,  0,  0, 0,  2, -2,-12, -2};
 
int16_t stopAngles[12] = {90,90,90,90,90,90,90,90,90,90,90,90};
int16_t loadAngles[6][13] = {
  { 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90,127},
  { 95, 90, 90, 95, 60, 90, 90,120, 85, 90, 90, 85, 6},
  { 95, 90, 90, 95, 60,140, 40,120, 85, 90, 90, 85, 8},
  { 95, 90, 90, 95, 60, 60,120,120, 85, 90, 90, 85, 6},
  { 95, 90, 90, 95, 60, 60,120,120, 85, 90, 90, 85,30},
  { 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 8}
};
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
  {90, 90, 90, 80, 80, 90, 90, 90, 80,100,100, 90, 8}, //左重心
  {90, 90, 90, 80, 70, 90, 90, 70, 70, 90, 90, 90, 4},
  {90, 90, 90, 80, 80, 90, 90, 90, 80, 80, 80, 90, 4}, //右足前
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
uint8_t svFlag = 0; //30msecの割り込みを知らせる
uint8_t dragLR;
uint8_t dragFrame;
uint8_t manuHandRX;
uint8_t manuHandRY;
uint8_t manuHandLX;
uint8_t manuHandLY;
 
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
 
void IRAM_ATTR onTimer(){
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
  if(wavCounter < sound_length) dacWrite(25,sound_data[wavCounter++]);
  else stop_playback();
}
 
void IRAM_ATTR onTimer2(){
  xSemaphoreGiveFromISR(timerSemaphore2, NULL);
  svFlag = 1;
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
 
void start_playback() {
  wavCounter = 0;
  timerAlarmEnable(timer);
  digitalWrite(2, LOW);
}
 
void stop_playback() {
  timerAlarmDisable(timer);
  digitalWrite(2, HIGH);
}
 
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();
  
      if (value.length() == 5) {
        byte bData = (uint8_t)value[0];
        switch(bData){
          case 127:
            bData = (uint8_t)value[1];
            dragLR = bData;
            bData = (uint8_t)value[2];
            dragFrame = bData;
            if(actionMode != FWDBWD){
              actionMode = FWDBWD;
              memcpy(motionAngles, fwrdAngles, sizeof(fwrdAngles));
              maxRows = sizeof(fwrdAngles) / sizeof(*fwrdAngles) - 1;
            }
            break;
          case 126:
            bData = (uint8_t)value[1];
            dragLR = bData;
            bData = (uint8_t)value[2];
            dragFrame = bData;
            if(actionMode != RIGHTSTEP){
              actionMode = RIGHTSTEP;
              memcpy(motionAngles, rghtAngles, sizeof(rghtAngles));
              maxRows = sizeof(rghtAngles) / sizeof(*rghtAngles) - 1;
            }
            break;
          case 125:
            bData = (uint8_t)value[1];
            dragLR = bData;
            bData = (uint8_t)value[2];
            dragFrame = bData;
            if(actionMode != RIGHTTURN){
              actionMode = RIGHTTURN;
              memcpy(motionAngles, rtrnAngles, sizeof(rtrnAngles));
              maxRows = sizeof(rtrnAngles) / sizeof(*rtrnAngles) - 1;
            }
            break;
          case 124:
            bData = (uint8_t)value[1];
            dragLR = bData;
            bData = (uint8_t)value[2];
            dragFrame = bData;
            if(actionMode != LEFTSTEP){
              actionMode = LEFTSTEP;
              memcpy(motionAngles, leftAngles, sizeof(leftAngles));
              maxRows = sizeof(leftAngles) / sizeof(*leftAngles) - 1;
            }
            break;
          case 123:
            bData = (uint8_t)value[1];
            dragLR = bData;
            bData = (uint8_t)value[2];
            dragFrame = bData;
            if(actionMode != LEFTTURN){
              actionMode = LEFTTURN;
              memcpy(motionAngles, ltrnAngles, sizeof(ltrnAngles));
              maxRows = sizeof(ltrnAngles) / sizeof(*ltrnAngles) - 1;
            }
            break;
          case 122:
          actionMode = ARM;
            bData = (uint8_t)value[1];
            manuHandRX = bData;
            bData = (uint8_t)value[2];
            manuHandRY = bData;
            bData = (uint8_t)value[3];
            manuHandLX = bData;
            bData = (uint8_t)value[4];
            manuHandLY = bData;
            break;
        }
        
      }else if(value.length() == 1) {
      //if (value.length() > 0) {
        //for (int i = 0; i < value.length(); i++){
          byte bData = (uint8_t)value[0];
          Serial.print(bData);
          if(bData == 0 || bData == 7 || bData == 9) {
            switch(bData){
            case 0:  //"0"ボタン
              start_playback();
              break;
            case 7:  //"7"ボタン  
              digitalWrite(14,HIGH);
              break;
            case 9:  //"9"ボタン  
              digitalWrite(14,LOW);
              break;
            default:
              break;
            }
          } else {
            switch(bData){  //<-switch文を追加
            case 2:  //"2"ボタン
              actionMode = FWRD;
              memcpy(motionAngles, fwrdAngles, sizeof(fwrdAngles));
              maxRows = sizeof(fwrdAngles) / sizeof(*fwrdAngles) - 1;
              break;
            case 5:  //"5"ボタン
              actionMode = STOP;
              for(int i=0; i<12; i++) {
                tempAngles[i] = stopAngles[i];
              }
              set_angle();
              break;
            case 8:  //"8"ボタン
              actionMode = BWRD;
              memcpy(motionAngles, bwrdAngles, sizeof(bwrdAngles));
              maxRows = sizeof(bwrdAngles) / sizeof(*bwrdAngles) - 1;
              break;
            case 1:  //"1"ボタン
              actionMode = LTRN;
              memcpy(motionAngles, ltrnAngles, sizeof(ltrnAngles));
              maxRows = sizeof(ltrnAngles) / sizeof(*ltrnAngles) - 1;
              break;
            case 3:  //"3"ボタン
              actionMode = RTRN;
              memcpy(motionAngles, rtrnAngles, sizeof(rtrnAngles));
              maxRows = sizeof(rtrnAngles) / sizeof(*rtrnAngles) - 1;
              break;
            case 4:   //"4"ボタン
              actionMode = LEFT;
              memcpy(motionAngles, leftAngles, sizeof(leftAngles));
              maxRows = sizeof(leftAngles) / sizeof(*leftAngles) - 1;
              break;
            case 6:  //"6"ボタン
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
        }  
      //}
    }
};
 
void setup() {
  //Serial.begin(115200);
  // audio
  timerSemaphore = xSemaphoreCreateBinary();
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 124, true);
   
  wavCounter = 0;
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
 
  attachInterrupt(digitalPinToInterrupt(interruptPin), rmUpdate, CHANGE);
 
  // led
  pinMode(14, OUTPUT);
  digitalWrite(14,LOW);
 
  // servo
  Wire.begin(21,22);
  Wire.setClock(100000);
 
  init_pca9685();
  set_angle();
   
  pinMode(19, OUTPUT);
  digitalWrite(19,LOW);
 
  timerSemaphore2 = xSemaphoreCreateBinary();
  timer2 = timerBegin(1, 80, true);
  timerAttachInterrupt(timer2, &onTimer2, true);
  timerAlarmWrite(timer2, 30000, true);
  timerAlarmEnable(timer2);
   
  // ble
  BLEDevice::init("MyESP32");
  BLEServer *pServer = BLEDevice::createServer();
 
  BLEService *pService = pServer->createService(SERVICE_UUID);
 
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
 
  pCharacteristic->setCallbacks(new MyCallbacks());
 
  pCharacteristic->setValue("Hello World");
  pService->start();
 
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();
}
 
void loop() {
  if(rmReceived){ //リモコン受信した
    detachInterrupt(digitalPinToInterrupt(interruptPin));
    rmReceived = 0;   //初期化
    rmState = 0;      //初期化
    //図とは左右が逆であることに注意
    customCode = rmCode;    //下16bitがcustomCode
    dataCode = rmCode >> 16;  //下16bitを捨てたあとの下8bitがdataCode
    invDataCode = rmCode >> 24; //下24bitを捨てたあとの下8bitがinvDataCode
    if((dataCode + invDataCode) == 0xff){   //反転確認
      if(dataCode == 22 || dataCode == 66 || dataCode == 74) {
        switch(dataCode){
        case 22:  //"0"ボタン
          start_playback();
          break;
        case 66:  //"7"ボタン  
          digitalWrite(14,HIGH);
          break;
        case 74:  //"9"ボタン  
          digitalWrite(14,LOW);
          break;
        default:
          break;
        }
      } else {
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
        case 13:  //"c"ボタン
          actionMode = LOAD;
          memcpy(motionAngles, loadAngles, sizeof(loadAngles));
          maxRows = sizeof(loadAngles) / sizeof(*loadAngles) - 1;
          break;
        default:
        break;
        }
        divCounter = 0;
        keyFrame = 0;
        nextKeyFrame = 1;
      }
    }
    attachInterrupt(digitalPinToInterrupt(interruptPin), rmUpdate, CHANGE);
  }
  if(actionMode != STOP && svFlag){
    svFlag = 0;
    if(actionMode == ARM){
      tempAngles[4] = 70-(manuHandLY-50)*1.5;
      tempAngles[5] = 90-(manuHandLX-50)*1.5;
      tempAngles[6] = 90+(manuHandRX-50)*1.5;
      tempAngles[7] = 110+(manuHandRY-50)*1.5;
    }else if(actionMode == FWDBWD
        || actionMode == RIGHTTURN
        || actionMode == LEFTTURN){
      if(dragLR == 1){
        if(dragFrame < 25){
          keyFrame = 0;
          divCounter = dragFrame;
        }else if(dragFrame < 50){
          keyFrame = 1;
          divCounter = dragFrame - 25;
        }else if(dragFrame < 75){
          keyFrame = 2;
          divCounter = dragFrame - 50;
        }else{
          keyFrame = 3;
          divCounter = dragFrame - 75;
        }
      }else{
        if(dragFrame < 25){
          keyFrame = 4;
          divCounter = dragFrame;
        }else if(dragFrame < 50){
          keyFrame = 5;
          divCounter = dragFrame - 25;
        }else if(dragFrame < 75){
          keyFrame = 6;
          divCounter = dragFrame - 50;
        }else{
          keyFrame = 7;
          divCounter = dragFrame - 75;
        }
      }
      nextKeyFrame = keyFrame + 1;
      if(nextKeyFrame > 7)nextKeyFrame = 0;
      for(int i=0; i<12; i++) {
        tempAngles[i] = motionAngles[keyFrame][i] +
            int8_t((motionAngles[nextKeyFrame][i] - motionAngles[keyFrame][i])
             * divCounter / 25);
      }
    }else if(actionMode == RIGHTSTEP
        || actionMode == LEFTSTEP){
      if(dragLR == 1){
        if(dragFrame < 33){
          keyFrame = 0;
          divCounter = dragFrame;
        }else if(dragFrame < 66){
          keyFrame = 1;
          divCounter = dragFrame - 33;
        }else{
          keyFrame = 2;
          divCounter = dragFrame - 66;
        }
      }else{
        if(dragFrame < 33){
          keyFrame = 3;
          divCounter = dragFrame;
        }else if(dragFrame < 66){
          keyFrame = 4;
          divCounter = dragFrame - 33;
        }else{
          keyFrame = 5;
          divCounter = dragFrame - 66;
        }
      }
      nextKeyFrame = keyFrame + 1;
      if(nextKeyFrame > 5)nextKeyFrame = 0;
      for(int i=0; i<12; i++) {
        tempAngles[i] = motionAngles[keyFrame][i] +
            int8_t((motionAngles[nextKeyFrame][i] - motionAngles[keyFrame][i])
             * divCounter / 33);
      }
    }else{
      divCounter++;
      if(divCounter >= motionAngles[nextKeyFrame][12]) {
        divCounter = 0;
        keyFrame = nextKeyFrame;
        nextKeyFrame++;
        if(nextKeyFrame > maxRows) nextKeyFrame = 0;
        if(motionAngles[nextKeyFrame][12] == 127)actionMode=STOP;
      }
      for(int i=0; i<12; i++) {
        tempAngles[i] = motionAngles[keyFrame][i] +
            int8_t((motionAngles[nextKeyFrame][i] - motionAngles[keyFrame][i])
             * divCounter / motionAngles[nextKeyFrame][12]);
      }
    }
     
    set_angle();
    //delay(30);
  }
}
