#include "audiodata.h"
 
// remote
const uint8_t interruptPin = 16;
boolean  rmRecieved = 0;  //信号受信完了した
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
 
void IRAM_ATTR onTimer(){
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
  if(wavCounter < sound_length) dacWrite(25,sound_data[wavCounter++]);
  else stop_playback();
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
          rmRecieved = 1;
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
 
void setup() {
  timerSemaphore = xSemaphoreCreateBinary();
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 124, true);
  //timerAlarmEnable(timer);
 
  wavCounter = 0;
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
 
  attachInterrupt(digitalPinToInterrupt(interruptPin), rmUpdate, CHANGE);
}
 
void loop() {
  if(rmRecieved){ //リモコン受信した
    detachInterrupt(digitalPinToInterrupt(interruptPin));
    rmRecieved = 0;   //初期化
    rmState = 0;      //初期化
    //図とは左右が逆であることに注意
    customCode = rmCode;    //下16bitがcustomCode
    dataCode = rmCode >> 16;  //下16bitを捨てたあとの下8bitがdataCode
    invDataCode = rmCode >> 24; //下24bitを捨てたあとの下8bitがinvDataCode
    if((dataCode + invDataCode) == 0xff){   //反転確認
      switch(dataCode){  //<-switch文を追加
        case 22:  //"0"ボタン
          start_playback();
        break;
        default:
        break;
      }
    }
    attachInterrupt(digitalPinToInterrupt(interruptPin), rmUpdate, CHANGE);
  }
}
