#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <U8g2lib.h>
#include "ADS1X15.h"

#define KNOB_BUTTON   PA2
#define KNOB_DT       PA0
#define KNOB_CLK      PA1
#define CURRENT_PWM   PB8
#define VOLTAGE_PWM   PB9
#define BUTTON1       PA3
#define BUTTON2       PA4
#define BUTTON3       PA5
#define BATTERY_ADC   PA7
#define BACKLIGHT     PB6

#define REFRESHRATEMS     333
#define DEBOUNCETIME      100
#define DEBOUNCETIMEBUTTON 300
#define MAXVSET           6000
#define MAXASET           1200
#define MINVSET           1330
#define MINASET           0

U8G2_ST7565_NHD_C12864_F_4W_HW_SPI u8g2(U8G2_R2, /* cs=*/ PA8, /* dc=*/ PA10, /* reset=*/ PA9);
TwoWire WireCustom(PB11, PB10);
ADS1115 ADS(0x48, &WireCustom);

unsigned long totalTime = 0;
int knob_rot = 0;
int knob_debouncer = 0;
int knob_butt_debouncer = 0;
int knob_butt = 0;
int cursor = 0;
int change_set = 0;
int mVset = 2000;
int mAset = 100;
int mVrel = 2000;
int mArel = 25;
uint32_t idling = 0;
int vBat;
bool power_on = false;
bool is_low_power_mode = false;
uint16_t refresh_rate_ms = REFRESHRATEMS;


const char* getTimeStr() {
  static char timeStr[10];
  unsigned long seconds = millis() / 1000;
  unsigned int hours = seconds / 3600;
  unsigned int minutes = (seconds % 3600) / 60;
  unsigned int remainingSeconds = seconds % 60;
  //unsigned int tens = millis() % 1000 / 100;
  snprintf(timeStr, sizeof(timeStr), "%02u:%02u:%02u", hours, minutes, remainingSeconds);
  return timeStr;
}

void knobIT(){
  //Serial.println("knob");
  if(knob_debouncer == 0 and digitalRead(KNOB_CLK) and digitalRead(KNOB_BUTTON)){
    knob_debouncer = millis();
    if(digitalRead(KNOB_DT) == HIGH){
      knob_rot--;
    }else{
      knob_rot++;
    }
  }
}

void knobbuttIT(){
  //Serial.println("but");
  if(!knob_butt_debouncer and !digitalRead(KNOB_BUTTON)){
    knob_butt_debouncer = millis();
    knob_butt++;
  }
}

void debounce(){
  if(knob_debouncer){
    if(millis()>(knob_debouncer+DEBOUNCETIME)){
      knob_debouncer = 0;
    }
  }
  if(knob_butt_debouncer){
    if(millis()>(knob_butt_debouncer+DEBOUNCETIMEBUTTON)){
      knob_butt_debouncer = 0;
    }
  }
}

void blink(){
  digitalWrite(PC13, LOW);
  delay(2);
  digitalWrite(PC13, HIGH);
}

void generate_pwm(){
  analogWrite(CURRENT_PWM, mAset*53.7);
  analogWrite(VOLTAGE_PWM, (mVset-1250)*13.27);
}

void idle(uint32_t ms = 1){
  uint32_t tim;
  tim = micros();
  for(; ms; ms--){
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
  }
  idling += micros() - tim;
}

int get_free_cpu(){
  static uint32_t last_get;
  int ret = idling * 100 / (micros() - last_get);
  last_get = micros();
  idling = 0;
  return ret; 
}

void setup(void) {
  pinMode(PC13, OUTPUT);
  digitalWrite(PC13, HIGH); //Boarde led off

  pinMode(KNOB_CLK, INPUT);
  pinMode(KNOB_BUTTON, INPUT);  //button
  pinMode(KNOB_DT, INPUT); 
  attachInterrupt(digitalPinToInterrupt(KNOB_CLK), knobIT, RISING);
  attachInterrupt(digitalPinToInterrupt(KNOB_BUTTON), knobbuttIT, FALLING);

  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
  pinMode(BUTTON3, INPUT_PULLUP);
  pinMode(BATTERY_ADC, INPUT_ANALOG);
  pinMode(BACKLIGHT, OUTPUT); 
  digitalWrite(BACKLIGHT, HIGH);//LCD Backlight on

  //Serial.setRx(PB7);
  //Serial.setTx(PB6);
  //Serial.begin(2E6);

  SPI.setMOSI(PB5);
  SPI.setSCLK(PB3);
  SPI.begin();
  
  u8g2.begin();
  u8g2.setContrast(128); 

  WireCustom.begin();
  if (!ADS.begin()) {
    blink();
  }
  analogReadResolution(12);
  analogWriteResolution(16);
  analogWrite(PB9, 0);
  analogWrite(PB8, 0);

}

void set_value(int* pSet, int curs, int kr){
  switch (curs) {
    case 1:
    (*pSet) += kr*1000;
    break;
    case 2:
    (*pSet) += kr*100;
    break;
    case 3:
    (*pSet) += kr*10;
    break;
    case 4:
    (*pSet) += kr;
    break;
  }
}

void drawscreen(){
  static bool blink;
  u8g2.clearBuffer();		
  u8g2.setFont(u8g2_font_t0_11_me );	

  char buffer [22];
  snprintf(buffer, sizeof(buffer), "SET mV:%04d mA:%04d", mVset, mAset);
  
  if(knob_butt){ //knob_butt
    knob_butt=0;
    cursor++;
    if(cursor>4){
      cursor = 0;
    }
  }

  if(knob_rot){
    if(change_set == 1)
      set_value(&mVset, cursor, knob_rot);
    if(change_set == 2)
      set_value(&mAset, cursor, knob_rot);
    knob_rot = 0;
    if (mVset < MINVSET)
      mVset = MINVSET;
    else if (mVset > MAXVSET)
      mVset = MAXVSET;
    if (mAset > MAXASET)
      mAset = MAXASET;
    else if(mAset < MINASET)
      mAset = MINASET;
  }
 
  if(cursor and blink){
    if(change_set == 1)
      buffer[cursor+6] = '_';
    else if(change_set == 2)
      buffer[cursor+14] = '_';
    blink = false;
  }else{
    blink = true;
  }
  
  u8g2.drawStr(0, 8, buffer); //SET
  u8g2.setFont(u8g2_font_5x8_mr );
  u8g2.drawStr(72, 64, getTimeStr());
  snprintf(buffer, sizeof(buffer), "idle:%02d%%", get_free_cpu());
  u8g2.drawStr(72, 56, buffer);
  snprintf(buffer, sizeof(buffer), "vBat:%d.%02dV", vBat/100, vBat%100);
  u8g2.drawStr(72, 48, buffer);
  int power_loss = (((vBat*10)-mVrel)*mArel)/1E5;
  snprintf(buffer, sizeof(buffer), "PL:%1d.%1dW", power_loss/10, power_loss%10);
  u8g2.drawStr(72, 40, buffer);

  u8g2.setFont(u8g2_font_ncenB10_tr);
  snprintf(buffer, sizeof(buffer), "%s", power_on ? "ON/-" : "-/OFF");
  u8g2.drawStr(72, 25, buffer);


  u8g2.setFont(u8g2_font_ncenB10_tr);
  snprintf(buffer, sizeof(buffer), "%04dmV", mVrel);
  u8g2.drawStr(0, 25, buffer);
  snprintf(buffer, sizeof(buffer), "%04dmA", mArel);
  u8g2.drawStr(0, 42, buffer);
  snprintf(buffer, sizeof(buffer), "%04dmW", (mArel*mVrel)/1000);
  u8g2.drawStr(0, 59, buffer);

  //snprintf(buffer, sizeof(buffer), "%d, %d", knob_butt, knob_rot);
  //u8g2.drawStr(0, 40, buffer);

  u8g2.sendBuffer();
}

void measure() {
  ADS.setDataRate(5);
  ADS.setGain(2);
  ADS.requestADC_Differential_2_3();
  idle(5);
  while(ADS.isBusy()){
    idle();
  } 
  uint16_t vout = ADS.getValue();

  ADS.setGain(16);
  ADS.requestADC(0);
  idle(5);
  while(ADS.isBusy()){
    idle();
  }  
  int16_t aout = ADS.getValue();

  if (aout < 0)
    aout = 0;

  mVrel = vout / 5.04;
  mArel = aout / 25.8;

  vBat = analogRead(BATTERY_ADC) * 0.2545; 
}

void low_power_mode(){
  if(is_low_power_mode){
    pinMode(BACKLIGHT, OUTPUT); 
    digitalWrite(BACKLIGHT, HIGH);
    refresh_rate_ms = REFRESHRATEMS;
    is_low_power_mode = false;
  }else{
    pinMode(BACKLIGHT, INPUT_PULLUP);
    refresh_rate_ms = 2000;
    is_low_power_mode = true;
  }

}

void buttons_checking(){
  static int but2_counter;
  if(digitalRead(BUTTON1) == LOW){
    generate_pwm();
    power_on = true;
    blink();
  }

  if(digitalRead(BUTTON2) == LOW){
    but2_counter++;
    analogWrite(CURRENT_PWM, 0);
    analogWrite(VOLTAGE_PWM, 0);
    power_on = false;
    blink();
    if(but2_counter >= 5){
      low_power_mode();
      but2_counter = 0;
    }
  }else{
    but2_counter=0;
  }
  
  if(digitalRead(BUTTON3) == LOW){
    change_set++;
    knob_butt=1;
    cursor=1;
    if(change_set>2)
      change_set = 0;
  }
}

void loop(void) {
  static uint32_t drawscreen_millis, printing;
  //Main loop code start
  debounce();
  if(millis() >= drawscreen_millis + refresh_rate_ms){
    drawscreen_millis = millis();
    measure();
    drawscreen();
  }
  if(millis() >= printing + 200){
    printing = millis();
    buttons_checking();
  }
  


  //Main loop code end
  idle();
}


extern "C" void HardFault_Handler(void) {
  while(1){
    digitalWrite(PC13, LOW);
    delay(200);
    digitalWrite(PC13, HIGH);
    delay(1800);
  }
}
extern "C" void MemManage_Handler(void) {
  while(1){
    digitalWrite(PC13, LOW);
    delay(800);
    digitalWrite(PC13, HIGH);
    delay(1200);
  }
}