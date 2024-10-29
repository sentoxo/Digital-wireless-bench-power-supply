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
#define MINVSET           1300
#define MINASET           0

U8G2_ST7565_NHD_C12864_F_4W_HW_SPI u8g2(U8G2_R2, /* cs=*/ PA8, /* dc=*/ PA10, /* reset=*/ PA9);
TwoWire WireCustom(PB11, PB10);
ADS1115 ADS(0x48, &WireCustom);

unsigned long totalTime = 0;
int knob_rot = 0;
uint32_t knob_debouncer = 0;
uint32_t knob_butt_debouncer = 0;
int knob_butt = 0;
int cursor = 0;
int change_set = 0;
int mVset = 2700;
int mAset = 100;
int mVrel = 2000;
int mArel = 25;
uint32_t idling = 0;
int vBat;
bool power_on = false;
bool is_low_power_mode = false;
uint16_t refresh_rate_ms = REFRESHRATEMS;
const char* message;
bool warning_state = false;
int mode = 1;
uint32_t mAh = 0;
uint32_t mWh = 0;
char message2[20];
uint16_t osc[128];
uint8_t osc_i;
uint16_t osc_millis = 16;
uint8_t current_offset = 70;

void warning(){
  static bool blink_state;
  if(warning_state){
    if(blink_state){
      digitalWrite(BACKLIGHT, LOW);
      blink_state = false;
    }else{
      digitalWrite(BACKLIGHT, HIGH);
      blink_state = true;
    }
  }
}

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
  analogWrite(CURRENT_PWM, mAset*53 + 40);
  analogWrite(VOLTAGE_PWM, (mVset-1250)*13.27);
}

void power_off(){
  analogWrite(CURRENT_PWM, 0);
  analogWrite(VOLTAGE_PWM, 0);
  power_on = false;
}

void idle(uint32_t ms = 1){
  uint32_t tim;
  tim = micros();
  for(; ms; ms--){
    //HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
    delay(1);
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

void setup_watchdog(){
  // STMF103 Reference manual, Independent watchdog (IWDG), page 476/1096
  IWDG->KR = 0x5555;  // Enable write access
  IWDG->PR = 1;       // Set prescaler to 8
  IWDG->RLR = 0xFFF;  // Set reload value for 819ms
  IWDG->KR = 0xAAAA;  // Reaload
  IWDG->KR = 0xCCCC;  // Start the watchdog
}

void reload_watchdog(){
  IWDG->KR = 0xAAAA; 
}

void setup(void) {
  setup_watchdog();

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
    if(power_on) generate_pwm();
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
  snprintf(buffer, sizeof(buffer), "%s %02d%%", getTimeStr(), get_free_cpu());
  u8g2.drawStr(72, 64, buffer);
  //snprintf(buffer, sizeof(buffer), "idle:%02d%%", get_free_cpu());
  //u8g2.drawStr(72, 56, buffer);
  int power_loss = (((vBat*10)-mVrel)*(mArel/10))/1E5;
  snprintf(buffer, sizeof(buffer), "%d.%02dV %1d.%1dW", vBat/100, vBat%100, power_loss/10, power_loss%10);
  u8g2.drawStr(72, 56, buffer);
  //snprintf(buffer, sizeof(buffer), "PL:%1d.%1dW", power_loss/10, power_loss%10);
  //u8g2.drawStr(72, 48, buffer);
  snprintf(buffer, sizeof(buffer), "%s", message ? message : "___________");
  u8g2.drawStr(72, 48, buffer);
  //snprintf(buffer, sizeof(buffer), "%s", message2);
  //u8g2.drawStr(72, 40, buffer);


  u8g2.setFont(u8g2_font_ncenB10_tr);
  snprintf(buffer, sizeof(buffer), "%s", power_on ? "ON/-" : "-/OFF");
  u8g2.drawStr(72, 25, buffer);


  u8g2.setFont(u8g2_font_ncenB10_tr);
  snprintf(buffer, sizeof(buffer), "%04dmV", mVrel);
  u8g2.drawStr(0, 25, buffer);
  snprintf(buffer, sizeof(buffer), "%03d.%dmA", mArel/10, mArel%10);
  u8g2.drawStr(0, 42, buffer);
  if(mode == 1){
    snprintf(buffer, sizeof(buffer), "%04dmW", (mArel/10*mVrel)/1000);
  }else if(mode == 2){
    snprintf(buffer, sizeof(buffer), "%04lumWh", mWh / 1000);
  }
  u8g2.drawStr(0, 59, buffer);

  //snprintf(buffer, sizeof(buffer), "%d, %d", knob_butt, knob_rot);
  //u8g2.drawStr(0, 40, buffer);

  u8g2.sendBuffer();
}

void drawscreen2(){
  char buffer [22];
  u8g2.clearBuffer();		
  static uint8_t prevY;
  for (uint8_t x = 0 ; x < 128; x++) {
    uint8_t y = 63 - (osc[x] / 10.0 / mAset * 64.0);
    u8g2.drawLine(x, prevY, x, y);
    //u8g2.drawPixel(x,y);
    prevY = y;
  }
  u8g2.drawLine(osc_i, 0, osc_i, 64);
  u8g2.setFont(u8g2_font_5x8_mr);
  snprintf(buffer, sizeof(buffer), "%d%% %dmV %dmA %ums", get_free_cpu(), mVrel, mArel/10, osc_millis);
  u8g2.drawStr(0, 7, buffer);
  u8g2.sendBuffer();
  
}

void measure() {
  static uint32_t delta_ms;
  ADS.setDataRate(5);
  ADS.setGain(2);
  ADS.requestADC_Differential_2_3();
  idle(5);
  while(ADS.isBusy()){
    idle();
  } 
  int16_t vout = ADS.getValue();
  if(vout<0) vout=0;

  ADS.setGain(16);
  ADS.requestADC(0);
  idle(5);
  while(ADS.isBusy()){
    idle();
  }  
  int16_t aout = ADS.getValue();
  snprintf(message2, 20, "%d0", aout); //debug
  aout += current_offset; // Zero it, fix value
  if (aout < 0)
    aout = 0;

  mVrel = vout / 5.04;
  mArel = aout / 2.6;

  mAh += mArel/10 * ((millis() - delta_ms) / 1000.0);
  mWh += (mArel/10 * mVrel) * ((millis() - delta_ms) / 3600000.0 );
  delta_ms = millis();

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
  static int but3_counter;
  if(digitalRead(BUTTON1) == LOW){
    if(power_on){
      power_off();
    }else{
      generate_pwm();
      power_on = true;
    }
    if(message) message = 0;
    blink();
  }

  if(digitalRead(BUTTON2) == LOW){
    if(mode == 3){
      osc_millis *= 2;
      if(osc_millis > 1000) osc_millis = 16;

    }else{
      change_set++;
      knob_butt=1;
      cursor=1;
      if(change_set>2) change_set = 0;
    }
    blink();
  }

  if(digitalRead(BUTTON3) == LOW){
    mode++;
    if(mode>3) mode = 1;
    but3_counter++;
    if(but3_counter >= 5){
      low_power_mode();
      but3_counter = 0;
    }
  }else{
    but3_counter=0;
  }
  
  
}

// In case of overvoltage on output, power off.
void check_voltage(){
  if(mVrel > (mVset*1.02)){
    power_off();
    message = "OVERVOLTAGE";
  }

  if(mArel/10 > (mAset*1.5)){
    power_off();
    message = "OVERCURRENT";
  }

  if(vBat < 610){
    power_off();
    warning_state = true;
    message = "LOW BAT!";
  }else if(vBat < 630){
    warning_state = true;
  }else{
    warning_state = false;
  }
}

void loop(void) {
  static uint32_t drawscreen_millis, twohundred_millis, oscilloscope_millis;
  //Main loop code start
  debounce();
  if(millis() >= drawscreen_millis + refresh_rate_ms){
    drawscreen_millis = millis();
    measure();
    check_voltage();
    if(mode == 3) drawscreen2();
    else drawscreen();
  }
  if(millis() >= twohundred_millis + 200){
    twohundred_millis = millis();
    buttons_checking();
    warning();
  }
  if((mode == 3) && (millis() >= oscilloscope_millis + osc_millis)){
    oscilloscope_millis = millis();
    ADS.setGain(16);
    int16_t aout = ADS.readADC(0) + current_offset;
    if(aout < 0) aout = 0;
    osc[osc_i++] = aout / 2.6;
    if(osc_i>127) osc_i=0;
  }


  //Main loop code end
  reload_watchdog();
  idle(1);
}


extern "C" void HardFault_Handler(void) {
  power_off();
  while(1){
    digitalWrite(PC13, LOW);
    delay(200);
    digitalWrite(PC13, HIGH);
    delay(1800);
  }
}
extern "C" void MemManage_Handler(void) {
  power_off();
  while(1){
    digitalWrite(PC13, LOW);
    delay(800);
    digitalWrite(PC13, HIGH);
    delay(1200);
  }
}