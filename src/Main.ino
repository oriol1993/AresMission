#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>


// PINS
#define pushbutton 5
#define led_blue 3
#define led_red 4
#define led_green 5 // Raro
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10
#define pin_staging 8
#define pin_parachuting 9

// BOARD PARAMETERS
#define pb_dtshort 150
#define pb_dtlong 3000
#define blueled_blinkshort 100
#define blueled_blinklong 1000

// PERFORMANCE
#define t_burn 10.0 // temps combustio motor mirar -7s
#define h_ignite_min 3.0
#define h_parachute 4.0 //altura d'ignicio del parachute - 450m
#define h_max_parachute 8.0 //revisar
#define h_detect 2.0
#define barom_period 33
#define n_pages 32768

//BMP280 variable define
TinyGPS gps;
SoftwareSerial ss(6, 7);
Adafruit_BMP280 bmp;
Buffer cbuffer;
SPIFlash flash;
byte altByte[bytes_alt], timeByte[bytes_timestamp];
uint32_t address = 0;
//

uint8_t state=0;
uint8_t pb_ispressed=0;
uint8_t blink_state=0;
uint8_t led_blue_state = 0;
uint8_t led_blue_n = 0;
float value_baromax=0;
float value_barom=0;
float alt_base=0;
float flat, flon;
bool flag_barom=0;
bool flag_gps = 0;
uint32_t pb_ton;
uint32_t led_blue_tchange = 0;
uint32_t blueled_lastchange = 0;
uint32_t tbarom_last=0;
uint32_t t_ignite=0;
//  Data storage
byte altByte[bytes_alt], timeByte[bytes_timestamp];

void setup()
{
Serial.begin(9600);
ss.begin(9600);
Serial.println("REBOOT");
pinMode(led_blue,OUTPUT);
pinMode(led_red,OUTPUT);
pinMode(led_green,OUTPUT);
pinMode(pushbutton,INPUT_PULLUP);
pinMode(pin_staging, OUTPUT);
pinMode(pin_parachuting, OUTPUT);
Serial.println(F("BMP280 OK"));

if (!bmp.begin()) {
    Serial.println(F("Error BMP"));
      while (1);
    }
for(int i=0; i<25; i++){
  alt_base += bmp.readAltitude(1013.25)/25;
  delay(20);
}
Serial.print(F("Altitud Base = "));
Serial.println(alt_base);
}

void loop()
{
  read_buttn();
  updt_barom();
  updt_gps();
  igni_detect();
  igni_stage();
  igni_parac();
  //rset_flash();
  //writ_flash();
  //send_xbee();
  //send_info();
  updt_leds();
  //dump_flash();
}

void read_buttn(){
  // inputs: state, tstate_last
  // outputs: state, tstate_last, erase_flash
 bool pb_state = !digitalRead(pushbutton);
 uint32_t dt = millis()-pb_ton;
if(pb_ispressed){
  if(!pb_state){
    if(dt>pb_dtlong){
      pb_ispressed = false;
      Serial.println(F("LongPress"));
      pb_ton = millis();
      //erase_flash();
    }else if(dt>pb_dtshort){
      pb_ispressed = false;
      state = !state;
      pb_ton = millis();
      Serial.print(F("ShortPress: State = ")); Serial.println(state);

    }
  }
}else{
  if(pb_state && dt>pb_dtshort){pb_ton = millis(); pb_ispressed = true;}
}
}

void updt_accel(){
  //inputs: state, tignite, taccel_last
  //outputs: taccel_last, flag_accel, value_accel
}
void updt_barom(){
  //inputs: state, tbarom_last, value_baromax
  //outputs: tbarom_last, flag_barom, value_barom, value_baromax
uint32_t dt = millis()-tbarom_last;
  if(dt>barom_period && !flag_barom){
    tbarom_last = millis();
    flag_barom = true;
   }
   value_barom=bmp.readAltitude(1013.25)-alt_base;
   if(value_barom > value_baromax){
     value_baromax=value_barom;
     Serial.print(F("BMP280 valor maxim = "));
     Serial.print(value_baromax);
     Serial.println(F("m"));
     }
 }

void updt_gps(){
  //inputs: tgps_last
  //outputs: tgps_last, flag_gps, flat, flo
  while(ss.available()){
    char c = ss.read();
    if(gps.encode(c)){flag_gps = true;};
    }
  if (flag_gps)
  {
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    Serial.print(flat,6);
    Serial.print(",");
    Serial.println(flon,6);
    flag_gps = false;
  }
}
void igni_detect(){
  if((state == 1) && (value_barom > h_detect)){
    state = 2;
    t_ignite = millis();
    Serial.println(F("State = 2"));
  }
}
void igni_stage(){
  //inputs: t_ignite, value_barom
  //outputs: tstage, flag_stage
  if((millis()-t_ignite > t_burn) && (value_barom > h_ignite_min) && (state == 2)){
    state=3;
    digitalWrite(pin_staging, HIGH);
    Serial.println(F("State = 3"));
  }
}
void igni_parac(){
  //inputs: value_barom, value_baromax
  //outputs: tparac, flag_parac
if((value_barom < h_parachute) && (state == 3) && (value_baromax > h_max_parachute)){
  state=4;
  digitalWrite(pin_parachuting, HIGH);
  Serial.println(F("State = 4"));
}
}
void rset_flash(){
  //inputs: erase_flash
  //outputs:
}
void writ_flash(){
  //inputs: taccel_last, flag_accel, value_accel, tbarom_last, flag_barom, value_barom, tstage, flag_stage, tparac, flag_parac
  //outputs:
    if(last_state<state){
        code = state;
    }else{
        if(flag_barom){code = 5;}
        else if(flag_gps){code = 6;}
    }
    last_state = state;
    
    // Load TimeStamp and code data into buffer
    //[-- bit0--][ --bit1--]
    //[7-5 ][4-0][   7-0   ]
    //[code][  timestamp   ]
    
    timestamp = (micros()/100)%8192; // 2^13
    timeByte[1] = timestamp & 255;
    timeByte[0] = timestamp<<8;
    timeByte[0] |= code<<5;
    cbuffer.CarregarBuffer(timeByte, sizeof(timeByte));
    
    if(code==5){
        // Load BMP280 data into buffer
        flag_barom = false;
        float2byte(value_barom, altByte);  // Convert float to byte array
        cbuffer.CarregarBuffer(altByte, sizeof(altByte));
        return;
    }
    if(code==6){   
        // Load GPS data into buffer
        unsigned long age;
        flag_gps = false;
        gps.f_get_position(&flat, &flon, &age);
        float2byte(flat, altByte);  // Convert float to byte array
        cbuffer.CarregarBuffer(altByte, sizeof(altByte));
        float2byte(flon, altByte);  // Convert float to byte array
        cbuffer.CarregarBuffer(altByte, sizeof(altByte));
    }
}

float byte2float(byte bytes_array[]){
  float out_float;
  memcpy(&out_float, bytes_array, 4);   // Assign bytes to input array
  return out_float;
}

void float2byte(float val,byte bytes_array[]){
  memcpy(bytes_array, &val, 4);
}

void buffer2flash(){
  if(cbuffer.Check(256)) {//252
    DEBUG_PRINT(F("Writing page ")); DEBUG_PRINTLN(pg);
    passDataToFlash();
  }
}

void passDataToFlash(){
  cbuffer.DescarregarBuffer(bff,256);//252
  flash.writeByteArray(pg++, 0, bff, PAGESIZE, false);
  if(pg>n_pages){Serial.println(F("Max data reached!")); startstop();}
}

void send_xbee(){
  //inputs: state, tstage, flag_stage, tparac, flag_parac, tgps_last, flag_gps, value_gps
  //outputs:
}
void send_info(){
  //inputs: state, tstage, flag_stage, tparac, flag_parac, tgps_last, flag_gps, value_gps
  //outputs:
}

void updt_leds(){
  //inputs: state
  //outputs:
  uint32_t dt = millis()-led_blue_tchange;
  if(state < 1){
    led_blue_state = HIGH;
  }
  else{
    if(blink_state == 0){
      if(dt>blueled_blinklong){
        blink_state = 1;
        led_blue_state = HIGH;
        led_blue_tchange = millis();
        led_blue_n = 0;
      }
    }
    else if(led_blue_n+1==state*2){
        blink_state = 0;
        led_blue_state = LOW;
        led_blue_tchange = millis();
      }
      else if(dt>blueled_blinkshort){
        led_blue_state = !led_blue_state;
        led_blue_tchange = millis();
        led_blue_n++;
      }
  }
  digitalWrite(led_blue,led_blue_state);
}

void dump_flash(){
  //inputs: state
  //outputs:
}

