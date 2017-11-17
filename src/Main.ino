#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <Buffer.h>
#include <SPIFlash.h>

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
#define serial_init 1
#define t_burn 10.0 // temps combustio motor mirar -7s
#define h_ignite_min 3.0
#define h_parachute 4.0 //altura d'ignicio del parachute - 450m
#define h_max_parachute 8.0 //revisar
#define h_detect 2.0
#define barom_period 33
#define gps_period 5000
#define n_pages 32768

#define bytes_gps 4
#define bytes_alt 4
#define bytes_header 2

#define DEBUG
#ifdef DEBUG
 #define DEBUG_PRINT(x)     Serial.print (x)
 #define DEBUG_PRINTLN(x)  Serial.println (x)
#else
 #define DEBUG_PRINT(x)
 #define DEBUG_PRINTLN(x)
#endif

//BMP280 variable define
TinyGPS gps;
SoftwareSerial ss(6, 7);
Adafruit_BMP280 bmp;
Buffer cbuffer;
SPIFlash flash;

byte altByte[bytes_alt];
byte gpsByte[bytes_gps];
byte headerByte[bytes_header];
uint8_t code;
uint32_t address = 0;
//  Addresses
uint16_t pg = 0;
uint8_t bff[256];
uint16_t timestamp = 0, timestamp_ant = 0;
float timestamp_s = 0;

uint8_t state=0, last_state=0;
uint8_t pb_ispressed=0;
uint8_t blink_state=0;
uint8_t led_blue_state = 0;
uint8_t led_blue_n = 0;
float value_baromax=0;
float value_barom=0;
float alt_base=0;
float flat = -1, flon = -1;
bool flag_barom=0;
bool flag_gps = 0;
uint32_t pb_ton;
uint32_t led_blue_tchange = 0;
uint32_t blueled_lastchange = 0;
uint32_t tbarom_last=0;
uint32_t tgps_last = 0;
uint32_t t_ignite=0;

void setup()
{
Serial.begin(115200);
ss.begin(9600);
Serial.println("REBOOT");
pinMode(led_blue,OUTPUT);
pinMode(led_red,OUTPUT);
pinMode(led_green,OUTPUT);
pinMode(pushbutton,INPUT_PULLUP);
pinMode(pin_staging, OUTPUT);
pinMode(pin_parachuting, OUTPUT);

// BMP Init
Serial.print(F("BMP."));
if (!bmp.begin()) {
  Serial.println(F("0"));
  while(1);
}
Serial.println(F("1"));
// Flash Initialization
Serial.print(F("FLASH."));
flash.begin();
delay(500);
if (!flash.begin()) {
  Serial.println(F("0"));
  while (1);
}
else {Serial.println(F("1"));}


for(int i=0; i<25; i++){
  alt_base += bmp.readAltitude(1013.25)/25;
  delay(20);
}
value_barom = alt_base;
Serial.print(F("h0="));
Serial.println(alt_base);
cbuffer.reset(); // Buffer Initialization (head = 0; tail = 0)
send_info();
}

void loop()
{
  checkSerial();
  read_buttn();
  updt_barom();
  updt_gps();
  igni_detect();
  igni_stage();
  igni_parac();
  //rset_flash();
  writ_flash();
  buffer2flash();
  //send_xbee();
  updt_leds();
}

void send_info(){
  for(uint8_t i=0;i<10;i++){Serial.print(F("-"));}
  for(uint8_t i=0;i<2;i++){Serial.println();}
  Serial.print(F("S")); Serial.print(state);
  Serial.print(F("h"));Serial.print(value_barom,2);
  Serial.print(F("lat"));Serial.print(flat,4);
  Serial.print(F("lon"));Serial.println(flon,4);
  Serial.println(F("1Erase"));
  Serial.println(F("2Erase req"));
  Serial.println(F("3SS"));
  Serial.println(F("4Dump"));
}

void checkSerial() {
  if(serial_init){
    if (Serial.available() > 0) {
      switchTask(Serial.parseInt());
    }
  }
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
      Serial.println(F("PBL"));
      pb_ton = millis();
      //erase_flash();
    }else if(dt>pb_dtshort){
      pb_ispressed = false;
      state = !state;
      cbuffer.reset();
      pb_ton = millis();
      Serial.print(F("PBS State")); Serial.println(state);

    }
  }
}else{
  if(pb_state && dt>pb_dtshort){pb_ton = millis(); pb_ispressed = true;}
}
}

void updt_barom(){
  //inputs: state, tbarom_last, value_baromax
  //outputs: tbarom_last, flag_barom, value_barom, value_baromax
  if(state>0){
    uint32_t dt = millis()-tbarom_last;
    if(dt>barom_period && !flag_barom){
      tbarom_last = millis();
      flag_barom = true;
     }
     value_barom=bmp.readAltitude(1013.25)-alt_base;
     if(value_barom > value_baromax){
       value_baromax=value_barom;
       DEBUG_PRINT(F("t="));
       DEBUG_PRINT(millis()*0.001);
       DEBUG_PRINT(F(",hmx="));
       DEBUG_PRINTLN(value_baromax);
       }
  }
 }

void updt_gps(){
  //inputs: tgps_last
  //outputs: tgps_last, flag_gps, flat, flo
  if(state>0){
    uint32_t dt = millis()-tgps_last;
    if(dt>gps_period && !flag_gps){
      Serial.println("GPSTime!");
      tgps_last = millis();
      while(ss.available()){
        char c = ss.read();
        if(gps.encode(c)){flag_gps = true;Serial.println("GPSData!");};
        if(flag_gps){break;}
      }
      if (flag_gps)
      {
        unsigned long age;
        gps.f_get_position(&flat, &flon, &age);
        DEBUG_PRINT(F("t="));
        DEBUG_PRINT(millis()*0.001);
        DEBUG_PRINT(",lat=");
        DEBUG_PRINT(flat);
        DEBUG_PRINT(",lon=");
        DEBUG_PRINTLN(flon);
      }
  }
  }
}
void igni_detect(){
  if((state == 1) && (value_barom > h_detect)){
    state = 2;
    t_ignite = millis();
    Serial.println(F("S2"));
  }
}
void igni_stage(){
  //inputs: t_ignite, value_barom
  //outputs: tstage, flag_stage
  if((millis()-t_ignite > t_burn) && (value_barom > h_ignite_min) && (state == 2)){
    state=3;
    digitalWrite(pin_staging, HIGH);
    Serial.println(F("S3"));
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
    byte databyte[4];
    code = 0;
    if(last_state<state){
        code = state;
    }else{
        if(flag_barom){code = 5;}
        else if(flag_gps){code = 6;}
    }
    last_state = state;

    if(code>0){
      // Load TimeStamp and code data into buffer
      //[--byte0--][--byte1--]
      //[7-5 ][4-0][   7-0   ]
      //[code][  timestamp   ]
  
      timestamp = (micros()/100)%8192; // 2^13
      headerByte[1] = timestamp & 255;
      headerByte[0] = timestamp>>8;
      headerByte[0] = headerByte[0] | code<<5;
      cbuffer.CarregarBuffer(headerByte, sizeof(headerByte));
  
      if(code==5){
          // Load BMP280 data into buffer
          flag_barom = false;
          float2byte(value_barom, altByte);  // Convert float to byte array
          cbuffer.CarregarBuffer(altByte, sizeof(altByte));
          return;
      }
      if(code==6){
          // Load GPS data into buffer
          flag_gps = false;
          //while(ss.available()){ss.read();}
          //gps.f_get_position(&flat, &flon, &age);
          float2byte(flat, databyte);  // Convert float to byte array
          cbuffer.CarregarBuffer(databyte, sizeof(databyte));
          float2byte(flon, databyte);  // Convert float to byte array
          cbuffer.CarregarBuffer(databyte, sizeof(databyte));
      }
    }
}

void printAllPages(bool doprint) {
  bool nd;
  DEBUG_PRINTLN("Reading all pages");
  cbuffer.reset();
  pg = 0;
  timestamp_s = 0;
  while(pg<n_pages){
    DEBUG_PRINT("Reading page "); DEBUG_PRINTLN(pg);
    flash.readByteArray(pg++, 0, bff, PAGESIZE, false);
    cbuffer.CarregarBuffer(bff, 256);//252
    //while(cbuffer.Check(sizeof(altByte) + sizeof(timeByte))){
    while(cbuffer.Check(10)){
      nd = !buffer2serial(doprint);
      if(nd){break;}
    }
    if(nd){break;}
  }
  DEBUG_PRINT(pg); DEBUG_PRINTLN(F("pg"));
}

bool buffer2serial(bool doprint){
  float alt;

  cbuffer.DescarregarBuffer(headerByte, sizeof(headerByte));
  code = headerByte[0]>>5;
  timestamp = (headerByte[0] & B00011111)<<8 | headerByte[1];
  timestamp_s+=((float) (timestamp-timestamp_ant))*0.0001;
  timestamp_ant = timestamp;
  Serial.print(F("t"));
  Serial.print(timestamp_s,3);
  switch(code){
    case 0:
      Serial.println(F(",S0"));
      break;    
    case 1:
      Serial.println(F(",S1"));
      break;
    case 2:
      Serial.println(F(",S2"));
      break;
    case 3:
      Serial.println(F(",S3"));
      break;
    case 5:
      cbuffer.DescarregarBuffer(altByte, sizeof(altByte));
      alt = byte2float(altByte);
      Serial.print(",h");
      Serial.println(alt,3);
      break;
    case 6:
      cbuffer.DescarregarBuffer(gpsByte, sizeof(gpsByte));
      flat = byte2float(gpsByte);
      cbuffer.DescarregarBuffer(gpsByte, sizeof(gpsByte));
      flon = byte2float(gpsByte);
      Serial.print(",lat");
      Serial.print(flat,5);
      Serial.print(",lon");
      Serial.println(flon,5);
      break;
  }

  if(headerByte[0]==255 && headerByte[1]==255){return false;}
  else {return true;}
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
    DEBUG_PRINT(F("WrtPg")); DEBUG_PRINTLN(pg);
    passDataToFlash();
  }
}

void passDataToFlash(){
  cbuffer.DescarregarBuffer(bff,256);//252
  flash.writeByteArray(pg++, 0, bff, PAGESIZE, false);
  if(pg>n_pages){Serial.println(F("MaxData"));} // Aqui es podria parar la grabació
}

void send_xbee(){
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

void switchTask(uint8_t var){
  uint16_t i_pg = 0;
  switch (var) {
    case 1:
        Serial.print(F("4."));
        if(flash.eraseChip()){
          Serial.println(F("1"));
        }else{
          Serial.println(F("0"));
        }
      break;
    case 2:
        Serial.print(F("Erasing chip (intellegently)..."));
        printAllPages(false);
        while(i_pg<=pg){
          flash.eraseSector(i_pg,0);
          i_pg += 16;
        }
        pg = 0;
        Serial.println(F("done!"));
        cbuffer.reset();
      break;
      Serial.println(F("Erase chip success!"));
    case 3:
        Serial.println(F("Serial start/stop!"));
        state = !state;
        cbuffer.reset();
      break;
    case 4:
      printAllPages(true);
      cbuffer.reset();
      break;
    default:
      break;
  }
  send_info();
}
