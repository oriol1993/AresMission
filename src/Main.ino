///////////////////////////////////////////////////////////////////////////////
//                       PROGRAM HEADER
///////////////////////////////////////////////////////////////////////////////

// Libraries
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <Buffer.h>
#include <SPIFlash.h>
//#include <SPI.H>

// Constants and sizes
#define LED 3
#define PB 5
#define force_serial 1
#define bytes_alt 4
#define bytes_timestamp 2
#define baud_rate 115200
#define sample_period 50
#define n_pages 32768
//#define DEBUG

#ifdef DEBUG
 #define DEBUG_PRINT(x)     Serial.print (x)
 #define DEBUG_PRINTLN(x)  Serial.println (x)
#else
 #define DEBUG_PRINT(x)
 #define DEBUG_PRINTLN(x)
#endif

// Declarations
//  Classes
Adafruit_BMP280 bmp;
Buffer cbuffer;
SPIFlash flash;
//  Data storage
byte altByte[bytes_alt], timeByte[bytes_timestamp];
uint16_t timestamp = 0, timestamp_ant = 0;
float timestamp_s = 0, alt;
uint32_t address = 0;
float ground_pressure = 1013.25;
//  Addresses
uint16_t pg = 0;
uint8_t bff[256];
//  Time
uint32_t last_bttn = 0, led_flash = 0, t_lastbmp = 0;
//  status
bool state = false;
uint8_t led_state = 0;
bool serial_init = 0;

uint8_t serialData = 0;
int x = 0;

///////////////////////////////////////////////////////////////////////////////
//                            FUNCTIONS
///////////////////////////////////////////////////////////////////////////////

void setup(){
  // Input and output
  pinMode(LED, OUTPUT);
  pinMode(PB, INPUT_PULLUP);
  if(!digitalRead(PB) | force_serial){
    Serial.begin(baud_rate);
    for(uint8_t i=0;i<7;i++)
    {
      digitalWrite(LED,HIGH);
      delay(100);
      digitalWrite(LED,LOW);
      delay(100);
    }
    serial_init = 1;
  }

  // Flash Initialization
  DEBUG_PRINT(F("Initializing FLASH memory ..."));
  flash.begin();
  delay(500);
  if (!flash.begin()) {
    DEBUG_PRINTLN(F("unable to connect!"));
    //while (1);
  }
  else {DEBUG_PRINTLN(F("connection successful!"));}

  // BMP180 Initialization
DEBUG_PRINT(F("Initializing BMP280 ..."));
  if (!bmp.begin()) {
    DEBUG_PRINTLN(F("unable to connect!"));
    while (1);
  }
  else{DEBUG_PRINTLN(F("connection successful!"));}

  bmp.osrs_p = 15; // 4.3.4 Table 21
  bmp.osrs_t = 1; // 4.3.4 Table 22
  bmp.t_sb = 0; // 3.6.3 Table 11, 4.3.5
  bmp.filter = 5; // 3.3.3
  get_gnd_pressure(ground_pressure);
  Serial.print(F("p @ h=0m = ")); Serial.print(ground_pressure); Serial.println(F(" Pa"));
  cbuffer.reset(); // Buffer Initialization (head = 0; tail = 0)
  info();
}
void info(){
  Serial.println(F("COMMANDS: "));
  Serial.println(F("1.- Erase chip"));
  Serial.println(F("2.- Erase required sectors"));
  Serial.println(F("3.- Manual start/stop"));
  Serial.println(F("4.- Dump data"));
}

void loop(){
  buttonCheck();
  checkLed();
  checkSerial();
  read_bmp();
  buffer2flash();
}

void buttonCheck(){
 if (millis() - last_bttn > 300) {
  if (digitalRead(PB) == 0){
    startstop();
    last_bttn = millis();
  }
 }
}

void checkLed(){
 if (state == true){
  if (millis() - led_flash > 500) {
   led_state = !led_state;
   led_flash = millis();
   digitalWrite(LED, led_state);
  }
 }
 else { digitalWrite(LED, LOW); }
}

void checkSerial() {
  if (Serial.available() > 0) {
    serialData = Serial.parseInt();
    switchTask(serialData);
  }
}

void read_bmp(){

  if(millis()-t_lastbmp>sample_period && state)
  {
    // Acquire heigh measurement and time stamp
    alt = bmp.readAltitude(ground_pressure);
    timestamp = (micros()/100)%65536;

    // Load data into buffer
    float2byte(alt, altByte);  // Convert float to byte array
    timeByte[0] = timestamp & 255;
    timeByte[1] = timestamp>>8;
    if (cbuffer.spaceAvailable(sizeof(altByte) + sizeof(timeByte))) {    //Check if there's enough space for all data
      cbuffer.CarregarBuffer(altByte, sizeof(altByte));
      cbuffer.CarregarBuffer(timeByte, sizeof(timeByte));
    }else{
      DEBUG_PRINTLN("Not enough space in buffer");
    }
    t_lastbmp = millis();

    DEBUG_PRINT("h="); DEBUG_PRINT(alt);
    DEBUG_PRINT(", ts="); DEBUG_PRINT(timestamp);
    // Debugging
    /*
    DEBUG_PRINT(", tb0="); DEBUG_PRINT(timeByte[0]);
    DEBUG_PRINT(", tb1="); DEBUG_PRINT(timeByte[1]);
    DEBUG_PRINT(", al0="); DEBUG_PRINT(altByte[0]);
    DEBUG_PRINT(", al1="); DEBUG_PRINT(altByte[1]);
    DEBUG_PRINT(", al2="); DEBUG_PRINT(altByte[2]);
    DEBUG_PRINT(", al3="); DEBUG_PRINT(altByte[3]);
    */
    DEBUG_PRINTLN();
  }
}

void buffer2flash(){
  if(cbuffer.Check(252)) {
    DEBUG_PRINT(F("Writing page ")); DEBUG_PRINTLN(pg);
    passDataToFlash();
  }
}

bool buffer2serial(){
  cbuffer.DescarregarBuffer(altByte, sizeof(altByte));
  cbuffer.DescarregarBuffer(timeByte, sizeof(timeByte));


  timestamp = timeByte[0] | timeByte[1]<<8;
  timestamp_s+=((float) (timestamp-timestamp_ant))*0.0001;
  timestamp_ant = timestamp;

  alt = byte2float(altByte);

  // Debugging

  DEBUG_PRINT("tb0="); DEBUG_PRINT(timeByte[0]);
  DEBUG_PRINT(", tb1="); DEBUG_PRINT(timeByte[1]);
  DEBUG_PRINT(", al0="); DEBUG_PRINT(altByte[0]);
  DEBUG_PRINT(", al1="); DEBUG_PRINT(altByte[1]);
  DEBUG_PRINT(", al2="); DEBUG_PRINT(altByte[2]);
  DEBUG_PRINT(", al3="); DEBUG_PRINT(altByte[3]);
  DEBUG_PRINTLN();

  Serial.print(alt); Serial.print(",");
  Serial.print(timestamp_s);
  Serial.println();

  if(timeByte[0]==255 && timeByte[1]==255 && altByte[0]==255 && altByte[1]==255 && altByte[2]==255 && altByte[3]==255 ){return false;}
  else {return true;}

}

void float2byte(float val,byte bytes_array[]){
  memcpy(bytes_array, &val, 4);
}

float byte2float(byte bytes_array[]){
  float out_float;
  memcpy(&out_float, bytes_array, 4);   // Assign bytes to input array
  return out_float;
}

void get_gnd_pressure(float &ground_pressure){
  uint8_t n=25;
  ground_pressure = 0;
  for(uint8_t i=0;i<25;i++){
      ground_pressure += bmp.readPressure();
      delay(50);
  }
  ground_pressure = ground_pressure/(float) (n*100);
}

void passDataToFlash(){
  cbuffer.DescarregarBuffer(bff,252);
  flash.writeByteArray(pg++, 0, bff, PAGESIZE, false);
  if(pg>n_pages){Serial.println(F("Max data reached!")); startstop();}
}

void printAllPages() {
  bool nd;
  DEBUG_PRINTLN("Reading all pages");
  cbuffer.reset();
  pg = 0;
  timestamp_s = 0;
  while(pg<n_pages){
    DEBUG_PRINT("Reading page "); DEBUG_PRINTLN(pg);
    flash.readByteArray(pg++, 0, bff, PAGESIZE, false);
    cbuffer.CarregarBuffer(bff, 252);
    while(cbuffer.Check(sizeof(altByte) + sizeof(timeByte))){
      nd = !buffer2serial();
      if(nd){break;}
    }
    if(nd){break;}
  }
  DEBUG_PRINT(pg); DEBUG_PRINTLN(F(" pages found"));
}

void switchTask(uint8_t var){
  uint16_t i_pg = 0;
  switch (var) {
    case 1:
        Serial.println(F("Erasing chip..."));
        if(flash.eraseChip()){

        }else{
          Serial.println(F("Erase chip failed!"));
        }
        info();
      break;
    case 2:
        Serial.print(F("Erasing chip (intellegently)..."));
        printAllPages();
        while(i_pg<=pg){
          flash.eraseSector(i_pg,0);
          i_pg += 16;
        }
        pg = 0;
        info();
      break;
      Serial.println(F("Erase chip success!"));
    case 3:
        Serial.println(F("Serial start/stop!"));
        startstop();
      break;
    case 4:
      printAllPages();
      info();
      break;
    default:
      break;
  }

}

void startstop(){
  led_flash = millis();
  led_state = 1;
  state = !state;
  if(state){
    cbuffer.reset();
    pg = 0;
    Serial.println(F("Recording started!"));
  }else{
    Serial.println(F("Recording finished!"));
    Serial.print(pg-1); Serial.println(F(" pages written"));
    cbuffer.reset();
    info();
  }
}
