#define pushbutton 5
#define pb_dtshort 150
#define pb_dtlong 3000
#define blueled_blinkshort 100
#define blueled_blinklong 1000
#define led_blue 3
#define led_red 4
#define led_green 6 // Raro


int state=0;
int pb_ispressed=0;
int blink_state=0;
int led_blue_state = 0;
int led_blue_n = 0;
uint32_t pb_ton;
uint32_t led_blue_tchange = 0;
uint32_t blueled_lastchange = 0;

void setup()
{
Serial.begin(9600);
Serial.println("REBOOT");
pinMode(led_blue,OUTPUT);
pinMode(led_red,OUTPUT);
pinMode(led_green,OUTPUT);
pinMode(pushbutton,INPUT_PULLUP);
}

void loop()
{
  read_buttn();
  //updt_barom();
  //updt_gps();
  //igni_stage();
  //igni_parac();
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
      Serial.println("LongPress");
      pb_ton = millis();
      //erase_flash();
    }else if(dt>pb_dtshort){
      pb_ispressed = false;
      state = !state;
      pb_ton = millis();
      Serial.print("ShortPress: State = "); Serial.println(state);

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

 // uint32_t dt = millis()-tbarom_last;
 // if(dt>barom_period && !flag_barom){
 //   tbarom_last = millis();
 //   flag_barom = true;
    // measure
//  }
//}
void updt_gps(){
  //inputs: tgps_last
  //outputs: tgps_last, flag_gps, value_gps
}
void igni_stage(){
  //inputs: tignite, value_barom
  //outputs: tstage, flag_stage
}
void igni_parac(){
  //inputs: value_barom, value_baromax
  //outputs: tparac, flag_parac
}
void rset_flash(){
  //inputs: erase_flash
  //outputs:
}
void writ_flash(){
  //inputs: taccel_last, flag_accel, value_accel, tbarom_last, flag_barom, value_barom, tstage, flag_stage, tparac, flag_parac
  //outputs:
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
    else if(led_blue_n==state*2){
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
