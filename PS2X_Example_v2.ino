/*
 * Testing the Ps2 controler
 * Controler Lynxmotion Ps2 Controler V. 4 with breakout board
 * Go to www.billporter.info for updates and to report bugs.
 * V1:
 *  - found the right setting for my controler
 * V2:
 *  - implemented controler setup in main loop in case contact is lost
 *  - removed unnecessary code (Guitar Hero, etc.), but not from library
 */
#ifdef COMPILE_PS2EXAMPLE

//definitions for Ps2 controler for teleoperation
#include <PS2X_lib.h>  //revised library from KurtE from Github
PS2X ps2x; // create PS2 Controller Class

// SPI bus pins on Arduino Due ICSP connector near SAM3X8E chip
#define SPI_MISO 74
#define SPI_MOSI 75
#define SPI_CLK  76
#define PS2X_CS  49 // chip select for PS2X controler

int PS2_config_result{254}; // controler never set = 254
byte PS2_type{0};
//byte PS2_vibrate_level{0}; // no vibration

//***************** PS2 controler setting
void set_ps2x(void) {
 digitalWrite(PS2X_CS, LOW); // select controler for action
 
 PS2_config_result = ps2x.config_gamepad(SPI_CLK, SPI_MOSI, PS2X_CS, SPI_MISO, true, true);   //setup pins and settings:  GamePad(clock, command, attention, data, Pressures?, Rumble?) check for PS2_config_result
 
 if(PS2_config_result == 0){
   Serial.println("Found Controller, configured successful");
 }
   
  else if(PS2_config_result == 1)
   Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");
   
  else if(PS2_config_result == 2)
   Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");
   
  else if(PS2_config_result == 3)
   Serial.println("Controller refusing to enter Pressures mode, may not support it. ");
   
   //debug(ps2x.Analog(1), HEX);
   
   PS2_type = ps2x.readType(); 
     switch(PS2_type) {
       case 0:
        Serial.println("Unknown Controller PS2_type");
       break;
       case 1:
        Serial.println("DualShock Controller Found");
       break;
     }
}

void setup() {
 Serial.begin(115200);
 pinMode(PS2X_CS, OUTPUT); //PS2 controler chip select pin
}

void loop(){
   /* You must Read Gamepad to get new values
   Read GamePad and set vibration values
   ps2x.read_gamepad(small motor on/off, larger motor strenght from 0-255)
   if you don't enable the rumble, use ps2x.read_gamepad(); with no values
   
   you should call this at least once a second
   */
   
 // make sure controler is active
 //debug if(PS2_config_result == 1) //skip loop if no controller found
 //debug  return; 
 
 if(PS2_config_result != 0) {//try to setup controler p to 10 times
  Serial.println("Setting controler...");
  byte i;
  for (i=0; i<10; i++) {
   set_ps2x();
   if (PS2_config_result == 0) break;
   delay(50);
  }
 }
  
 if (PS2_config_result==0 && PS2_type==1) { //DualShock Controller
  
    //ps2x.read_gamepad(false, PS2_vibrate_level);          //read controller and set large motor to spin at 'PS2_vibrate_level' speed
    ps2x.read_gamepad();
    
    if(ps2x.Button(PSB_START))                   //will be TRUE as long as button is pressed
         Serial.println("Start is being held");
    
    if(ps2x.Button(PSB_SELECT))
         Serial.println("Select is being held");
         
         
    if(ps2x.Button(PSB_PAD_UP)) {         //will be TRUE as long as button is pressed
      debug("Up held this hard: ");
      debugln__2arg(ps2x.Analog(PSAB_PAD_UP), DEC);
    }
    
    if(ps2x.Button(PSB_PAD_RIGHT)){
       debug("Right held this hard: ");
       debugln__2arg(ps2x.Analog(PSAB_PAD_RIGHT), DEC);
    }
    
    if(ps2x.Button(PSB_PAD_LEFT)){
      debug("LEFT held this hard: ");
      debugln__2arg(ps2x.Analog(PSAB_PAD_LEFT), DEC);
    }
    
    if(ps2x.Button(PSB_PAD_DOWN)){
      debug("DOWN held this hard: ");
      debugln__2arg(ps2x.Analog(PSAB_PAD_DOWN), DEC);
    }   
    
    //PS2_vibrate_level = ps2x.Analog(PSAB_BLUE);        //this will set the large motor PS2_vibrate_level speed based on 
                                              //how hard you press the blue (X) button    
    
    if (ps2x.NewButtonState())               //will be TRUE if any button changes state (on to off, or off to on)
    {
     
     //if(ps2x.Button(PSB_L3)) no L3 on my PS2
      //Serial.println("L3 pressed");
     //if(ps2x.Button(PSB_R3))  no R3 on my PS2
      //Serial.println("R3 pressed");
     if(ps2x.Button(PSB_L2))
      Serial.println("L2 pressed");
     
     if(ps2x.Button(PSB_R2))
      Serial.println("R2 pressed");
     
     if(ps2x.Button(PSB_GREEN))
      Serial.println("Triangle pressed");
    }   
         
    
    if(ps2x.ButtonPressed(PSB_RED))             //will be TRUE if button was JUST pressed
         Serial.println("Circle just pressed");
         
    if(ps2x.ButtonReleased(PSB_PINK))             //will be TRUE if button was JUST released
         Serial.println("Square just released");     
    
    if(ps2x.NewButtonState(PSB_BLUE))            //will be TRUE if button was JUST pressed OR released
         Serial.println("X just changed");    
    
    if(ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1)) // print stick values if either is TRUE
    {
        debug("Stick Values:");
        debug(ps2x.Analog(PSS_LX), DEC); //Left stick, Y axis. Other options: LX, RY, RX  
        debug(",");
        debugln__2arg(ps2x.Analog(PSS_LY), DEC); 
        //debug(",");
        //debug(ps2x.Analog(PSS_RY), DEC); 
        //debug(",");
        //debugln__2arg(ps2x.Analog(PSS_RX), DEC); 
    } 
 }
 delay(50);
}
#endif
