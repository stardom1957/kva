/*
 * Measuring the speed and direction of two motors DC using an L298N motor bridge.
 * Hall encoders are used as sensors
 * 
 * This test makes use of two Robokax Infrared Obstacle Avoidance module placed on each side
 * off ONE MOTOR drive wheel (the emiter of one sensor directed toward the receptor of the other) in order
 * to get a direct reading for that wheel. The drive wheel has tree spokes thus
 * gives tree readings per turn.
 * 
 * (motors_measure_) Version 2. Op mode is MEASURE_AND_CALIBRATE_MOTORS, adds this:
 * - DueTimer from Ivan Seidel is used instead of Arduino millis() function
 * - motors will run continuouly.
 * - results are taken every second
 * 
 * (motors_measure_) Version 3. Op mode is MEASURE_AND_CALIBRATE_MOTORS, adds this:
 * - the function ISR_timer1 will do all the work. Grab the counter values and put them into a table.
 * - the program will count during 1 s and there will be NBR_OF_SAMPLINGS
 * - the results will be displayed on the serial monitor at the end of the run 
 * as comma delimited numbers. The result can be copied to a spreadsheet for analysis,
 * 
 * *** (motors_measure_) see results in /home/dominique/Arduino/Documentation/results/2.8 - measuring motors
 
 * 
 * KVA V1: first integration of Ps2 controler to replace joystick in TELEOP mode. In this version,
 *  - function motor_TELEOP_node_v1(): 
 *    - will handle an on the spot rotation by setting one motor to full speed and the opposite one to 0,
 *      wich meens that full X left or full X right stick value while Y stick is in at rest will produce no movement.
 */

// ************** Compile directives
#define COMPILE_MAIN
//#define DUE_TIMER // see DueTimer tab
//************************************************

#ifdef COMPILE_MAIN
#include <DueTimer.h>

// robot vehicule modes of operation see vehiculeModes.txt
#define JUST_RUN 30                     // just run
#define MEASURE_AND_CALIBRATE_MOTORS 40  //used to test what ever needs testing
#define TELEOP 10                       //Teleoperation with a joystick // TELEOP: Joystick operation
#define NADDOCAM 20                     //NADDOCAM: Non Directed Autonomous Driving with Obstacle Collision Avoidance mode:
/* NADDOCAM: Non Directed Autonomous Driving with Obstacle Collision Avoidance mode: 
 *  the vehicule navigates freely, begining driving forward from a start point while avoiding collisions.
*/
byte opMode; //operation mode

#define NBR_OF_SAMPLINGS 70         // nbr max of sampling
#define SAMPLING_INTERVAL 1000000 // sampling interval = timer interval set in microseconds (10e-6s)

//************* measuring and motor control definitions
// left motor on L298N channel A
#define ENA_L 3 // pwm pin
#define IN1   29
#define IN2   27

// right motor on L298N channel B
#define ENB_R 2 // pwm pin
#define IN3   25
#define IN4   23

volatile unsigned long result_table[3][NBR_OF_SAMPLINGS]; // result table
volatile short int mcount{0}; // counter for the reading

volatile unsigned long S1_L_count {0}; // running count for Hall sensor S1, left motor
volatile unsigned long S1_R_count {0}; // running count for Hall sensor S1, right motor

volatile unsigned long S1_L_count_previous {0}; // previous count for Hall sensor S1, left motor
volatile unsigned long S1_R_count_previous {0}; // previous count for Hall sensor S1, right motor

const byte S1motorEncoder_L_PIN = 22;  // motor encoder S1 A pin
const byte S2motorEncoder_L_PIN = 24;  // motor encoder S2 B pin 
const byte S1motorEncoder_R_PIN = 26;  // motor encoder S1 pin
const byte S2motorEncoder_R_PIN = 28;  // motor encoder S2 pin

DueTimer myTimer = Timer.getAvailable();

//*************************************************************
//definitions for Ps2 controler for teleoperation
#include <PS2X_lib.h>  //revised library from KurtE from Github
PS2X ps2x; // create PS2 Controller Class object

// SPI bus pins on Arduino Due ICSP connector near SAM3X8E chip
#define SPI_MISO 74
#define SPI_MOSI 75
#define SPI_CLK  76
#define PS2X_CS  49 // chip select for PS2X controler

int PS2_config_result{254}; // controler never set = 254
byte PS2_type{0};
//byte PS2_vibrate_level{0}; // no vibration



//**************** FUNCTIONS DEFINITIONS
void ISR_timer1(void) {
     noInterrupts(); //stop all interrupts

     //S1_L_count for timer period = S1_L_count - S1_L_count_previous;
     result_table[0][mcount] = S1_L_count - S1_L_count_previous;
     S1_L_count_previous = S1_L_count;

     //S1_R_count for timer period = S1_R_count - S1_R_count_previous;
     result_table[1][mcount] = S1_R_count - S1_R_count_previous;
     S1_R_count_previous = S1_R_count;

     ++mcount; // number of samplings
     interrupts(); //restart all interupts

/*   attachInterrupt(S1motorEncoder_L_PIN, ISR_S1_L, CHANGE);
     attachInterrupt(S1motorEncoder_R_PIN, ISR_S1_R, CHANGE);
*/
}

// ISR for the sensor counters
void ISR_S1_L(void) {
  ++S1_L_count;
}
void ISR_S1_R(void) {
  ++S1_R_count;
}

// Motor Speed Values - Start at zero

int motorSpeed_L = 0;
int motorSpeed_R = 0;
#define MOTOR_LOWER_PWM_LIMIT 25 // to avoid buzzing

// this set of motor functions are use for motor control in all
// the modes that require motor control by software.

// motor direction
const byte FORWARD {0};
const byte REVERSE {1};

void motorRightSet(int speed, byte direction) {
 switch (direction) {
  case FORWARD:
   // moteur droit avance
   if (speed >= MOTOR_LOWER_PWM_LIMIT && speed <= 255) {
     digitalWrite(IN3, LOW);
     digitalWrite(IN4, HIGH);
     analogWrite(ENB_R, speed);
   }
   break;

  case REVERSE:
   // moteur droit recule
   if (speed >= MOTOR_LOWER_PWM_LIMIT && speed <= 255) {
     digitalWrite(IN3, HIGH);
     digitalWrite(IN4, LOW);
     analogWrite(ENB_R, speed);
   }
   break;
 }
}

void motorLeftSet(int speed, byte direction) {
 switch (direction) {
  case FORWARD:
   // moteur droit avance
   if (speed >= MOTOR_LOWER_PWM_LIMIT && speed <= 255) {
     digitalWrite(IN1, LOW);
     digitalWrite(IN2, HIGH);
     analogWrite(ENA_L, speed);
   }
   break;

  case REVERSE:
   // moteur droit recule
   if (speed >= MOTOR_LOWER_PWM_LIMIT && speed <= 255) {
     digitalWrite(IN1, HIGH);
     digitalWrite(IN2, LOW);
     analogWrite(ENA_L, speed);
   }
   break;
 }
}

void motorRightForward(int speed) {
  // moteur droit avance
  if (speed >= 100 && speed <= 255) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB_R, speed);
  }
}

void motorRightReverse(int speed) {
  if (speed >= 100 && speed <= 255) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB_R, speed);
  }
}

void motorRightStop(void) {
    digitalWrite(ENB_R, LOW);
}

void motorLeftForward(int speed) {
  if (speed >= 100 && speed <= 255) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA_L, speed);
  }
}

void motorLeftReverse(int speed) {
  if (speed >= 100 && speed <= 255) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA_L, speed);
  }
}

void motorLeftStop(void) {
    digitalWrite(ENA_L, LOW);
}

void motorAllStop(void)
{
  motorLeftStop();
  motorRightStop();
}

/* this function implement the TELEOP mode: the vehicule movement is fully controled by a remote operator using a Ps2 type controler. 
 *  This mode doesn't provide for any type of collision avoidance.

*/

//***************** PS2 controler setting function
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
   
   //Serial.print(ps2x.Analog(1), HEX);
   
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



//********************************************
//***************** TELEOP *******************

void motor_TELEOP_node_v1(void) {
  byte leftMotorDirection;
  byte rightMotorDirection;
  int ps2LY;
  int ps2LX;  
  const int atRestZone {12};  // buffer zone to indicate stick is at rest in the middle
  
  if(PS2_config_result == 254) { //fisrt time in, try to setup controler up to 3 times
   // debug Serial.println("Setting controler...");
   byte sc;
   for (sc=0; sc<3; sc++) {
     set_ps2x();
     if (PS2_config_result == 0) break;
     delay(50);
   }
  }

  // if no controler was found on last run, then ensure we will attempt to setup controler in next run
  if (PS2_config_result != 0 || PS2_type != 1) {
    PS2_config_result == 254;
    return;
  }
 
  //ps2x.read_gamepad(false, PS2_vibrate_level);          //read controller and set large motor to spin at 'PS2_vibrate_level' speed
  ps2x.read_gamepad();

  //****************** left stick will be read only if button PSB_L1 is held pressed
  // dead man grip
  if(ps2x.Button(PSB_L1)) { // we are using the left stick
    ps2LX = ps2x.Analog(PSS_LX); //Raw left stick X axis values are from 0 (full left) to 255 (full right), 128 is at rest in middle
    ps2LY = ps2x.Analog(PSS_LY); //Raw left stick Y axis values are from 0 (full up) to 255 (full down), 127 is at rest in middle

/* debug
    Serial.print(ps2LX, DEC); //Left stick, Y axis. Other options: LX, RY, RX  
    Serial.print(",");
    Serial.println(ps2LY, DEC); 
*/
   // PSS_LY determines if this is a forward or reverse motion
   // FORWARD is PSS_LY <= (127 - atRestZone)
   // REVERSE is PSS_LY >= (127 + atRestZone)
   
   // Do this by reading the Verticle Value Y
   // Apply results to MotorSpeed and to Direction

   if (ps2LY >= (127 + atRestZone))
   {
     // This is reverse
     leftMotorDirection = REVERSE;
     rightMotorDirection = REVERSE;

     //motor speeds is determined from stick values ps2LY
     // we need to map the reading from 0 to 255

     motorSpeed_L = map(ps2LY, (127 + atRestZone), 255, 0, 255);
     motorSpeed_R = map(ps2LY, (127 + atRestZone), 255, 0, 255);
  }
  
  else if (ps2LY <= (127 - atRestZone))
  {
    // This is Forward
     leftMotorDirection = FORWARD;
     rightMotorDirection = FORWARD;

    //motor speeds is determined from stick values ps2LY
    //we need to map reading from 0 to 255

    motorSpeed_L = map(ps2LY, (127 - atRestZone), 0, 0, 255);
    motorSpeed_R = map(ps2LY, (127 - atRestZone), 0, 0, 255); 

  }
  else
  {
    // the stick is in the middle rest zone, so this is Stopped

    motorSpeed_L = 0;
    motorSpeed_R = 0; 

  }
  
  // Now do the steering
  // The Horizontal position X will "weigh" the motor speed
  // Values for each motor

   // PSS_LX determines the steering direction
   // LEFT  is PSS_LX <= (128 - atRestZone)
   // RIGHT is PSS_LX >= (128 + atRestZone)

  if (ps2LX <= (128 - atRestZone)) {
    // We move to the left
    // Map the number to a value of 255 maximum

    ps2LX = map(ps2LX, 0, (128 - atRestZone), 255, 0);
        

    motorSpeed_L = motorSpeed_L - ps2LX;
    motorSpeed_R = motorSpeed_R + ps2LX;

    // Don't exceed range of 0-255 for motor speeds

    if (motorSpeed_L < 0) motorSpeed_L = 0;
    if (motorSpeed_R > 255) motorSpeed_R = 255;
  }
    else if (ps2LX >= (128 + atRestZone)) {
    // we move to the right
    // Map the number to a value of 255 maximum

      ps2LX = map(ps2LX, (128 + atRestZone), 255, 0, 255);
      motorSpeed_L = motorSpeed_L + ps2LX;
      motorSpeed_R = motorSpeed_R - ps2LX;

    // Don't exceed range of 0-255 for motor speeds

      if (motorSpeed_L > 255) motorSpeed_L = 255;
      if (motorSpeed_R < 0) motorSpeed_R = 0;      
    }

  // Adjust to prevent "buzzing" at very low speed

  if (motorSpeed_L < MOTOR_LOWER_PWM_LIMIT) motorSpeed_L = 0;
  if (motorSpeed_R < MOTOR_LOWER_PWM_LIMIT) motorSpeed_R = 0;

  // get the motors going
  motorLeftSet(motorSpeed_L, leftMotorDirection);
  motorRightSet(motorSpeed_R, rightMotorDirection);

  }
    else { // if PSB_L1 not pressed, then we make sure motors are stopped
      motorAllStop();
    }
  delay(50);
} // fin motor_TELEOP_node_v1

void just_run() {
  Serial.println("Both motors FORWARD for 4s");
  motorLeftSet(200, FORWARD);
  motorRightSet(200, FORWARD);
  delay(4000);
  motorAllStop();
  Serial.println("Both motors STOPTED for 3s");
  delay(3000);
  Serial.println("Both motors REVERSE for 4s");
  motorLeftSet(200, REVERSE);
  motorRightSet(200, REVERSE);
  delay(4000);

  Serial.println("Left motor FORWARD and Right motor REVERSE for 4s");
  motorAllStop();
  motorLeftSet(200, FORWARD);
  motorRightSet(200, REVERSE);
  delay(4000);

  Serial.println("Left motor REVERSE and Right motor FORWARD for 4s");
  motorAllStop();
  motorLeftSet(200, REVERSE);
  motorRightSet(200, FORWARD);
  delay(4000);
     
  Serial.println("All stop for 2s");
  motorAllStop();
  delay(2000);
  Serial.println("_________");

  //while(true); //we're only doing this once
}

void setup()
{
  //debug we might use this later
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  Serial.begin(115200);

  // set all the motor control pins to outputs
  pinMode(ENA_L, OUTPUT);
  pinMode(ENB_R, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // set all encoder sensors to INPUT
  pinMode(S1motorEncoder_L_PIN,INPUT);
  pinMode(S2motorEncoder_L_PIN,INPUT);
  pinMode(S1motorEncoder_R_PIN,INPUT);
  pinMode(S2motorEncoder_R_PIN,INPUT);

  //timer interrupt settings
  myTimer.setPeriod(SAMPLING_INTERVAL); // in microseconds
  myTimer.attachInterrupt(ISR_timer1);

  motorAllStop();

  pinMode(PS2X_CS, OUTPUT); //PS2 controler chip select pin

//  debug noInterrupts(); //no interrupts for now
  attachInterrupt(S1motorEncoder_L_PIN, ISR_S1_L, CHANGE);
  attachInterrupt(S1motorEncoder_R_PIN, ISR_S1_R, CHANGE);
// debug interrupts(); // start counting motor rotation
  myTimer.start();
}

void loop()
{
 //****************************************************************
 //***************** get new opMode value from command channel ****
 //****************************************************************
 // for now hardcode the opMode

 //opMode = TELEOP;
 //opMode = MEASURE_AND_CALIBRATE_MOTORS;
 opMode = JUST_RUN;

  switch (opMode) {
    case JUST_RUN:
     just_run();
     break;

    case TELEOP: // teleoperation by remote
     motor_TELEOP_node_v1();
     break;

    //**********************************************
    //***************** NADDOCAM *******************
    case NADDOCAM:
     break;

    //************************************************
    //***************** MEASURE_AND_CALIBRATE_MOTORS *
    case MEASURE_AND_CALIBRATE_MOTORS:
     // the measures will start here
     motorRightForward(255);
     motorLeftForward(255);
     delay(2000); // wait 2 s to stabilize motors

     interrupts();
     myTimer.start();

     while(mcount < NBR_OF_SAMPLINGS){ // wait for NBR_OF_SAMPLINGS readings to occur
     }
     
     // display results on Serial Monitor as comma delimited values
     // for future data analysis on spreadsheet
     // format left, right, ir raw value
     
     for (short int r=0; r < NBR_OF_SAMPLINGS; ++r) {
      for (short int v=0; v < 3; ++v) {
        Serial.print(result_table[v][r], DEC);
        if (v!=2) Serial.print(",");
       }
       Serial.println();
     }

     myTimer.stop();
     motorAllStop();
     detachInterrupt(S1motorEncoder_L_PIN);
     detachInterrupt(S1motorEncoder_R_PIN);

     while(true){
      ; // we do this only once
     }
     break;
    
    default:
     ; //do nothing for now
  }
}
//endif for COMPILE
#endif

