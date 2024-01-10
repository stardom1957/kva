/*History, see kva_history.h
 */

// ************** Compile directives
#define COMPILE_MAIN

#ifdef COMPILE_MAIN
#define RTC_COMPILE
#define MOTOR_CONTROL_COMPILE
#define SENSORS_COMPILE
#define SERIAL_DEBUG_PORT     0 // serial port for debuging
#define SERIAL_TELEMETRY_PORT 1 // serial port to XBee RF module
#define SERIAL_HMI            2 // serial port to Nextion HMI
#define TELEMETRY               // compile telemetry code
#define PID_COMPILE // compile kva_pid.h

// definition of debug levels
// mainly replacing Serial.print and Serial.println by nothing when not needed
// that is DEBUG == 0

#define DEBUG 0 // 1 means debugging is active 0 means it's not
#if DEBUG == 1
#define debug(x) Serial.print(x)
#define debug_2arg(x, y) Serial.print(x, y)
#define debugln(x) Serial.println(x)
#define debugln_2arg(x, y) Serial.println(x, y)

#else
#define debug(x)
#define debug_2arg(x, y)
#define debugln(x)
#define debugln_2arg(x, y)
#endif

#include <PS2X_lib.h>  //for teleoperation with PS2 style remote. Revised KurtE library (Github)

#ifdef TELEMETRY
#include "telemetry.h"
#endif

#include "kva.h"
#include "sensors.h"
#include "motor_control.h"
#include "kva_rtc.h"
#include "kva_hmi.h" // for HMI display and control
#include "kva_pid.h"

//************** OPMODE ***************************
//*********** EMERGENCY_MODE **********************

volatile unsigned long entry_time{0};
bool do_emergency_reverse{false};
unsigned long emergency_run_time;
long turnRightOrLeft{false};

void handle_emergency(byte calling_op_mode) {
  entry_time = micros();
  switch(calling_op_mode) {
    case FREE_RUN:
      if (contact_sensor_just_triggered) {
        contact_sensor_just_triggered = false;
        emergency_run_time = 0;
        do_emergency_reverse = true;
        // initialise random direction for turn
        // we want to turn in one direction if even and in the opposite direction if odd
        turnRightOrLeft = random(RANDOM_RANGE) % 2; // 0 if even (false), 1 if odd (true)
      }
      
      // handle emergency reverse
      // reverse is allways done the same way
      
      if ( do_emergency_reverse ) { 
	      if (emergency_run_time < EMERGENCY_REVERSE_RUN_TIME) {
          motorLeftSet(EMERGENCY_SLOW, BACKWARD);
          motorRightSet(EMERGENCY_SLOW, BACKWARD);
	      }
	      else {
          digitalWrite(ENB_R, LOW); //this will stop both motors
          digitalWrite(ENA_L_PIN, LOW);
          do_emergency_reverse = false;
	      }
      }

      // update emergency run time
      emergency_run_time += (micros() - entry_time);
      
      // handle emergency turn after EMERGENCY_WAIT_TIME has passed
      if ((EMERGENCY_REVERSE_RUN_TIME + EMERGENCY_WAIT_TIME < emergency_run_time) &&
       (emergency_run_time < EMERGENCY_REVERSE_RUN_TIME + EMERGENCY_TURN_RUN_TIME + EMERGENCY_WAIT_TIME) ) {
        // turn direction is dependant of wich contact sensor is triggered
        switch(contact_sensor_just_triggered_ID) {
          case RIGHT_CONTACT_TRIGGERED:
            // turn left
            vehiculeRotateLeft(EMERGENCY_SLOW);
            break;

          case CENTER_CONTACT_TRIGGERED:
            // turn randomly left or right
            if (turnRightOrLeft) { vehiculeRotateLeft(EMERGENCY_SLOW); }
            else { vehiculeRotateRight(EMERGENCY_SLOW); }
            break;

          case LEFT_CONTACT_TRIGGERED:
            // turn right
            vehiculeRotateRight(EMERGENCY_SLOW);
            break;

        }
      }
      else { // going out of emergency mode
        digitalWrite(ENB_R, LOW); //this will stop both motors
        digitalWrite(ENA_L_PIN, LOW);
        // TODO reenable all contact sensors interrupts ???
        // restore old opmode
        currentOpMode = calling_op_mode;
        // reset contact_sensor_triggered_ID
        contact_sensor_just_triggered_ID = 0;
       } // else emergency turn
      
      // update emergency run time
      emergency_run_time += (micros() - entry_time);
    
    break; // FREE_RUN

    case TELEOP:
    break;
  }
}

//************** OPMODE ***************************
//*********** RUN_PRESET_COURSE *******************

void run_preset_course(void) {
  Serial.println("Both motors FORWARD for 4s");
  motorLeftSet(200, FORWARD);
  motorRightSet(200, FORWARD);
  delay(4000);
  motorAllStop();
  Serial.println("Both motors STOPED for 3s");
  delay(3000);
  Serial.println("Both motors BACKWARD for 4s");
  motorLeftSet(200, BACKWARD);
  motorRightSet(200, BACKWARD);
  delay(4000);

  Serial.println("Left motor FORWARD and Right motor BACKWARD for 4s");
  motorAllStop();
  motorLeftSet(200, FORWARD);
  motorRightSet(200, BACKWARD);
  delay(4000);

  Serial.println("Left motor BACKWARD and Right motor FORWARD for 4s");
  motorAllStop();
  motorLeftSet(200, BACKWARD);
  motorRightSet(200, FORWARD);
  delay(4000);
     
  Serial.println("All stop for 2s");
  motorAllStop();
  delay(2000);
  Serial.println("___ wait 5 s______");

  //while(true); //we're only doing this once
  delay(5000);
}

//************ OPMODE ************************
//*********** STANDBY ***********************

void standby(void) {
  // #TODO
  if (!continue_run) {
     motorAllStop();  
  }
  continue_run = true;
}

//************ OPMODE ************************
//************ FREE_RUN ***********************
// for motor pid
#define FREE_RUN_TARGET_LEFT 240  //initial sPeed in encoder counts
#define FREE_RUN_TARGET_RIGHT 240  //initial sPeed in encoder counts

// actual speed values during pid control
int l_speed{0};
int r_speed{0};

void free_run(void) {
  if (!contact_sensor_just_triggered) {
    if (!continue_run) {
      // setup
      l_speed = FREE_RUN_TARGET_LEFT;
      r_speed = FREE_RUN_TARGET_RIGHT;
    }

    // we would do pid here
    //l_speed = target_L;
    //r_speed = target_R;

    motorLeftSet(l_speed, FORWARD);
    motorRightSet(r_speed, FORWARD);  
    continue_run = true;

  }
}

/*
void free_run(void) {
 // #TODO
  # int target_L = EMERGENCY_SLOW;  //in encoder counts
  # int target_R = EMERGENCY_SLOW;
     
  motorLeftSet(target_L, FORWARD);
  motorRightSet(target_R, FORWARD);

  // do PID
  // ***********************
  // time difference
  long currTime = micros();
  float deltaT = ((float) (currTime - prevTime))/( 1.0e6 );
  prevTime = currTime;

  // Read the S1 encoders positions
  int pos_actual_L;
  int pos_actual_R;
  noInterrupts(); // disable interrupts temporarily while reading
  pos_actual_L = posi_L;
  pos_actual_R = posi_R;
  interrupts(); // turn interrupts back on

  // evaluate PID for each motor
  int pwrL, dirL, pwrR, dirR;
  // evaluate the control signal for left and right motor
  pid_L.evalu(pos_actual_L, target_L, deltaT, pwrL, dirL);
  pid_R.evalu(pos_actual_R, target_R, deltaT, pwrR, dirR);

  // signal the motor
  //Serial.print("pwr: ");
  Serial.println(pwrL);
  Serial.println(pwrR);
  //Serial.print(", dir: ");
  Serial.println(dirL);
  Serial.println(dirR);

  motorLeftSet(pwrL, dirL);
  motorRightSet(pwrR, dirR);
}
*/

// runs the selected opMode
void runOpMode(byte om) {
  switch (om) {
    case EMERGENCY_MODE:
     handle_emergency(oldOpMode);
     break;
     
    case STANDBY:
     currentOpModeName = "STANDBY";
     motorAllStop();
     standby();
     break;

    case SENSORS_DEVELOPEMENT:
     currentOpModeName = "SENS. DEVEL.";
     sensorDeveloppement();
     break;

    case RUN_PRESET_COURSE:
     currentOpModeName = "RUN_PRESET_COURSE";
     run_preset_course();
     break;

    case TELEOP: // teleoperation by remote
     currentOpModeName = "TELEOP";
     motor_TELEOP_node_v1();
     break;

    case FREE_RUN:
     currentOpModeName = "FREE RUN";
     free_run();
     break;
    
    default:
     standby(); //safest thing to do
  }  
}
// put part of a string into char char_buffer
void strToChar(String s) {
  memset(char_buffer, 0, sizeof(char_buffer));
  for (byte i=0; i < s.length(); i++) {
    char_buffer[i] = s.charAt(i);
  }
}


void setGPIOs(void) {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // set all the motor control pins to outputs
  pinMode(ENA_L_PIN, OUTPUT);
  pinMode(ENB_R, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // set all encoder sensors to INPUT
  pinMode(S1motorEncoder_L_PIN,INPUT);
  pinMode(S2motorEncoder_L_PIN,INPUT);
  pinMode(S1motorEncoder_R_PIN,INPUT);
  pinMode(S2motorEncoder_R_PIN,INPUT);

 // indicators, controls and displays
  pinMode(LED_YELLOW_ALERT_CONDITION, OUTPUT);
  pinMode(LED_GREEN_SYSTEM_READY, OUTPUT);
  pinMode(PS2X_CS, OUTPUT); //PS2 controler chip select pin
  digitalWrite(LED_GREEN_SYSTEM_READY, LOW);  // system not ready
  digitalWrite(LED_YELLOW_ALERT_CONDITION, LOW); // yellow LED will indicate start of setup

  // contact sensors
  pinMode(RIGHT_CONTACT_SENSOR_PIN, INPUT);
}

// manages change of opMode
void manageOpModeChange(void) {
  if (opModeChangeRequested && opModeChangeAutorized) {
   debug("in manageOpModeChange");
   //dbSerialPrintln("  ");
   currentOpMode = requestedOpMode;
   continue_run = false; // ensures that mode setup code is run during first function entry of current opmode

   debugln("debug requestedOpMode, value is: ");
   debugln(requestedOpMode);
   
   opModeChangeRequested = false;
   opModeChangeAutorized = false;
   //effect opmode change
   //runOpMode(currentOpMode);
   currentHMIpage = 1;
   page1.show(); //show opmode change page
   setOpmodeButtonColors();
  }
}

void updateDisplayAndIndicators(void) {
 if ((millis() - displayTimer) > displayInterval) {
     //yellow LED on if any adverse condition occur
     digitalWrite(LED_YELLOW_ALERT_CONDITION, LOW); // reset alert condition

     //check RTC status
     if (rtc.isrunning() == 0) {
      digitalWrite(LED_YELLOW_ALERT_CONDITION, HIGH);
      // try to initialize RTC
      rtcFound = kva_rtc_init();
     }

     //check HMI
     if (!hmiFound) {
      digitalWrite(LED_YELLOW_ALERT_CONDITION, HIGH);
     }
    

   // update LED_YELLOW_ALERT_CONDITION on vehicule
   #ifdef TELEMETRY
   if (currentTopic == maxNbrTopics) {
     digitalWrite(LED_YELLOW_ALERT_CONDITION, HIGH);
   } else {digitalWrite(LED_YELLOW_ALERT_CONDITION, LOW);}
   #endif

   // ###########################################################
   // HMI control
   // display status according to currently selected status page
   switch (currentHMIpage) {
    case 0:
      // this page 0; status page
      // display topicsOverflow **************
      //debug
      #ifdef TELEMETRY
      topicsOverflow = random(100); //debug
      if (topicsOverflow > 70) {
       jbuffstat.Set_font_color_pco(RED);
      }
      else {
        jbuffstat.Set_font_color_pco(GREEN); // green
      }
     
      if (topicsOverflow > 100) { topicsOverflow = 100; }
      jbuffstat.setValue(topicsOverflow);
      #endif //end ifdef TELEMETRY
     
     // update current opMode on HMI **************
     memset(char_buffer, 0, sizeof(char_buffer));
     strToChar(currentOpModeName); // mode name converted to chr in char_buffer
     topmode.setText(char_buffer);

    // update time field
     //String tm{NULL};
     memset(char_buffer, 0, sizeof(char_buffer));
     if (rtcFound) {
      // get time with seconds
      now = rtc.now();
      strToChar("hh:mm:ss"); // set format in char_buffer
      now.toString(char_buffer); // put time in char_buffer
      ttime.setText(char_buffer);
      ttime.Set_background_color_bco(GREEN);
     }
     else {
        strToChar("no RTC!"); // time converted to chr in char_buffer
        ttime.setText(char_buffer);
        ttime.Set_background_color_bco(RED);
       }
     
     tready.Set_background_color_bco(GREEN);

     // RTC status
     if (rtcFound){
        trtc.Set_background_color_bco(GREEN);
     }
     else { trtc.Set_background_color_bco(RED); }

    break;

    case 1:
      // this is page 1; opmode selection
      // set color of current opmode button to green
      setOpmodeButtonColors();
    break;

    case 2:
      // this is the RTC page
      // update all fields

    default:
    break;
   }
   displayTimer = millis();
 } // end check display timer
} // end updateDisplayAndIndicators

void setup()
{
  Serial.begin(9600);    // for debuging, handled by Nextion library see Nextion.h
  Serial1.begin(115200); // XBee for telemetry
  Serial2.begin(115200); // HMI communication

  setGPIOs();

  // set yellow alert off
  digitalWrite(LED_YELLOW_ALERT_CONDITION, LOW); //debug to indicate end of init
  digitalWrite(LED_GREEN_SYSTEM_READY, LOW); // now system is not ready

  // starts RTC
  rtcFound = kva_rtc_init();
  if (!rtcFound) {
    digitalWrite(LED_YELLOW_ALERT_CONDITION, LOW); //debug to indicate end of init
  }

  motorAllStop();

  // prepare PID controlers for L & R motors
    pid_L.setParams(1, 0.25, 1, 255); // left motor
    pid_R.setParams(1, 0, 0, 255); // right motor

  // attach interrupt to S1 encoder of each motor
  attachInterrupt(digitalPinToInterrupt(S1motorEncoder_L_PIN), ISR_readEncoder_L, RISING);
  attachInterrupt(digitalPinToInterrupt(S1motorEncoder_R_PIN), ISR_readEncoder_R, RISING);

  //setup timer interrupt for motor encoders
  encoderTimer.setPeriod(ENCODER_MEASURE_INTERVAL); // in microseconds
  encoderTimer.attachInterrupt(ISR_timerEncoder);
  encoderTimer.start();

  hmiFound = nexInit(); // start HMI

  /* attach all event callback functions to HMI components. */

  bstat1.attachPop(bstat1PopCallback, &bstat1);
  bstat2.attachPop(bstat2PopCallback, &bstat2);
  bop0.attachPop(bop0PopCallback, &bop0);
  bop2.attachPop(bop2PopCallback, &bop2);
  btele.attachPop(btelePopCallback, &btele);
  bsby.attachPop(bsbyPopCallback, &bsby);
  bfrun.attachPop(bfrunPopCallback, &bfrun);
  brtc0.attachPop(brtc0PopCallback, &brtc0);
  brtc1.attachPop(brtc1PopCallback, &brtc1);
  bplus5.attachPop(bplus5PopCallback, &bplus5);
  bplus1.attachPop(bplus1PopCallback, &bplus1);
  bminus.attachPop(bminusPopCallback, &bminus);
  bsetRTC.attachPop(bsetRTCPopCallback, &bsetRTC);

  /* this is the default opmode at start of vehicule */
  currentOpMode = STANDBY;

  digitalWrite(LED_GREEN_SYSTEM_READY, HIGH); // now system is ready
  digitalWrite(FLIPFLOP_PORT, LOW);
  debugln("Setup finished\n");

  #ifdef SENSORS_COMPILE
    // attach the isr to proper interrupt for contact sensors
       attachInterrupt(digitalPinToInterrupt(RIGHT_CONTACT_SENSOR_PIN), isr_right_contact_sensor, RISING);
       attachInterrupt(digitalPinToInterrupt(CENTER_CONTACT_SENSOR_PIN), isr_center_contact_sensor, RISING);
       attachInterrupt(digitalPinToInterrupt(LEFT_CONTACT_SENSOR_PIN), isr_left_contact_sensor, RISING);
  #endif  

    // set random function to decide certain events
       randomSeed(analogRead(ANALOG_PORT_RANDOM_SEED));
}

void loop()
{
 //
 // reverse FLIPFLOP_PORT
 // to be monitored externally i.e. an oscilloscope
 //
 digitalWrite(FLIPFLOP_PORT, !digitalRead(FLIPFLOP_PORT));
 updateDisplayAndIndicators();
 nexLoop(nex_listen_list);
 manageOpModeChange();
 runOpMode(currentOpMode);
 delay(10); //#TODO this delay will have to be ajusted later
}
#endif //endif for COMPILE_MAIN
