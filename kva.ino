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
//#define TELEMETRY               // compile telemetry code
#define PID_COMPILE // compile kva_pid.h

// definition of debug levels
// mainly replacing Serial.print and Serial.println by nothing when not needed
// that is DEBUG == 0

#define DEBUG 1 // 1 means debugging is active 0 means it's not
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

// run time definitions for emergency manoeuvers in microseconds
// 1 000 000 microseconds by second
#define EMERGENCY_REVERSE_RUN_TIME 2000000
#define EMERGENCY_TURN_RUN_TIME 2500000
#define EMERGENCY_WAIT_TIME 500000 // wait time between emergency reverse and turn

unsigned long emergency_function_last_exit_time{0};
bool do_emergency_reverse{false};
bool do_wait{false};
bool do_emergency_turn{true};
unsigned long emergency_run_time;
int turnRightOrLeft{0};

void handle_emergency(byte calling_mode) {
  // update emergency run time to account for the time past OUTSIDE this function
  // if it's the first entry (contact_sensor_just_triggered is set), the value is irrelevant and will be set to 0
  // in each case upon first entry
  
  emergency_run_time += (micros() - emergency_function_last_exit_time);
  
  switch(calling_mode) {
    case FREE_RUN:
      /* About wait times:
         The majority of time is passed OUTSIDE of this function!
         
         Therefore, do_wait is a flag used to freeze an ongoing operation by preventing the start
         of any other operation before EMERGENCY_WAIT_TIME has elapsed for this particular operation.
      */

      
      /* *******************************************
         Setup for first entry; that is            *
         right after contact_sensor_just_triggered *
         is set by one of the sensor contact ISR   *
        ********************************************
      */

      if (contact_sensor_just_triggered) {
        debugln("in contact_sensor_just_triggered");
        contact_sensor_just_triggered = false;
        emergency_run_time = 0;
        do_emergency_reverse = true; // start reverse operation
        do_wait = false; // wait time in effect
        do_emergency_turn = false; // not yet doing turn

        // initialise random direction for turn when center contact sensor
        // has triggered.
        // We want to turn left or right randomly 50% of the time in on direction.
        turnRightOrLeft = random(0, 1);
        debug(" turnRightOrLeft: ");
        debugln(turnRightOrLeft);
        debugln();
      }
      
      // *******************
      // emergency reverse *
      // *******************
      
      if ( do_emergency_reverse) { 
	      if ( emergency_run_time < EMERGENCY_REVERSE_RUN_TIME ) {
          debugln("in do_emergency_reverse, motors should run in reverse");
          motorLeftSet(EMERGENCY_SLOW, BACKWARD);
          motorRightSet(EMERGENCY_SLOW, BACKWARD);
	      }
          else {
            // stop both motors
            digitalWrite(ENB_R, LOW);
            digitalWrite(ENA_L_PIN, LOW);
          }

        // waiting EMERGENCY_WAIT_TIME
        if ( emergency_run_time > (EMERGENCY_REVERSE_RUN_TIME + EMERGENCY_WAIT_TIME) ) {
            do_emergency_reverse = false;
            do_emergency = false;  // used in run_op_mode
            run_setup = true; // to ensure that setup code is run upon reentry in calling op mode
        }
      }

    break; // FREE_RUN

    case TELEOP:
    break; //TELEOP

    default:
    break;
  } // end switch... case

  // update exit time
  emergency_function_last_exit_time = micros();
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
  if (run_setup) {
     motorAllStop();
     run_setup = false;
  }
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
    if (run_setup) {
      // setup
      currentOpModeName = "FREE RUN";
      l_speed = FREE_RUN_TARGET_LEFT;
      r_speed = FREE_RUN_TARGET_RIGHT;
      run_setup = false;
    }

    // we would do pid here
    //l_speed = target_L;
    //r_speed = target_R;

    motorLeftSet(l_speed, FORWARD);
    motorRightSet(r_speed, FORWARD);  
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

    case TELEOP: // teleoperation by gamepad type controller
     motor_TELEOP_node_v1();
     break;

    case FREE_RUN:
     if (do_emergency) {
      handle_emergency(FREE_RUN);
     }
       else {
         free_run();
       }
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

  digitalWrite(LED_YELLOW_ALERT_CONDITION, LOW); // yellow LED will indicate start of setup

  // contact sensors
  pinMode(RIGHT_CONTACT_SENSOR_PIN, INPUT);
  pinMode(LEFT_CONTACT_SENSOR_PIN, INPUT);
  pinMode(CENTER_CONTACT_SENSOR_PIN, INPUT);
}

// manages change of opMode
void manageOpModeChange(void) {
  if (opModeChangeRequested && opModeChangeAutorized) {
   debugln("in manageOpModeChange");
   //dbSerialPrintln("  ");
   currentOpMode = requestedOpMode;
   run_setup = true; // ensures that mode setup code is run during first function entry of current opmode

   debug("  debug requestedOpMode, value is: ");
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

  /* this is the default opmode at start of vehicule */
  currentOpMode = STANDBY;
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
