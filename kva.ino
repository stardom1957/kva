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
//#define PID_COMPILE // compile kva_pid.h

/* definition of debug levels
    mainly replacing Serial.print and Serial.println by nothing when not needed
    that is:
     DEBUG_LEVEL_0 used to debug HMI
     DEBUG_LEVEL_1 used to debug handle_emergency
*/ 
#define DEBUG_LEVEL_0 0 // 1 means debug level 0 is active 0 means it's not
#if DEBUG_LEVEL_0 == 1
  #define debugp(x) Serial.print(x)
  #define debug Serial.print
  #define debug_2arg(x, y) Serial.print(x, y)
  #define debugln Serial.println
  #define debugln(x) Serial.println(x)
  #define debugln_2arg(x, y) Serial.println(x, y)
#else
  #define debugp(x)
  #define debug
  #define debug_2arg(x, y)
  #define debugln(x)
  #define debugln_2arg(x, y)
#endif

#define DEBUG_LEVEL_1 0 // 1 means debug 1 is active 0 means it's not
#if DEBUG_LEVEL_1 == 1
  #define debug1(x) Serial.print(x)
  #define debug1 Serial.print
  #define debug_2arg1(x, y) Serial.print(x, y)
  #define debugln1(x) Serial.println(x)
  #define debugln1 Serial.println
  #define debugln_2arg1(x, y) Serial.println(x, y)

#else
  #define debug1(x)
  #define debug1
  #define debug_2arg1(x, y)
  #define debugln1(x)
  #define debugln_2arg1(x, y)
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
#ifdef PID_COMPILE
#include "kva_pid.h"
#endif

/* Collision sensors - handling of emergency status ***
   About counting time:
      Since KVA is operated in a continuous polling mode, we cannot time the manoeuvers and wait times
      by using a blocking function like delay()! We must cumulate the time in the variable emergency_run_time
      to count time. The time is taken upon exit of this function and taken again at the entry.

      Since the majority of time is passed OUTSIDE of this function, we don't bother to count the time spent
      working INSIDE!

      Each section handles the timing of particular menoever using a flag and a check of the total time elapse during
      emergency against a particular timing value (i. e. WAIT1). At the end of the allowed time, the action pertaining
      to the next section will be initiated.
      
      The elapsed time is calculated using these values in the #defines below (in microseconds) for all
      of these operations done in the following order.
*/

#define EMERGENCY_WAIT_RUN_TIME1    2000000 // delay time after setup in handle_emergency; right after motor stop
#define EMERGENCY_REVERSE_RUN_TIME  4000000 // run motor in reverse for this amount of time
#define EMERGENCY_WAIT_RUN_TIME2    2000000 // motor stopped for this amount of time after reverse
#define EMERGENCY_TURN_RUN_TIME     3000000 // turn vehicule in place (left/right) for this amount of time
#define EMERGENCY_WAIT_RUN_TIME3    3000000 // delay for this amount of time before exit and return to calloing mode

/*
 Used for timing operations in microseconds
*/
unsigned long emergency_function_last_exit_time{0};
unsigned long emergency_run_time{0};

/*
  Flags used to control each particular operation for the amount of time specified bye *_RUN_TIMEx values above
*/

bool do_emergency_reverse{false};
bool do_emergency_turn{false};
bool do_wait1{false};
bool do_wait2{false};
bool do_wait3{false};

/*
  turnRightOrLeft is used to randomly turn in one direction or the other depending on the ID of the
  sensor triggered.
*/

short int turnRightOrLeft{0};

void handle_emergency(byte calling_mode) {
  // calculate the time elapsed since last exit this function
  emergency_run_time += (micros() - emergency_function_last_exit_time);
  
  switch(calling_mode) {
    case STANDBY:
      /* setup ******
         Setup for first entry in emergency mode; that is, right after one of the ISR
         has set contact_sensor_just_triggered and;
         imediately at the end, starts do_wait1
      */
      if (contact_sensor_just_triggered) {
        debugln1("In STANDBY mode. Reset of contact trigger.");
        debug1("contact_sensors_ID= ");
        debugln1(contact_sensors_ID);
        debugln1();

        debug1("Number of contacts detected in ISR= ");
        debugln1(nbr_of_contact);
        debugln1();

        // init of flags and 
        emergency_run_time = 0; //reset time cownter
        nbr_of_contact = 0;

        // Wher'e not moving so terminate emergency state and reset variables
        contact_sensor_just_triggered = false;
        state_of_emergency = false;
        contact_sensors_ID = 0; // reset sensor contact ID
      } // if contact_sensor_just_triggered

    break;

    case FREE_RUN:
    case TELEOP:
      /* setup ******
         Setup for first entry in emergency mode; that is, right after one of the ISR
         has set contact_sensor_just_triggered and;
         imediately at the end, starts do_wait1
      */
      if (contact_sensor_just_triggered) {
        // report wich contact has triggered
        debug1("contact_sensors_ID= ");
        debugln1(contact_sensors_ID);
        debugln1();

        // turn motors off
        debugln1("Setup... stopping motors");
        motorAllStop();

        // init and reset of flags
        bool do_emergency_reverse = false;
        bool do_emergency_turn = false;
        bool do_wait2 = false;
        bool do_wait3 = false;
        emergency_run_time = 0; //reset time cownter

        debug1("emergency_run_time= ");
        debug1(float(emergency_run_time) / 1000000, DEC);
        debugln1(" s");

        debug1("Number of contacts detected in ISR= ");
        debugln1(nbr_of_contact);
        debugln1();

        digitalWrite(LED_YELLOW_ALERT_CONDITION, HIGH);
        debugln1("Start do_wait1");

        // initialise random direction for turn when center contact sensor
        // has triggered.
        // We want to turn left or right randomly 50% of the time each one direction
        turnRightOrLeft = random(0, 2);

        // terminates this step and starts next one
        contact_sensor_just_triggered = false;
        do_wait1 = true;

      } // if contact_sensor_just_triggered

      /* do_wait1 ******
        Handles timing of do_wait1. At this point the timer (emergency_run_time) starts to count and we must
        accumulate the time for a delay of WAIT1 microseconds and;
        at the end of WAIT1 micriseconds, starts do_emergency_reverse
      */

      if (do_wait1 && (emergency_run_time > EMERGENCY_WAIT_RUN_TIME1)) {
          debug1("emergency_run_time= ");
          debug1(float(emergency_run_time) / 1000000, DEC);
          debugln1(" s");

          debug1("Number of contacts detected in ISR= ");
          debugln1(nbr_of_contact);
          debugln1();

        // terminates this step and starts next one
          do_wait1 = false;
          do_emergency_reverse = true;

          // start reversing motor
          debugln1("Start reverse");
          motorLeftSet(EMERGENCY_SLOW, BACKWARD);
          motorRightSet(EMERGENCY_SLOW, BACKWARD);
      }

      /* do_emergency_reverse ******
        Handles timing of do_emergency_reverse and;
        at the end of EMERGENCY_REVERSE_RUN_TIME microseconds, starts do_wait2
      */
      if (do_emergency_reverse && emergency_run_time > \
            (EMERGENCY_WAIT_RUN_TIME1 + EMERGENCY_REVERSE_RUN_TIME)) {
         /*
           Finished reversing, initiate wait of WAIT2 time 
         */
           debug1("emergency_run_time= ");
           debug1(float(emergency_run_time) / 1000000, DEC);
           debugln1(" s");

           debug1("Number of contacts detected in ISR= ");
           debugln1(nbr_of_contact);
           debugln1();

           debugln1("Start WAIT2");

           // terminates this step and starts next one
           // turn motors off
           debugln1("Stopping motors");
           motorAllStop();
           do_wait2 = true;
           do_emergency_reverse = false;
      } // do_emergency_reverse

      /* do_wait2 ******
        Handle timing of do_wait2 and;
        at the end of EMERGENCY_WAIT_RUN_TIME2 microseconds, starts do_emergency_turn
      */
      if (do_wait2 && emergency_run_time > \
           (EMERGENCY_WAIT_RUN_TIME1 + EMERGENCY_REVERSE_RUN_TIME + EMERGENCY_WAIT_RUN_TIME2)) {
        debug1("emergency_run_time= ");
        debug1(float(emergency_run_time) / 1000000, DEC);
        debugln1(" s");

        debug1("Number of contacts detected in ISR= ");
        debugln1(nbr_of_contact);
        debugln1();

        // terminates this step and starts next one
        do_wait2 = false;
        do_emergency_turn = true;
        debugln1("Start the turn");

        debug1("contact_sensors_ID= ");
        debugln1(contact_sensors_ID);
        debugln1();

        // turn direction is dependant of wich contact sensor is triggered
        switch(contact_sensors_ID) {
          case RIGHT_CONTACT_TRIGGERED:
          case RIGHT_CONTACT_TRIGGERED + CENTER_CONTACT_TRIGGERED: 
            // turn left
            vehiculeRotateLeft(EMERGENCY_SLOW);
            debugln1("Turning left");
          break;

          case CENTER_CONTACT_TRIGGERED:
            // turn as set in entry setup section above
            if (turnRightOrLeft) { //0 is left
               vehiculeRotateLeft(EMERGENCY_SLOW);
               debugln1("at random turning left");
            }
              else { //1 is right
                vehiculeRotateRight(EMERGENCY_SLOW);
                debugln1("at random turning right");
              }
          break;

          case LEFT_CONTACT_TRIGGERED:
          case LEFT_CONTACT_TRIGGERED + CENTER_CONTACT_TRIGGERED: 
          // turn right
            vehiculeRotateRight(EMERGENCY_SLOW);
            debugln1("Turning right");
          break;

          default: // all other contact combinaison are random turn
            debugln1("Multiple contact combinaison");
            if (turnRightOrLeft) { //0 is left
               vehiculeRotateLeft(EMERGENCY_SLOW);
               debugln1("at random turning left");
            }
              else { //1 is right
                vehiculeRotateRight(EMERGENCY_SLOW);
                debugln1("at random turning right");
              }
          break;
        }
      }

      /* do_emergency_turn ******
        Handle timing of do_emergency_turn and;
        at the end of EMERGENCY_TURN_RUN_TIME microseconds, stars do_wait3
      */
      if (do_emergency_turn && emergency_run_time > \
           (EMERGENCY_WAIT_RUN_TIME1 + EMERGENCY_REVERSE_RUN_TIME + \
              EMERGENCY_WAIT_RUN_TIME2 + EMERGENCY_TURN_RUN_TIME)) {

            debug1("emergency_run_time= ");
            debug1(float(emergency_run_time) / 1000000, DEC);
            debugln1(" s");

            debug1("Number of contacts detected in ISR= ");
            debugln1(nbr_of_contact);
            debugln1();

            debugln1("Stopping the turn");
            // turn motors off
            motorAllStop();

            do_wait3 = true;
            do_emergency_turn = false;
       }

      /* do_wait3 ******
        Handle timing of do_wait3 and;
        at the end of EMERGENCY_WAIT_RUN_TIME3 microseconds, get out of emergency mode
      */
      if (do_wait3 && emergency_run_time > \
           (EMERGENCY_WAIT_RUN_TIME1 + EMERGENCY_REVERSE_RUN_TIME + EMERGENCY_WAIT_RUN_TIME2 +\
             EMERGENCY_TURN_RUN_TIME + EMERGENCY_WAIT_RUN_TIME3)) {
        debug1("emergency_run_time= ");
        debug1(float(emergency_run_time) / 1000000, DEC);
        debugln1(" s");

        debug1("Number of contacts detected in ISR= ");
        debugln1(nbr_of_contact);
        debugln1();
        /*
           Finished do_wait3, wher'e going out of emergency mode wich will put
           us right back a the calling mode (calling_mode)
        */
         digitalWrite(LED_YELLOW_ALERT_CONDITION, LOW);
         debugln1("Going out of emergency mode\n");
         state_of_emergency = false;
         contact_sensors_ID = 0; // reset contact sensor ID
         nbr_of_contact = 0;
      }
    break; // FREE_RUN


    default:
    break;
  } // end switch... case

  // update exit time
  emergency_function_last_exit_time = micros();
}

//************** OPMODE ***************************
//*********** RUN_PRESET_COURSE *******************

void run_preset_course(void) {
  if (run_setup) {
    run_setup = false;
    currentOpModeName = "RUN_PRESET_COURSE";
  }
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

  //while(true); //we're only doing this onceemer
  delay(5000);
}

//************ OPMODE ************************
//*********** STANDBY ***********************

void standby(void) {
  // #TODO
  if (run_setup) {
     currentOpModeName = "STANDBY";
     motorAllStop();
     run_setup = false;
  }
  ; //do nothong
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
  if (state_of_emergency) {
      handle_emergency(om);
  }
    else {
      switch (om) {
          case STANDBY:
          standby();
          break;

          case RUN_PRESET_COURSE:
          run_preset_course();
          break;

          case TELEOP: // teleoperation by gamepad type controller
          motor_TELEOP();
          break;

          case FREE_RUN:
            free_run();
          break;
          
          default:
          standby(); //safest thing to do
        }  // end switch
    } // else fo_emergency
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

  // set all the motor control pins to outputsattachInterrupt
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

   // stop motors
   motorAllStop();
   
   // make sure that emergency mode is cancelled
   if (state_of_emergency) {
      state_of_emergency = false;
      digitalWrite(LED_YELLOW_ALERT_CONDITION, LOW);
      contact_sensor_just_triggered = false;
      contact_sensors_ID = 0;
   }


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
     // debug digitalWrite(LED_YELLOW_ALERT_CONDITION, LOW); // reset alert condition

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

    // update time      opmode
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
  Serial1.begin(115200); // XBee for telemetryopmode
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

  #ifdef PID_COMPILE
    // prepare PID controlers for L & R motors
      pid_L.setParams(1, 0.25, 1, 255); // left motor
      pid_R.setParams(1, 0, 0, 255); // right motor
    opmode
    // attach interrupt to S1 encoder of each motor
    attachInterrupt(digitalPinToInterrupt(S1motorEncoder_L_PIN), ISR_readEncoder_L, RISING);
    attachInterrupt(digitalPinToInterrupt(S1motorEncoder_R_PIN), ISR_readEncoder_R, RISING);

    //setup timer interrupt for motor encoders
    encoderTimer.setPeriod(ENCODER_MEASURE_INTERVAL); // in microseconds
    encoderTimer.attachInterrupt(ISR_timerEncoder);
    encoderTimer.start();
  #endif //PID_COMPILE

  hmiFound = nexInit(); // start HMI

  /* attach all event callback functions to HMI co
 //
 // reverse LOOP_FLIPFLOP_PORTmponents. */

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
  digitalWrite(LOOP_FLIPFLOP_PORT, LOW);
  debugln("Setup finished\n");

  #ifdef SENSORS_COMPILE
    // attach the isr to proper interrupt for contact sensors
       attachInterrupt(digitalPinToInterrupt(RIGHT_CONTACT_SENSOR_PIN), isr_right_contact_sensor, RISING);
       attachInterrupt(digitalPinToInterrupt(CENTER_CONTACT_SENSOR_PIN), isr_center_contact_sensor, RISING);
       attachInterrupt(digitalPinToInterrupt(LEFT_CONTACT_SENSOR_PIN), isr_left_contact_sensor, RISING);

       // clear pending status of interrupt on each pin of the above pins 
       //NVIC_ClearPendingIRQ(PIOC_IRQn);
       //(PIOC PIOC ((Pio *)0x400E1200U) or PIOC_IRQn)
       //debug NVIC_ClearPendingIRQ(PIOC_IRQn); // (PIOC PIOC ((Pio *)0x400E1200U) or PIOC_IRQn)
       //debug NVIC_ClearPendingIRQ(PIOC_IRQn); // (PIOC PIOC ((Pio *)0x400E1200U) or PIOC_IRQn)
 #endif  

    // set random function to decide certain events
       randomSeed(analogRead(ANALOG_PORT_RANDOM_SEED));

  /* this is the default opmode at start of vehicule */
  currentOpMode = STANDBY;

} //end setup()

void loop()
{
 //
 // reverse LOOP_FLIPFLOP_PORT
 // to be monitored externally i.e. an oscilloscope
 //
 digitalWrite(LOOP_FLIPFLOP_PORT, !digitalRead(LOOP_FLIPFLOP_PORT));
 updateDisplayAndIndicators();
 nexLoop(nex_listen_list);
 manageOpModeChange();
 runOpMode(currentOpMode);
 delay(10); //#TODO this delay will have to be ajusted later
}
#endif //endif for COMPILE_MAIN
