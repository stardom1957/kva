#ifndef _kva_history_h
#define _kva_history_h
/* 
 * see Kookye_Vehicule_Planning_v2.planner to follow developement and issues
 * 
 * KVA V1: first integration of Ps2 controler to replace joystick in TELEOP mode. In this version,
 *  - function motor_TELEOP_node_v1(): 
 *    - will handle an on the spot rotation by setting one motor to full speed and the opposite one to 0,
 *      wich meens that full X left or full X right stick value while Y stick is in at rest will produce no movement.
 *
 * KVA_V2: continuing integration. Changes for pin assignment for motorization.
 * 
 * 2019-06-23: projet created in my GitHub account. Pushed from KVA_V2 into ...Kookye_vehicule/kva.
 * History: 
 * ... will be maintained here and,
 * ... using issues, etc., on GitHub when mentionned
 * Issue 2.12 - Integrating and testing motor sensors through logic level shifter (8 ch):
 *  - 2.12.2: 
 *  -- DueTime encoderTimer is enabled during setup, but does not run if PWM signal is routed to pin 2 (ENB_R):
 *  --- #DONE see new definition for ENB_R
 *  - #DONE all code for measuring motors te be modified to simply report motor encoder counts
 *    every time the program passes trough.
 * Issue 2.10.6: teleoperating -  emergency maneuvers:
 *  - #DONE slow in place rotation using PS2 controler l/r buttons
 *  - #DONE slow forward/revers using PS2 controler up/down buttons
 * 
 * Issue 6.3: On board displays and controls using Nextion HMI:
 *  - on kva_menu3.hmi.
 *  - #DONE Nextion library
 *  - #DONE  Nextion HMI page menu for setting opMode
 *  - #DONE  Nextion HMI page for STATUS
 *  - on branch hmiv3:
 *  -- #DONE replace opmode select DS button on page 1 by standard button
 *  -- #DONE opMode opmode select button set to GREEN and other set to GRAY upon entering page 1 using callback
 *  -- #DONE commit changes
 *  - on branch master: 
 *  -- #DONE merge hmiv3 branch
 *  -- #DONE delete hmiv3
 *  
 *  ON BRANCH issue7_2_1 vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
 *  
 * Issue 7.2.1 RTC:
 *  - on
 *  -- #DONE init RTC module according DS3231Due_test.ino with ZS-042 connected to SDA1 and SCL1
 *  -- #DONE set RTC number fields page 2 according to RTC date and time
 *  -- #DONE RTC set from RTC SETUP page 2
 *  -- #DONE RTC set page 2 added a +1 btn
 *  -- #DONE test the above very thouroully <#####################################
 *  
 *  BRANCH issue7_2_1 merged and deleted
 * 
 * Issue 6.3: On board displays and controls using Nextion HMI:
 *  - #TODO SEEMS A BUG: sometimes system seems to reset when going to RTc page 2 from page 1
 *  - #DONE THERE'S A BUG: on change status when a opmode is allready yellow a press on a btn that is gray cancels the change: SHOULD do nothing <#####################################
 *  - #DONE opmode change implement mechanics using btn color and status variable
 *  - #TODO implement general diagnotics for sensors and peripheral and various status report using updateDisplayAndIndicators() for HMi and LEDs:
 *  -- #TODO changes from FREE_RUN to other modes to consider (because one  master switch for motors power and sensors and peripherals (5 vdc) is used!?
 *  -- #TODO will probably change page 0 to add dt values and other pages if needed
 *  
 * Issue 6.2.1 Software (class) developement:
 *  - #TODO message passing routines see telemetry.h
 * 
 * * vvvvvv ON BRANCH statusVersion2 vvvv ON BRANCH statusVersion2 vvvv ON BRANCH statusVersion2 vvvv ON BRANCH statusVersion2 vvvv
 * Issue 6.3: On board displays and controls using Nextion HMI: version 2
 *  - #TODO implement status reported in title on page (0)
 *  - #TODO implement heartbeat to the right of status (title) on page 0 (perhaps using two agjascent square that change color to green every 0.5 sec)
 *  - #TODO implement one button (w/o callback) for each state using color grey == not implemented, yellow == problem, green == ok, red == big problem
 *  -- #TODO perhaps place these buttons in a sort of matrix drawn with lines
 *  
 * ^^^^ END OF BRANCH statusVersion2 ^^^^ END OF BRANCH statusVersion2 ^^^^ END OF BRANCH statusVersion2 ^^^^ END OF BRANCH statusVersion2 ^^^^ END OF BRANCH statusVersion2
 * 
 *  BRANCH statusVersion2 merged into master and deleted
 *  ON BRANCH definitions_for_debugs vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
 *  
 *  - add any necessary #define statements to compile | not compile serial.print statements mainly used for debuging
 *  
 *  ON BRANCH debug_hmi vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
 *  - new numbers 
 *  
 3. section 12, see note inside KVA binder
4. free
5. section 4.5.4, 5v power bus
6. write the block diagram for the entire KVA vehicule. See tobotic book for an example
  see also section 2 motor control
7. Annex B to update DONE
8. section 2.10.1, schematic to make?
9. Annex AL
 9.1 block diagram?
 9.2 free
 
10. on vehicule DEL status (section 6.x):
  10.1 write a function to turn on off given condition DONE

11. free

12. For PID programming, use duetimer library with specific timer instead of firts timer
    provided by init function. This way we'll always be shure of time ID.
    
13. free
14. GREEN DEL on vehicule turns on when init done and vehicule ready DONE

15. XBee works at 115200 bps OK

16. TELEOP mode. revise code for ps2 controler:
  16.1 emergency (slow) movement not working properly: changed to buttons triangle, square, circle and cross DONE
  (**** on branch teleop_mode_opitmize ****)
  16.2 bug when joystick is realeased (autocenter): motor are not stopped DONE
  16.3 bug because of 16.2 fix, emergency does not run anymore SOLVED
  16.4 solve for joystick full left | right: should give same as emergency slow right | left??
  16.5 digitalWrite(PS2X_CS, LOW); when we enter teleop mode; should we do digitalWrite(PS2X_CS, HIGH); when leeving teleop mode??
  16.6 in joystick opeation, is delay(50); really necessary??
  

17. Issue 2.13.2. motor speed should be set from 1 to 10 ABANDONNED
branch motor_1_to_10 merged to master and deleted

18. Issue 5.1.1 #defines on branch «definitions_for_debugs» DONE

19. Issue 6.2.3 Simple messages Tx for debugs trought telemetry

20. Issue 2.5.10 TELEOP using simple commands trough telemetry (Xbee) for development of free run?

21. Nextion devel is on laptop. copy on kva and document using print screens
   This is project 23 in sketches binder

22. HMI has some issues: branch debug_hmi
  22.1 page 1 only update comming from page 0 (ok from page 2):
    22.1.2 due ti now version of Nextion Editor, ID numbers of certain objets had changed FIXED
    22.1.2 Removed unnecessary page change button on pages where not needed (like page 1 change on page 1!) DONE
    22.1.3 commit and merge into master

23. RTC not responding (on branch ********** debug_rtc ******) DONE
  23.1 tested on sketch on RTC_DS3231_r_glage_v3.ino on Uno: RTC working and date and time set OK
  23.2 replaced on KVA vehicule OK
  23.3 bug RTC not found!:
    23.3.1 had it connected to pins SCL1 and SDA1 instead of SCL 20 and SDA 21 now OK
    23.3.2 free
    23.3.3 RTC not responding, new sketch rtc_ds3231_on_due on KVA Due:
      23.3.3.1 works fine OK DONE
      23.3.3.2 hardware level shifted 3.3v- 5v only on SDA using 8-ports level shifter DONE
      23.3.3.3 kva_rtc_init() simplified base on rtc_ds3231_on_due to only set dt from compile time if power lost detected
       - runs OK DONE
      23.3.3.4 let run for extended period w/o changing opmode, just changing page from HMI: works OK DONE
      23.3.3.5 ran free run:
      23.3.3.5.1 could not read RTC during this run
      23.3.3.5.2 return to standby mode:
       - RTC gives 2007 12 12 and no time
       - yellow LED off
       - status page RTC is green
      23.3.3.5.3 reset Due:
       - yellow LED on
       - status page RTC is green
       - RTC no longuer reachable
    
    23.3.4. Replaced ZS-042 by Deek-Robot RTC DS1307Z with datalogger (microSD card reader)
      23.3.4.1 SDA will be shifted 5V - 3.3V. Determined from sketch rtc_and_data_logger_on_due on second Arduono Due,
               see project binder, section 8.1.
      23.3.4.2 integrated to vehicule in the following manner (as per section 8.1):
        - Arduino Due SDA (3.3V) -> SDA BUS bar -> Wavechare 8-ch. level shifter A4 -> Wavechare 8-ch. level shifter B4 -> mini data-logger SDA
        - In kva_rtc.h, changed RTC_DS3231 rtc; --> RTC_DS1307 rtc;
        - sketch rtc_and_data_logger_on_due uploaded on KVA Due for testing: RTC non trouvé!
        - from testing according to binder, section 8.1, it seems that level shifting was the culprit. It works perfectly if
          no level shifting is done. Connected this way (KVA I2C bus not used):
           -- 5V and ground from Arduino Due
           -- Arduino Due SDA --> data-logger SDA
           -- Arduini Due SCL --> data-logger SCL
         - test on I2C bus and KVA 5V bus OK
         - measured V (oscilloscope) SDA and SCL 3.4V
         - RTC works perfectly
      23.3.4.3 ran free run and teleop modes several times: RTC works oerfectly during and after

24. RTC diagnostics and indicators (on branch ********** debug_rtc ******):
  24.1 rtc running?: according to RTClib: isrunning(void)
  24.2 lost power (battery)

25. branch debug_rtc merged into master and deleted DONE

26. Worked out issue 16 TELEOP mode. revise code for ps2 controler DONE
  26.1 (**** on branch teleop_mode_opitmize ****)
  26.2 commit and merged into master
  26.3 (**** deleted branch teleop_mode_opitmize ****)

27. update:
  27.1 in updateDisplayAndIndicators, check on availability of RTC and report with YELLOW LED and status on HMI page 1 DONE
  27.2 label cable for RTC correctly and update annex B DONE

28. work on issue 21: document HMI i binder and copy code in kva folder

branch pid_developpement_old abandonned but kept (TELEOP not working anymore)
this was in pid_developpement_old
see binder, section    for details.

vvvvvvvvv on branch pid_devel vvvvvvvv

29. testing motor Hall encoders in FREE_RUN mode: FINISHED
  29.1 Left motor:
    29.1.1 connector MS3  (services pins 22, 24, 26, 28) temporalily disconnected from DUE DONE
    29.1.2 S1 <- Oscilloscope ch1 probe connected to cable MS3 (pin 22) DONE
    29.1.3 S2 <- Oscilloscope ch2 probe connected to cable MS3 (pin 24) DONE
    29.1.4 ch1 is reference trigger DONE
    29.1.5 signals are ok :: encoders are OK DONE
    29.1.6 direction (tested in TELEOP mode):
      - S1motorEncoder_L_PIN high (always):
         S2motorEncoder_L_PIN High = FORWARD
         S2motorEncoder_L_PIN LOW  = BACKWARD

  29.2 Right motor:
    29.2.1 connector MS3  (services pins 22, 24, 26, 28) temporalily disconnected from DUE DONE
    29.2.2 S1 <- Oscilloscope ch1 probe connected to cable MS3 (pin 26) DONE
    29.2.3 S2 <- Oscilloscope ch2 probe connected to cable MS3 (pin 28) DONE
    29.2.4 ch1 is reference trigger DONE
    29.2.5 signals are ok :: encoders are OK DONE
    29.2.6 direction (tested in TELEOP mode):
      - S1motorEncoder_R_PIN high (always):
         S2motorEncoder_R_PIN LOW = FORWARD
         S2motorEncoder_R_PIN HIGH  = BACKWARD

  29.3 git commit [pid_devel 99bbb65] DONE

30. removing MEASURE_AND_CALIBRATE_MOTORS mode: FINISHED
  30.1 removing code for motor testing : mode, ISR and timer (archived in BACKUPS/motor_testing_code) DONE
  30.2 upload to DUE and testing all modes DONE
  30.3 git commit [pid_devel 7601e08] DONE

31. PID implementation:
  31.1 create new kva_pid.h 
  31.2 create basic PID class DONE
  31.2 complete code in loop <--
    compare setMotor function with my own motor functions to determine usage of direction DONE
  31.3 create ISR for motor S1 counts:
    31.3.1 one that counts according to motor direction (based on S2)
    31.3.2 one that only calculate the difference in count during one cycle <----
  31.4 create functions for ramping speed up and down
  31.5 test pid evalu function w/o motor control
vvvvvvvvv branch pid_devel merged into master and deleted vvvvvvvv

vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
vvvvvvvvv on branch master vvvvvvvv

32. In general:
  32.1 documentation, photos for motor integration (L298N) and insertion in binder
  32.2 documentation, photos for IR and LIDAR sensors test gig and insertion in binder

33. Digital "flip flop" for  time off main loop: DONE
  33.1 variable flipflop done

2024-01-07
34. collision contact sensors (7.5): some code done in kva and see below. TODO
  34.1 In runopmode, intercept do_emergency before case statement
  34.2 bool do_emergency --> emergency_state
  34.3 develop handle_emergency in FREE_RUN mode

vvvvvvvvv branch master commit vvvvvvvv
vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv

vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
vvvvvvvvv on branch hmi_page3 vvvvvvvv

2024-01-24
35. Adding page 3 on HMI to serve to list the log:
   35.1 Add page 3 with one text field (tlog) DONE
   35.2 add two scroll buttons TODO
   35.3 Add nav buttons on page 0 to 2 to goto to page 3 DONE
   35.4 add appropriate code: TODO
    35.4.1 HMI page 3 page and other objects
    35.4.2 send test text to from Due upon page change from HMI

2024-02-01
vvvvvvvvv branch hmi_page3 merged into master and deleted vvvvvvvv
vvvvvvvvv return to main

vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
vvvvvvvvv on branch collision_sensor_devel vvvvvvvv

36. Test collision sensor ISR (in STANDBY mode) :
  36.1 modif ISR of right collision sensor to count up each entry
vvvvvvvvv branch collision_sensor_devel merged into master vvvvvvvv

vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
vvvvvvvvv on branch PID_control_compile vvvvvvvv

37. PID When PID_COMPILE is not defined, kva_pid.h should no be compiled DONE
   37.1 test dedined or not OK DONE
vvvvvvvvv branch PID_control_compile merge into master vvvvvvvv

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


XX. Divers :
  34.1 LED_YELLOW_ALERT_CONDITION will have to review it's usage
  34.2 TELEOP mode needs a real good review : report controller found even if not found (see run_setup variable)
  34.3 updateDisplayAndIndicators(void) : case for page 2 ????
  34.4 


Working on
 ----------
34 <---
28
23.3.4.5 I2C level shifter
 */

      // ****************      
      // emergency turn *
      // ****************
/*
      if ( do_emergency_turn && !do_wait ) {
        if ( (EMERGENCY_REVERSE_RUN_TIME + EMERGENCY_WAIT_TIME < emergency_run_time) &&
             (emergency_run_time < EMERGENCY_REVERSE_RUN_TIME + EMERGENCY_WAIT_TIME + EMERGENCY_TURN_RUN_TIME) ) {

          // turn direction is dependant of wich contact sensor is triggered
          debugln("in handle emergency turn");
          switch(contact_sensors_ID) {
            case RIGHT_CONTACT_TRIGGERED:
              // turn left
              vehiculeRotateLeft(EMERGENCY_SLOW);
              break;

            case CENTER_CONTACT_TRIGGERED:
              // turn as set in entry setup section above
              if (turnRightOrLeft) { vehiculeRotateLeft(EMERGENCY_SLOW); }
                 else { vehiculeRotateRight(EMERGENCY_SLOW); }
              break;

            case LEFT_CONTACT_TRIGGERED:
              // turn right
              vehiculeRotateRight(EMERGENCY_SLOW);
              break;
          }
        }
          // the turn is finished
          else {
            do_emergency_turn = false;
            // stop both motors
            digitalWrite(ENB_R, LOW);
            digitalWrite(ENA_L_PIN, LOW);
            do_wait = true; // flag for last wait time before exit of emergency
          }
      }  // do_emergency_turn && !do_wait

      // ************************
      // Going out of emergency *
      // ************************

      // last wait time before going out of emergency
      
      if ( do_wait &&
           !do_emergency_reverse &&
           !do_emergency_turn &&
           (emergency_run_time < EMERGENCY_REVERSE_RUN_TIME + EMERGENCY_TURN_RUN_TIME + 2 * EMERGENCY_WAIT_TIME) ) { // where finished emergency
        ; // do nothing during wait time
      }

        else {
          // setup to for calling_mode re-entry
          do_emergency = false;  // used in run_op_mode
          run_setup = true; // to ensure that setup code is run upon reentry in calling op mode

          // reset contact_sensor_triggered_ID
          contact_sensors_ID = 0;
          // re-attach interrupts
          //attachInterrupt(digitalPinToInterrupt(RIGHT_CONTACT_SENSOR_PIN), isr_right_contact_sensor, RISING);
        }



#endif
