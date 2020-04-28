#ifndef _kva_history_h
#define _kva_history_h
/* 
 * see Kookye_Vehicule_Planning_v2.planner to follow develpement and issues
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
 *  on branch issue7_2_1 vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
 *  
 * Issue 7.2.1 RTC:
 *  - on
 *  -- #DONE init RTC module according DS3231Due_test.ino with ZS-042 connected to SDA1 and SCL1
 *  -- #DONE set RTC number fields page 2 according to RTC date and time
 *  -- #DONE RTC set from RTC SETUP page 2
 *  -- #DONE RTC set page 2 added a +1 btn
 *  --- #TODO test the above very thouroully <#####################################
 * 
 * Issue 6.3: On board displays and controls using Nextion HMI:
 *  - #TODO SEEMS A BUG: sometimes system seems to reset when going to RTc page 2 from page 1
 *  - #TODO why is HMI select automaticaly page 0 when reset even if HMi is runing?
 *  - #DONE opmode change implement mechanics using btn color and status variable
 *  - #TODO implement general diagnotics for sensors and peripheral and various status report using updateDisplayAndIndicators() for HMi and LEDs:
 *  -- #TODO changes from FREE_RUN to other modes to consider (because one  master switch for motors power and sensors and peripherals (5 vdc) is used!?
 *  - #TODO will probably change page 0 to add dt values and other pages if needed
 *  
 * Issue 6.2.1 Software (class) developement:
 *  - #TODO message passing routines see telemetry.h
 * 
*/
#endif

