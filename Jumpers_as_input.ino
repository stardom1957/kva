/*
 * This module reads the values of 3 jumpers and sets the operation mode according
 * to the binary value.
 * 
 * Standby request jumper:
 *  The jumper connected to D50 is the mode select request. 
 *   When set (1): 
 *    - this is the mode select request command mode and the opMode is changed to STANDBY
 *    - the values of J1_PIN and J2_PIN are read continuously every 50 ms.
 *    When reset (0):
 *    - the values of J1_PIN and J2_PIN are NOT read anymore.
 *    - the opMode is set corresponding to the last values of J1_PIN and J2_PIN
 *      (see table below).
 *
 * Jumper value table and opMode
 * 
 * J1_PIN is LSB of byte jumperValues
 * J1_PIN J2_PIN opMode                           jumperValues
 * ----------------------------------------------------
 *  0  0 STANDBY                             0
 *  0  1 TELEOP                              1
 *  1  0 MEASURE_AND_CALIBRATE_MOTORS        2
 *  1  1 JUST_RUN                            3
 */
#ifdef JUMPERS_AS_INPUT

// robot vehicule modes of operation see vehiculeModes.txt
byte opMode;                             //operation mode
byte jumperValues{0};                        //jumper values
#define MODE_SELECT_PIN 50  //D50 mode select request
#define J1_PIN 51           //D51 from LSB of jumperValue (bit 0)
#define J2_PIN 52           //D52 from LSB of jumperValue (bit 1)
/*
 */

#define STANDBY 0                        //at rest but diagnostic and communication running
#define JUST_RUN 30                      // just run
#define MEASURE_AND_CALIBRATE_MOTORS 40  //used to test what ever needs testing
#define TELEOP 10                        //Teleoperation with a joystick // TELEOP: Joystick operation
//#define NADDOCAM 20                      //NADDOCAM: Non Directed Autonomous Driving with Obstacle Collision Avoidance mode:
/* NADDOCAM: Non Directed Autonomous Driving with Obstacle Collision Avoidance mode: 
 *  the vehicule navigates freely, begining driving forward from a start point while avoiding collisions.
*/

// this module reads J1_PIN and J2_PIN and set/reset values of corresponding
// bits in jumperValues
void readOpModeJumpers(void) {
  bitWrite(jumperValues, 0, digitalRead(J1_PIN));
  bitWrite(jumperValues, 1, digitalRead(J2_PIN));
}

void setOpMode(byte jp){
   // set new mode into opMode
  Serial.print("jumperValues = ");
  Serial.println(jp, DEC);
  switch (jp) {
  case 0:
    opMode = STANDBY;
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.print("opMode : ");
    Serial.print(opMode, DEC);
    Serial.println(" (STANDBY selected by user)");
    break;
  case 1:
    opMode = TELEOP;
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.print("opMode : ");
    Serial.print(opMode, DEC);
    Serial.println(" (TELEOP)");
    break;
  case 2:
    opMode = MEASURE_AND_CALIBRATE_MOTORS;
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.print("opMode : ");
    Serial.print(opMode, DEC);
    Serial.println(" (MEASURE_AND_CALIBRATE_MOTORS)");
    break;
  case 3:
    opMode = JUST_RUN;
    digitalWrite(LED_BUILTIN, LOW);
    Serial.print("opMode : ");
    Serial.print(opMode, DEC);
    Serial.println(" (JUST_RUN)");
    break;
  }
}

void setup() {
  opMode = STANDBY;
//debug  pinMode(LED_BUILTIN, OUTPUT); //if on solid = in operation
//debug  digitalWrite(LED_BUILTIN, LOW);
  
  Serial.begin(115200);

  // set jumper pins to input
  pinMode(MODE_SELECT_PIN, INPUT);
  pinMode(J1_PIN, INPUT);
  pinMode(J2_PIN, INPUT);
  Serial.println("Start of program... vehicule in STANDBY mode.");
}

void loop() {
  //while MODE_SELECT_PIN is HIGH, read new opMode into jumperValues
  if (digitalRead(MODE_SELECT_PIN) == HIGH) {
    Serial.println("Mode select mode is selected. The vehicule has been put in STANDBY mode.");
    // run code to put vehicule in standby mode
  }
  while (digitalRead(MODE_SELECT_PIN) == HIGH) {
    readOpModeJumpers();
    delay(50);
  }
  setOpMode(jumperValues);
  delay(1500);
}
#endif
