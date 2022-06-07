#ifndef _kva_h
#define _kva_h 

//functions prototypes
void set_ps2x();
void motor_TELEOP_node_v1();
void run_preset_course(void);
void free_run(void);
void standby(void);
void runOpMode(byte);

#ifdef TELEMETRY
String assembleMessage(Topic);
byte storeTopic(byte, String, char, byte);
void sendTelemetry(byte);
#endif

void setGPIOs(void);
void updateDisplayAndIndicators(void);
void manageOpModeChange(void);
void strToChar(String);

#ifdef RTC_COMPILE
boolean kva_rtc_init(void); // to starts RTC
String strDateTime(bool); // gets date time in string
#endif

void setOpmodeButtonColors(void);
void updateDTtoHMI(void);
void incrementField(int);
void setRTCfromInput(void);

//opmodes
#define STANDBY 0                        // at rest but diagnostic and communication running
#define SENSORS_DEVELOPEMENT 15          // as it says
#define RUN_PRESET_COURSE 30             // executes a series of preset commands
#define FREE_RUN  35                     // runs in obstacle collision avoidance on
#define TELEOP 10                        // Teleoperation with a joystick // TELEOP: Joystick operation

char char_buffer[25] = {0};              // C-style char buffer to hold String data

//***STATUS*****STATUS*****STATUS*****STATUS*****STATUS*****STATUS*
//*****************************************************************
// Status definitions for displays, indicators, etc.
//*****************************************************************
//***STATUS*****STATUS*****STATUS*****STATUS*****STATUS*****STATUS*

#define LED_YELLOW_ALERT_CONDITION 53
#define LED_GREEN_SYSTEM_READY 52

// *** STATUS FOR OPMODE CONTROL
byte currentOpMode;                   // operation mode
String currentOpModeName;             // text value of current opMode name 
boolean currentOpModeOK{false};       // indicates if current opmode is OK
byte requestedOpMode;                 // user requested opmode
boolean opModeChangeRequested{false}; // indicates opmode change requested from HMI
boolean opModeChangeAutorized{false}; // indicates that a change of opMode is autorized

// status for motor control
int motorSpeed_L = 0; // Motor Speed Values - Start at zero
int motorSpeed_R = 0;
#define MOTOR_LOWER_PWM_LIMIT 25 // to avoid buzzing
#define EMERGENCY_SLOW 150 //emergency speed to get out of trouble

// status for TELEOP mode
int  PS2_config_result{254}; // controler never set = 254
byte PS2_type{0};

//*************** motors encoders (Hall) sensors definitions

const byte S1motorEncoder_L_PIN = 22;  // motor encoder S1 A pin
const byte S2motorEncoder_L_PIN = 24;  // motor encoder S2 B pin 
const byte S1motorEncoder_R_PIN = 26;  // motor encoder S1 pin
const byte S2motorEncoder_R_PIN = 28;  // motor encoder S2 pin

// motor direction
const byte FORWARD {0};
const byte BACKWARD {1};

//*************************************************************
//definitions for Ps2 controler for teleoperation
//*************************************************************

PS2X ps2x; // create PS2 Controller Class object

// SPI bus pins on Arduino Due ICSP connector near SAM3X8E chip
#define SPI_MISO 74
#define SPI_MOSI 75
#define SPI_CLK  76
#define PS2X_CS  49 // chip select for PS2X controler
//byte PS2_vibrate_level{0}; // no vibration

/* FREE_RUN: Autonomous Driving with Collision Avoidance: 
 *  the vehicule navigates freely, begining driving forward from a start point while avoiding collisions.
*/

//******************************************************
// definitions for MEASURE_AND_CALIBRATE_MOTORS opMode
//******************************************************
#define ENCODER_MEASURE_INTERVAL 100000 // sampling interval = timer interval; set in microseconds (10e-6s)

//*******************************************
//************* motor control definitions
//*******************************************

// left motor on L298N channel A
#define ENA_L_PIN 4 // pwm pin
#define IN1_PIN   29
#define IN2_PIN   27

// right motor on L298N channel B
//#define ENB_R 2 // pwm pin
#define ENB_R 3 // pwm pin
#define IN3   25
#define IN4   23

#endif
