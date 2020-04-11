#ifndef _kva_h
#define _kva_h 

//functions prototypes
void ISR_timerEncoder();
void ISR_S1_L();
void ISR_S1_R();
void motorRightSet();
void vehiculeRotateRight();
void vehiculeRotateLeft();
void motorLeftSet();
void motorRightStop();
void motorLeftStop();
void motorAllStop();
void set_ps2x();
void motor_TELEOP_node_v1();
void just_run();
void measureAndCalibrateMotors();
void runOpMode();



//class for message passing
/*
 * Message structure
 * 
 * topic: what has the message to fo with (string)
 * message: the contents of the message (string)
 * priority: lower priority sent first
 * 
 * Notes
 * 1. priority 0 is only used for cleared messages
*/

// message topics
#define LMOTOR_STATUS 0
#define RMOTOR_STATUS 1
#define COLLISION_SENSOR_STATUS 2
#define CURRENT_OP_MODE 3



class Message {
 public:
 byte topic;
 String msg;
 int priority{0};

 void createMsg(byte t, String m, int p) {
  topic = t;
  msg = m;
  priority = p;
 }

 void clearMsg(void) {
  priority = 0;
 }

};

const byte maxNbrMessages{10};
byte currentMsg{0};
byte indexRing{0};
Message msgRingBuffer[maxNbrMessages];


#endif
