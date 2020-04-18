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
void just_do_this();
void free_run();
void measureAndCalibrateMotors();
void standby();
void runOpMode();
String assembleMessage();
byte storeTopic();
void sendTelemetry();
void setGPIOs();
void updateDisplayAndIndicators();
void manageOpModeTimer();
void strToChar();

//class for message passing
/*
 A. MESSAGE SYNTAX
    Syntax is : "topicID[stc]data(dataType)", where
     topicID is the message subject
     [stc] is the system time code
     data is the message data contents
     (dataType) is the type of the data:
      i integer
      u unsigned integer
      f float
      t text
      b byte
      d word 16-bit
      w word 32-bit

  B. MESSAGES CONSTRUCTION
     A message is constructed from a Topic class where all members are converted
     to a series of strings.
     
     The . are made fromsent/received as Strings from the original
     data types by the sender. The (type) provided can be used for data
     conversions if used other than for display, i.e. calculations. See
     paragraph C.
     
  C. TOPIC PROCESSING
     Each topic will have it's own function for processing.
*/

// topics
#define MOTORS_STATUS 0 //bit mapped to byte value
#define LMOTOR_SPEED 1 // unsigned int
#define RMOTOR_SPEED 2 // unsigned int
#define COLLISION_SENSOR_STATUS 3 // bit mapped to byte value
#define CURRENT_OP_MODE 4

class Topic {
 public:
  byte topicID;
  unsigned long stc;
  String data;
  char dataType;
  byte priority;

 Topic(void) {priority = 0;}

 void createTopic(byte tID, String dat, char typ, byte prio) {
  topicID = tID;
  data = dat;
  dataType = typ;
  priority = prio;
  stc = millis();
 }

 void clearTopic(void) {
  priority = 0; // indicates message has been processed / sent
                // witch also indicates that the object os free
 }
};

// assemble a complete message (for telemetry) from a Topic
String assembleMessage(Topic t) {
  String tempo;
  tempo = String(t.topicID);
  tempo += "[";
  tempo += String(t.stc);
  tempo += "]";
  tempo += t.data;
  tempo += "(";
  tempo += t.dataType;
  tempo += ")";
  return tempo;
}

const byte maxNbrTopics{10}; //buffer size
byte currentTopic{0};
unsigned int topicsOverflow{0}; // incremented everytime topicsRingBuffer is full
//byte indexRing{0};
Topic topicsRingBuffer[maxNbrTopics];


byte storeTopic(byte tID, String dat, char type, byte prio) {
  // check for next available entry in buffer
  byte e;
  for (e = 0; e < maxNbrTopics; e++) {
    if (topicsRingBuffer[e].priority == 0) {
      break;
    }
  }

  // create topic in entry found createTopic(byte tID, String dat, char typ, byte prio)
  if (e < maxNbrTopics) {
    topicsRingBuffer[e].createTopic(tID, dat, type, prio);
  }
  else {
    ++topicsOverflow; // at least one message was lost!
  }
  return e;
}


// will have to be recursive
void sendTelemetry(byte port) {
 // find the number of topics to send in buffer (priority <> 0)
 byte nbr{0};
 byte t;
 for (t=0; t < maxNbrTopics; t++) {
    if (topicsRingBuffer[t].priority != 0) {
      ++nbr;
     }
  }
  
  if (nbr != 0) { // there is something to send
    unsigned long smallest{4294967295};// for stc, max number for millis()
    unsigned long ckeckThisOne;
    byte indexTopic{0};
  
    // find smallest stc in buffer
    for (t=0; t < maxNbrTopics; t++) {
      if (topicsRingBuffer[t].priority != 0) {
         ckeckThisOne = topicsRingBuffer[t].stc;
         if (ckeckThisOne < smallest) {
            smallest = ckeckThisOne;
            indexTopic = t;
         }
       }
     }

    // send the smallest found ...
    if (port == SERIAL_TELEMETRY_PORT) { //send to telemetry port
     Serial1.println(assembleMessage(topicsRingBuffer[indexTopic])); 
    }
    else { // send to serial terminal (debug port)
      Serial.println(assembleMessage(topicsRingBuffer[indexTopic]));
     }

     // debug wait for acknoledge 3 times.
     // what to do if ...
     // ... and clear it
     topicsRingBuffer[indexTopic].clearTopic();
     // check buffer again, recursively
     sendTelemetry(byte (1));
   } //end if
 return;
}

#endif
