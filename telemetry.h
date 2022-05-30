#ifndef _telemetry_h
#define _telemetry_h

#ifdef TELEMETRY

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
     
     The . are made from sent/received as Strings from the original
     data types by the sender. The (type) provided can be used for data
     conversions if used other than for display, i.e. calculations. See
     paragraph C.
     
  C. TOPIC PROCESSING
     Each topic will have it's own function for processing.
*/

/*
 * This function sends data to the control station and receive data from it.
 * 
 * Communication is acheived through an Xbee
 */

 /*
  //debug interrupts(); // allow interrupt starting here
  // debug
  // create somme messages
  //createTopic(byte tID, String dat, char typ, byte prio)
  //1
  currentTopic = storeTopic(CURRENT_OP_MODE, String(SENSORS_DEVELOPEMENT), 'b', 2);
  //2
  delay(3500);
  currentTopic = storeTopic(COLLISION_SENSOR_STATUS, "5", 'b', 1);
  //3
  delay(500);
  currentTopic = storeTopic(RMOTOR_SPEED, String(7824), 'u', 4);
  //4
  delay(50);
  currentTopic = storeTopic(LMOTOR_SPEED, String(5824), 'u', 4);
  //5
  delay(15);
  currentTopic = storeTopic(COLLISION_SENSOR_STATUS, "4", 'b', 1);
  //6
  delay(102);
  currentTopic = storeTopic(RMOTOR_SPEED, String(3824), 'u', 4);
  //7
  delay(152);
  currentTopic = storeTopic(LMOTOR_SPEED, String(3823), 'u', 4);
  //8 
  delay(254);
  currentTopic = storeTopic(COLLISION_SENSOR_STATUS, "0", 'b', 1);
  //9
  delay(82);
  currentTopic = storeTopic(LMOTOR_SPEED, String(11823), 'u', 4);
  //10
  delay(204);
  currentTopic = storeTopic(COLLISION_SENSOR_STATUS, "0", 'b', 1);
 //11 overflow 1
  delay(24);
  currentTopic = storeTopic(COLLISION_SENSOR_STATUS, "1", 'b', 1);
  //12 overflow 2
  delay(43);
  currentTopic = storeTopic(COLLISION_SENSOR_STATUS, "7", 'b', 1);
  //13 overflow 3
  delay(82);
  currentTopic = storeTopic(LMOTOR_SPEED, String(11823), 'u', 4);
  //14 overflow 4
  delay(204);
  currentTopic = storeTopic(COLLISION_SENSOR_STATUS, "0", 'b', 1);
  //15 overflow 5
  delay(24);
  currentTopic = storeTopic(COLLISION_SENSOR_STATUS, "1", 'b', 1);
  //16 overflow 6
  delay(43);
  currentTopic = storeTopic(COLLISION_SENSOR_STATUS, "7", 'b', 1);
*/
 
/*
 delay(5000);
 //send telemetry and clear buffer
 sendTelemetry(SERIAL_TELEMETRY_PORT);

 // then create more topics and print them
 
 Serial.println("\nTwo more\n");
 delay(1000);
 currentTopic = storeTopic(COLLISION_SENSOR_STATUS, "0", 'b', 1);
 delay(5);
 currentTopic = storeTopic(COLLISION_SENSOR_STATUS, "7", 'b', 1);
 //send telemetry and clear buffer
 sendTelemetry(SERIAL_TELEMETRY_PORT);

 Serial.println("\nThere should be nothing after this\n");
 delay(1000);
*/
/*
 Serial.println(topicsRingBuffer[currentTopic].topicID);
 Serial.println(topicsRingBuffer[currentTopic].stc);
 Serial.println(topicsRingBuffer[currentTopic].dataType);
 Serial.println(topicsRingBuffer[currentTopic].priority);
 Serial.println();
*/  
// topics
#define MOTORS_STATUS_TOPIC           0 //bit mapped to byte value
#define LMOTOR_SPEED_TOPIC            1 // unsigned int
#define RMOTOR_SPEED_TOPIC            2 // unsigned int
#define COLLISION_SENSOR_STATUS_TOPIC 3 // bit mapped to byte value
#define CURRENT_OP_MODE_TOPIC         4

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
}; // end class Topic

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

#endif //if defined TELEMETRY
#endif
