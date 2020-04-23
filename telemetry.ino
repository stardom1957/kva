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


