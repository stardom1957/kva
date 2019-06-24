#ifdef DUE_TIMER
#include <DueTimer.h>

volatile boolean timesUp = false;
void myHandler(void){
  timesUp = true;
}

DueTimer myTimer = Timer.getAvailable();
void setup(){
  Serial.begin(9600);
  delay(2000);
  Serial.println("Début de test.");
  //if (myTimer != DueTimer(0))
  myTimer.setPeriod(1000000); // 1 000 000 us, set period to 1 s
  myTimer.attachInterrupt(myHandler);
}

unsigned int t{0};
void loop(){
  myTimer.start();
  while(!timesUp){
  }
  myTimer.stop();
  timesUp=false;
  ++t;
  Serial.println(t, DEC);
}
#endif
