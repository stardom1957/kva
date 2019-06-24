#ifdef PID_COMPILE
const byte S1motorEncoder_L = 22;  // motor encoder S1 A
const byte S2motorEncoder_L = 23;  // motor encoder S2 B
const byte S1motorEncoder_R = 24;  // motor encoder S1 S1
const byte S2motorEncoder_R = 25;  // motor encoder S2 S2

// begin put this in setup
attachInterrupt(digitalPinToInterrupt(S1motorEncoder_L), ISR_S1_L, CHANGE);
attachInterrupt(digitalPinToInterrupt(S1motorEncoder_R), ISR_S1_R, CHANGE);


// end put this in setup

volatile unsigned long S1_L_count {0};
volatile unsigned long S1_R_count {0};
void ISR_S1_L(void) {
  ++S1_L_count;
}
void ISR_S1_R(void) {
  ++S1_R_count;
}

#endif
