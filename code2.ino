#ifdef COMPILE_CODE2

/* snippet use for setting up buttons hooked up to interrupts
   but not a good idea if hardware debouncing is use, that include capacitors
   see 7.4 and Annex A
*/

//motor directions and speed
#define FORWARD 0
#define REVERSE 1
#define STOP    2
#define MOTORSPEED 200

// buttons that control motor direction
// each trigger an interrupt when pressed=HIGH
#define btnGoForward 40
#define btnGoReverse 41
#define btnSTOP 23

volatile byte direction = STOP; // start motor at STOP

void ISR_forward(void) {
 direction = FORWARD;
}

void ISR_reverse(void) {
 direction = REVERSE;
}

void ISR_stop(void) {
 direction = STOP;
}

void setup() {
  // snippet use for setting up buttons hooked up to interrupts
  //but not a good idea if hardware debouncing is use, that include capacitors
  pinMode(btnGoForward, INPUT);
  pinMode(btnGoReverse, INPUT);
  pinMode(btnSTOP, INPUT);

  attachInterrupt(digitalPinToInterrupt(btnGoForward), ISR_forward, RISING);
  attachInterrupt(digitalPinToInterrupt(btnGoReverse), ISR_reverse, RISING);
  attachInterrupt(digitalPinToInterrupt(btnSTOP), ISR_stop, CHANGE);
}

void demoOne() {
  motorRightForward(255);
  motorLeftForward(255);
  delay(3000);
  motorAllStop();
  delay(3000);
  
  motorRightReverse(255);
  motorLeftReverse(255);
  delay(3000);
  motorAllStop();
  delay(3000);
}

void demoTwo()
{
  // this function will run the motors across the range of possible speeds
  // note that maximum speed is determined by the motor itself and the operating voltage
  // the PWM values sent by analogWrite() are fractions of the maximum speed possible
  
  for (int v = 100; v <= 255; v++)
  {
    motorRightForward(v);
    motorLeftForward(v);
    delay(20);
  }

  delay(2000);
  for (int v = 255; v > 100; v--)
  {
    motorRightForward(v);
    motorLeftForward(v);
    delay(20);
  }
  delay(2000);
  
  //en arrière
  for (int v = 100; v <= 255; v++)
  {
    motorRightReverse(v);
    motorLeftReverse(v);
    delay(20);
  }

  delay(2000);
  for (int v = 255; v > 100; v--)
  {
    motorRightReverse(v);
    motorLeftReverse(v);
    delay(20);
  }
  delay(2000);
}


byte dir;
void loop()
{
   dir = direction;
 switch (dir) {
  case FORWARD:
    //motorAllStop;
    //delay(200);
    //Serial.println("FORWARD");
    noInterrupts();
    motorRightForward(MOTORSPEED);
    motorLeftForward(MOTORSPEED);
    interrupts();
    break;

  case REVERSE:
    //motorAllStop;
    //delay(200);
    //Serial.println("REVERSE");
    noInterrupts();
    motorRightReverse(MOTORSPEED);
    motorLeftReverse(MOTORSPEED);
    interrupts();
    break;

  case STOP:
    //Serial.println("STOP");
    noInterrupts();
    motorAllStop();
    interrupts();break;
  
  default:
    //Serial.println("DEFAULT = STOP");
    noInterrupts();
    motorAllStop();
    interrupts();
    break;
 }

}

void motorRightForward(int speed) {
  if (speed >= 100 && speed <= 255) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB_R, speed); //why is this line prevents encoderTimer from working? 
  }
}

void motorRightReverse(int speed) {
  if (speed >= 100 && speed <= 255) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB_R, speed);
  }
}

void motorLeftForward(int speed) {
  if (speed >= 100 && speed <= 255) {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
    analogWrite(ENA_L_PIN, speed);
  }
}

void motorLeftReverse(int speed) {
  if (speed >= 100 && speed <= 255) {
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
    analogWrite(ENA_L_PIN, speed);
  }
}

#endif
