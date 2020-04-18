#ifdef COMPILE_DBWS
/*  
  L298N Motor Control Demonstration with Joystick
  L298N-Motor-Control-Demo-Joystick.ino
  Demonstrates use of Joystick control with Arduino and L298N Motor Controller
  
  DroneBot Workshop 2017
  http://dronebotworkshop.com
*/
  
// Motor A

int ENA_L_PIN = 9;
int IN1_PIN = 8;
int IN2_PIN = 7;

// Motor B

int ENB_R = 3;
int IN3 = 5;
int IN4 = 4;

// motor Joystick Input

#define motorJoyVertY A0 // Vertical is Y
#define motorJoyVertX A1 // Horizontal is X

// Motor Speed Values - Start at zero

int motorSpeed_L = 0;
int motorSpeed_R = 0;

// motor Joystick Values - Start at 512 (middle position)

int motorJoyposVertY = 512;
int motorJoyposHorzX = 512;  

void setup()

{

  // Set all the motor control pins to outputs

  pinMode(ENA_L_PIN, OUTPUT);
  pinMode(ENB_R, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
   
  // Start with motors disabled and direction forward
  
  // Motor A left
  
  digitalWrite(ENA_L_PIN, LOW);
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  
  // Motor B right
  
  digitalWrite(ENB_R, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  
}

void loop() {

  // Read the Joystick X and Y positions

  motorJoyposVertY = analogRead(motorJoyVertY); 
  motorJoyposHorzX = analogRead(motorJoyVertX);

  // Determine if this is a forward or backward motion
  // Do this by reading the Verticle Value
  // Apply results to MotorSpeed and to Direction

  if (motorJoyposVertY < 460)
  {
    // This is Backward

    // Set Motor A backward

    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);

    // Set Motor B backward

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

    //Determine Motor Speeds

    // As we are going backwards we need to reverse readings

    motorJoyposVertY = motorJoyposVertY - 460; // This produces a negative number
    motorJoyposVertY = motorJoyposVertY * -1;  // Make the number positive

    motorSpeed_L = map(motorJoyposVertY, 0, 460, 0, 255);
    motorSpeed_R = map(motorJoyposVertY, 0, 460, 0, 255);

  }
  else if (motorJoyposVertY > 564)
  {
    // This is Forward

    // Set Motor A forward

    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);

    // Set Motor B forward

    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

    //Determine Motor Speeds

    motorSpeed_L = map(motorJoyposVertY, 564, 1023, 0, 255);
    motorSpeed_R = map(motorJoyposVertY, 564, 1023, 0, 255); 

  }
  else
  {
    // This is Stopped

    motorSpeed_L = 0;
    motorSpeed_R = 0; 

  }
  
  // Now do the steering
  // The Horizontal position will "weigh" the motor speed
  // Values for each motor

  if (motorJoyposHorzX < 460)
  {
    // Move Left

    // As we are going left we need to reverse readings

    motorJoyposHorzX = motorJoyposHorzX - 460; // This produces a negative number
    motorJoyposHorzX = motorJoyposHorzX * -1;  // Make the number positive

    // Map the number to a value of 255 maximum

    motorJoyposHorzX = map(motorJoyposHorzX, 0, 460, 0, 255);
        

    motorSpeed_L = motorSpeed_L - motorJoyposHorzX;
    motorSpeed_R = motorSpeed_R + motorJoyposHorzX;

    // Don't exceed range of 0-255 for motor speeds

    if (motorSpeed_L < 0)motorSpeed_L = 0;
    if (motorSpeed_R > 255)motorSpeed_R = 255;

  }
  else if (motorJoyposHorzX > 564)
  {
    // Move Right

    // Map the number to a value of 255 maximum

    motorJoyposHorzX = map(motorJoyposHorzX, 564, 1023, 0, 255);
        

    motorSpeed_L = motorSpeed_L + motorJoyposHorzX;
    motorSpeed_R = motorSpeed_R - motorJoyposHorzX;

    // Don't exceed range of 0-255 for motor speeds

    if (motorSpeed_L > 255)motorSpeed_L = 255;
    if (motorSpeed_R < 0)motorSpeed_R = 0;      

  }


  // Adjust to prevent "buzzing" at very low speed

  if (motorSpeed_L < 8)motorSpeed_L = 0;
  if (motorSpeed_R < 8)motorSpeed_R = 0;

  // Set the motor speeds

  analogWrite(ENA_L_PIN, motorSpeed_L);
  analogWrite(ENB_R, motorSpeed_R);

}
//endif of COMPILE 
#endif
