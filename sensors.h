#ifndef _sensors_h
#define _sensors_h
#ifdef SENSORS_COMPILE

#define LEFT_CONTACT_SENSOR_PIN 30
#define CENTER_CONTACT_SENSOR_PIN 32
#define RIGHT_CONTACT_SENSOR_PIN 34

volatile bool contact_sensor_just_triggered{false};
volatile byte contact_sensors_ID{0};
volatile bool state_of_emergency{false}; // indicates state of emergency in effect

//this function will serve as a base for sensor devel when needed
void sensorDeveloppement(void) {
  ; // do noting for now
}

// contact sensors isr
/*
Contact sensor IDs binary (decimal)
100 (4) left
010 (2) center
001 (1) right
*/
#define LEFT_CONTACT_TRIGGERED 4
#define CENTER_CONTACT_TRIGGERED 2
#define RIGHT_CONTACT_TRIGGERED 1

// function prototypes
void isr_left_contact_sensor(void);
void isr_center_contact_sensor(void); 
void isr_right_contact_sensor(void); 

volatile u_short nbr_of_contact{0}; // count of number of interrupt detected
void isr_left_contact_sensor(void) {
    ++nbr_of_contact;
    contact_sensor_just_triggered = true;
    state_of_emergency = true;
    contact_sensors_ID |= LEFT_CONTACT_TRIGGERED; // set appr bit
}

void isr_center_contact_sensor(void) {
    ++nbr_of_contact;
    state_of_emergency = true;
    contact_sensor_just_triggered = true;
    contact_sensors_ID |= CENTER_CONTACT_TRIGGERED; // set appr bit
}

void isr_right_contact_sensor(void) {
    //NVIC_DisableIRQ(PIOC_IRQn);
    ++nbr_of_contact;
    contact_sensor_just_triggered = true;
    state_of_emergency = true;
    contact_sensors_ID |= RIGHT_CONTACT_TRIGGERED; // set appr bit
}

#endif //SENSORS_COMPILE
#endif
