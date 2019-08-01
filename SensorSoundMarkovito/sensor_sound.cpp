/***Include the sensor_sound.h file******************************************************/
#include "sensor_sound.h"
/********************** Struct Sound Sensor**********************************************/
typedef enum {zero, first, second, third, fourth} state;
struct sensor_sound {
  /*Save the position the sound arrive the pines *************/
  volatile uint8_t arrive[2]={0,0};
  /** Save the time delay to arrive between first and second**/
  volatile unsigned long delay_time = 0;
  /** Save the time delay to arrive sample  first and second**/
  volatile unsigned long delay_time_sample = 0;
  /*** Count interrupt 16 us   *******************************/
  volatile unsigned int timer_counter = 0;
  /***** Count sound arrive **********************************/
  volatile unsigned int counter_sample_time=0;
  /***Enable or disable the sensor sound from Markovito*******/
  int start = 0;
  /************Control the arrive state **********************/
  volatile state State;
  /******Variable to save of the port hirtory ****************/
  volatile uint8_t portbhistory = 0xFF;
  /****Variable to save of the completion of pins samplings***/
  volatile uint8_t finish_sample_pins=0;
} sensor_sound;
/******************* Instances ROS *****************************************************/
/******************* ROS Suscriber *****************************************************/
ros::NodeHandle nh;
/************ CallBack Funtion to Suscriber ********************************************/
void callback_start(const std_msgs::String& start_message){
     sensor_sound.start = 1;
}
ros::Subscriber<std_msgs::String> start_sensor("/isdf/isdf_ss",callback_start);
/******************  ROS Publisher  ****************************************************/
std_msgs::String sound_direction_message;
ros::Publisher sound_direction("/isdf/isdf_pos",&sound_direction_message);
/******************* Funtion Prototype**************************************************/
/***************** Funtion to initialize pines *****************************************/
void init_pins();
/******Funtion configuration of the Pin Change Interrupt(PCI) **************************/
void init_PCI_PortD();
/******Funtion configuration of the Timer2 Interrupt************************************/
void init_TIMER2();
/******This funtion return the relative angle on grades using delay time to arrive *****/
int angle_degrees(unsigned int time_delay_funtion, float distance);
/******This funtion return the relative angle on radians using delay time to arrive ****/
float angle_radians(unsigned int time_delay_funtion, float distance);
/**********This funtion determine the delay time to arrive *****************************/
void  determine_time_delay();
/******This funtion return the absolut angle on radians ********************************/
float determine_angle_radians();
/******This funtion return the absolut angle on radians coord **************************/
float determine_angle_radians_quadrants();
/******This funtion return the absolut angle on degrees ********************************/
int determine_angle_degrees();

/****************************** Class SensorSoundMarkovito *****************************/
/***************** Contructor Class SensorSoundMarkovito *******************************/
SoundSensorMarkovito::SoundSensorMarkovito() {
  angle_markovito_degrees= 0;
  angle_markovito_radians= 0;
  angle_markovito_radians_quadrant=0;
}
/************* Method to  initialize Sensor Sound***************************************/
void SoundSensorMarkovito::Init() {
  init_pins();
  cli();//Stop interruptions
  init_TIMER2();//Initialize Timer 2
  init_PCI_PortD();//Initialize Pin Change Interrupt PortD
  sei();//Enable interruptions
  /****************** Initialize Node **************************************************/
  nh.initNode();
  /****************** Suscriber to "/isdf/isdf_ss" topic *******************************/
  nh.subscribe(start_sensor);
  /****************** Advertise "/isdf/isdf_pos" topic *********************************/
  nh.advertise(sound_direction);
  /****************** Initialize State to zero *****************************************/
  sensor_sound.State = state::zero;
}
/**************** Method to publish the source sound position **************************/
void SoundSensorMarkovito::PublishPosition() {
  if ((sensor_sound.State == state::fourth) && (sensor_sound.start)){
    angle_markovito_degrees = determine_angle_degrees();
    if (angle_markovito_degrees == 0 || angle_markovito_degrees == 360) {
      sound_direction_message.data  = "Front";
    }
    else if ((angle_markovito_degrees > 0) && (angle_markovito_degrees< 90)) {
      sound_direction_message.data  = "Front Left";
    }
    else if (angle_markovito_degrees == 90) {
      sound_direction_message.data  = "Left";
    }
    else if ((angle_markovito_degrees > 90) && (angle_markovito_degrees < 180)) {
      sound_direction_message.data  = "Back left";
    }
    else if (angle_markovito_degrees == 180) {
      sound_direction_message.data  = "Back";
    }
    else if ((angle_markovito_degrees > 180) && (angle_markovito_degrees < 270)) {
     sound_direction_message.data  = "Back Right";
    }
    else if (angle_markovito_degrees== 270) {
      sound_direction_message.data  = "Right";
    }
    else if ((angle_markovito_degrees > 270) && (angle_markovito_degrees < 360)) {
      sound_direction_message.data  = "Front Right";
    }
    sound_direction.publish(&sound_direction_message);
  }
}
/**************** Method to publish the  arrive sound to robot ***********************/
void SoundSensorMarkovito::PublishArrive() {
  /****Publish only if the state is second and control is enable *******/
  if ((sensor_sound.State == state::fourth) && (sensor_sound.start)){
    angle_markovito_degrees = determine_angle_degrees();
    char dataBuffer[20]; /** Char buffer to send data ******************/
    /** Publish the first to arrive ************************************/
    sprintf(dataBuffer, "FIRST: %d", sensor_sound.arrive[0]);
    sound_direction_message.data = dataBuffer;
    sound_direction.publish(&sound_direction_message);
    /** Publish the second to arrive ***********************************/
    sprintf(dataBuffer, "SECOND: %d", sensor_sound.arrive[1]);
    sound_direction_message.data = dataBuffer;
    sound_direction.publish(&sound_direction_message);
    /** Publish the delay time between the first and secont to arrive **/
    sprintf(dataBuffer, "TIME 1-2: %d", sensor_sound.delay_time);
    sound_direction_message.data = dataBuffer;
    sound_direction.publish(&sound_direction_message);
  }
}
/************ Method to publish the sources sound angle position *********************/
void SoundSensorMarkovito::PublishAngleDegrees() {
  /**** Publish only if the state is second and control is enable ******/
  if ((sensor_sound.State == state::fourth) && (sensor_sound.start)) {
    /*Publish the sources sound angle position *************************/
    angle_markovito_degrees = determine_angle_degrees();
    char dataBuffer[10];
    sprintf(dataBuffer,"%d",angle_markovito_degrees);
    sound_direction_message.data = dataBuffer;
    sound_direction.publish(&sound_direction_message);
  }
}
/************ Method to publish the sources sound angle position ********************/
void SoundSensorMarkovito::PublishAngleRadians() {
  /**** Publish only if the state is second and control is enable *******/
  if ((sensor_sound.State == state::fourth) && (sensor_sound.start)) {
    /*Publish the sources sound angle position **************************/
    angle_markovito_radians = determine_angle_radians();
    char float_str[10];
    dtostrf(angle_markovito_radians,4,2,float_str);
    char dataBuffer[10];
    sprintf(dataBuffer,"%s",float_str);
    sound_direction_message.data = dataBuffer;
    sound_direction.publish(&sound_direction_message);
  }
}
/****Method to publish the source sound  radians angles position Reverse*************/ 
void SoundSensorMarkovito::PublishAngleRadiansQuadrants(){
if ((sensor_sound.State == state::fourth) && (sensor_sound.start)){
    angle_markovito_radians_quadrant = determine_angle_radians_quadrants();
    char float_str[10];
    dtostrf(angle_markovito_radians_quadrant,4,2,float_str);
    char dataBuffer[10];
    sprintf(dataBuffer,"%s",float_str);
    sound_direction_message.data = dataBuffer;
    sound_direction.publish(&sound_direction_message);
  }
}
/**************** Method to restart the sensor data *********************************/
void SoundSensorMarkovito::RestartSensor() {
  /****Publish only if the state is second and control is enable **********/
  if ((sensor_sound.State == state::fourth) && (sensor_sound.start)){
    /*********Reset variables**********************************************/
    sensor_sound.State = state::zero;
    sensor_sound.arrive[0] = 0;
    sensor_sound.arrive[1] = 0;
    sensor_sound.delay_time = 0;
    sensor_sound.counter_sample_time= 0;
    sensor_sound.portbhistory = 0xFF;
    sensor_sound.start=0;
    sensor_sound.finish_sample_pins=0;
    /************* Enable pin change interrupts again **********************/
    cli();
    PCMSK2 &= 0b00000000;
    PCMSK2 |= 0b00111100;
    PCICR |=(1 << PCIE2);
    sei();
    delay(1000);
  }
  while(sensor_sound.State != state::fourth){
   if((!sensor_sound.finish_sample_pins)&&((((sensor_sound.timer_counter << 8) + TCNT2) / 16)>10000)){
      TCCR2B &= 0b00000000;
      PCICR &=~(1 << PCIE2);
      sensor_sound.timer_counter=0;
      TCNT2=0;
      sensor_sound.State = state::fourth;
      }
    else if((sensor_sound.finish_sample_pins)&&(PIND & (1 << PD2))&&(PIND & (1 << PD3))&&(PIND & (1 << PD4))&&(PIND & (1 << PD5))){
      PCICR &=~(1 << PCIE2);
      sensor_sound.State = state::fourth;
      }
    nh.spinOnce();
    }
  nh.spinOnce();
}
/**********************Interrupt PIN CHANGE PortD**************************/
ISR(PCINT2_vect) { // handle pin change interrupt for D0 to D7 here
  /**************If any pin D2,D3,D4,D5 change of state********************/
  uint8_t changedbits;
  changedbits = PIND ^ sensor_sound.portbhistory;
  /*************** CHANGE PIN  D2 to LOW **********************************/
  if(sensor_sound.start){
  /***only if D2 is LOW and change state and first to arrive isn't D2 *****/
  if (!(PIND & (1 << PD2)) &&(changedbits&(1<< PIND2))&&(PCMSK2 & (1 << PCINT18))) {
    TCCR2B &= 0b00000000;
    if (sensor_sound.State == state::zero) {//First to arrive is D2
      TCCR2B |= 0b00000001;
      PCMSK2 |= 0b00111100;
      PCMSK2 &= ~(1 << PCINT18);
      TCNT2 = 0;
      sensor_sound.timer_counter = 0;
      sensor_sound.State = state::first;
      if(!sensor_sound.counter_sample_time){
         sensor_sound.arrive[0]=2;
        }
      sensor_sound.finish_sample_pins=0;
    }
    else if (sensor_sound.State == state::first) { //Second to arrive is D2
      TCCR2B |= 0b00000001;
      sensor_sound.delay_time_sample+= ((sensor_sound.timer_counter << 8) + TCNT2) / 16;
      PCMSK2 &= ~(1 << PCINT18);
      sensor_sound.State = state::second;
      if(!sensor_sound.counter_sample_time){
         sensor_sound.arrive[1]=2;
        }
      sensor_sound.counter_sample_time++;
      sensor_sound.finish_sample_pins=0;
    }
    else if (sensor_sound.State == state::second) { //Third to arrive is D2
      TCCR2B |= 0b00000001;
      PCMSK2 &= ~(1 << PCINT18);
      sensor_sound.State = state::third;
      sensor_sound.finish_sample_pins=0;
    }
    else if (sensor_sound.State == state::third) {//Fourth to arrive is D2
      PCMSK2 |= 0b00111100;
      PCMSK2 &= ~(1 << PCINT18);
      sensor_sound.State = state::zero;
      sensor_sound.finish_sample_pins=1;
    }
  }
  /**************CHANGE PIN  D3  to LOW *****************************/
  /*only if D3 is LOW and chanege state and first to arrive isn't D3*/
  if (!(PIND & (1 << PD3)) &&(changedbits&(1<< PIND3))&&(PCMSK2 & (1 << PCINT19))) {
    TCCR2B &= 0b00000000;
    if (sensor_sound.State == state::zero) {//First to arrive is D3
      TCCR2B |= 0b00000001;
      PCMSK2 |= 0b00111100;
      PCMSK2 &= ~(1 << PCINT19);
      TCNT2 = 0;
      sensor_sound.timer_counter = 0;
      sensor_sound.State = state::first;
      if(!sensor_sound.counter_sample_time){
         sensor_sound.arrive[0]=3;
        }
      sensor_sound.finish_sample_pins=0;
    }
    else if (sensor_sound.State == state::first) { //Second to arrive is D3
      TCCR2B |= 0b00000001;
      sensor_sound.delay_time_sample+= ((sensor_sound.timer_counter << 8) + TCNT2) / 16;
      PCMSK2 &= ~(1 << PCINT19);
      sensor_sound.State = state::second;
      if(!sensor_sound.counter_sample_time){
         sensor_sound.arrive[1]=3;
        }
      sensor_sound.counter_sample_time++;
      sensor_sound.finish_sample_pins=0;
    }
    else if (sensor_sound.State == state::second) { //Third to arrive is D3
      TCCR2B |= 0b00000001;
      PCMSK2 &= ~(1 << PCINT19);
      sensor_sound.State = state::third;
      sensor_sound.finish_sample_pins=0;
    }
    else if (sensor_sound.State == state::third) { //Fourth to arrive is D3
      PCMSK2 |= 0b00111100;
      PCMSK2 &= ~(1 << PCINT19);
      sensor_sound.State = state::zero;
      sensor_sound.finish_sample_pins=1;
    }
  }
  /**************CHANGE PIN  D4 to LOW*********************************/
  /**only if D4 is LOW and chanege state and first to arrive isn't D4**/
  if (!(PIND & (1 << PD4)) &&(changedbits&(1<< PIND4))&&(PCMSK2 & (1 << PCINT20))) {
    TCCR2B &= 0b00000000;
    if (sensor_sound.State == state::zero) {//First to arrive is D4
      TCCR2B |= 0b00000001;
      PCMSK2 |= 0b00111100;
      PCMSK2 &= ~(1 << PCINT20);
      TCNT2 = 0;
      sensor_sound.timer_counter = 0;
      sensor_sound.State = state::first;
      if(!sensor_sound.counter_sample_time){
         sensor_sound.arrive[0]=4;
        }
      sensor_sound.finish_sample_pins=0;
    }
    else if (sensor_sound.State == state::first) { //Second to arrive is D4
      TCCR2B |= 0b00000001;
      sensor_sound.delay_time_sample+= ((sensor_sound.timer_counter << 8) + TCNT2) / 16;
      PCMSK2 &= ~(1 << PCINT20);
      sensor_sound.State = state::second;
      if(!sensor_sound.counter_sample_time){
         sensor_sound.arrive[1]=4;
        }
      sensor_sound.counter_sample_time++;
      sensor_sound.finish_sample_pins=0;
    }
    else if (sensor_sound.State == state::second) {//Third to arrive is D4
      TCCR2B |= 0b00000001;
      PCMSK2 &= ~(1 << PCINT20);
      sensor_sound.State = state::third;
      sensor_sound.finish_sample_pins=0;
    }
    else if (sensor_sound.State == state::third) {//Fourth to arrive is D2
      PCMSK2 |= 0b00111100;
      PCMSK2 &= ~(1 << PCINT20);
      sensor_sound.State = state::zero;
      sensor_sound.finish_sample_pins=1;
    }
  }
  /************** CHANGE PIN D5 to LOW *********************************/
  /*only if D5 is LOW abnd chanege state and  first to arrive isn't 5**/
  if (!(PIND & (1 << PD5)) &&(changedbits&(1<< PIND5))&&(PCMSK2 & (1 << PCINT21))) {
    TCCR2B &= 0b00000000;
    if (sensor_sound.State == state::zero) {//First to arrive is D2
      TCCR2B |= 0b00000001;
      PCMSK2 |= 0b00111100;
      PCMSK2 &= ~(1 << PCINT21);
      TCNT2 = 0;
      sensor_sound.timer_counter = 0;
      sensor_sound.State = state::first;
      if(!sensor_sound.counter_sample_time){
         sensor_sound.arrive[0]=5;
        }
      sensor_sound.finish_sample_pins=0;
    }
    else if (sensor_sound.State == state::first) { //Second to arrive is D5
      TCCR2B |= 0b00000001;
      sensor_sound.delay_time_sample+= ((sensor_sound.timer_counter << 8) + TCNT2) / 16;
      PCMSK2 &= ~(1 << PCINT21);
      sensor_sound.State = state::second;
      if(!sensor_sound.counter_sample_time){
         sensor_sound.arrive[1]=5;
        }
      sensor_sound.counter_sample_time++;
      sensor_sound.finish_sample_pins=0;
    }
    else if (sensor_sound.State == state::second) { //Third to arrive is D2
      TCCR2B |= 0b00000001;
      PCMSK2 &= ~(1 << PCINT21);
      sensor_sound.State = state::third;
      sensor_sound.finish_sample_pins=0;
    }
    else if (sensor_sound.State == state::third) { //Fourth to arrive is D2
      PCMSK2 |= 0b00111100;
      PCMSK2 &= ~(1 << PCINT21);
      sensor_sound.State = state::zero;
      sensor_sound.finish_sample_pins=1;
    }
   }
  } 
  sensor_sound.portbhistory = PIND;
}
/*** Timer 2 interrupt funtion******/
ISR(TIMER2_OVF_vect) {
  /**Increase counter every 16 us **/
  sensor_sound.timer_counter++;
}

/***************** Funtion to initialize pines *******************/
void init_pins() {
  /**Set the pins 2,3,4,5 like inputs*******************/
  DDRD &= ~((1 << 2) | (1 << 3) | (1 << 4) | (1 << 5));
  /**Enable the resistors pullup for the pins 2,3,4,5***/
  PORTD |= ((1 << 2) | (1 << 3) | (1 << 4) | (1 << 5));
}
/******Funtion configuration of the Pin Change Interrupt(PCI) ***********/
void init_PCI_PortD() {
  /***** Pin Change Interrupt Control Register for the port D************/
  /****** Activating Pin Change Interrupt portD (PINES D2,D3,D4,D5)******/
  PCMSK2 |= 0b00111100;
  /************* Enable Activating Pin Change Interrupt *****************/
  PCICR |= (1 << PCIE2);
}
/******Funtion configuration of the Timer2 Interrupt*********************/
void init_TIMER2() {
  TCCR2A = 0;
  TCCR2B &= 0b00000000;
  TCNT2  = 0;
  TIMSK2 |= (1 << TOIE2);
}
/******This funtion return the relative angle on grades using delay time to arrive ****/
int angle_degrees(unsigned int time_delay_funtion, float distance) {
  return (int)((asin((((float)time_delay_funtion) * 0.000001 * 343.0000) / distance)) * 180 / PI);
}
/******This funtion return the relative angle on radians using delay time to arrive ****/
float angle_radians(unsigned int time_delay_funtion, float distance) {
  return (float)(asin((((float)time_delay_funtion) * 0.000001 * 343.0000)/ distance));
}
/**********This funtion determine the delay time to arrive *****************************/
void  determine_time_delay() {
  unsigned int time_delay_maximum;
  float time_diference=0;
  time_diference=sensor_sound.delay_time_sample/sensor_sound.counter_sample_time;
  sensor_sound.delay_time=floor(time_diference);
  if ((sensor_sound.arrive[0] == 5 && sensor_sound.arrive[1] == 2) || (sensor_sound.arrive[0] == 2 && sensor_sound.arrive[1] == 5)) {
    time_delay_maximum = (int)(distance_5to2 / (0.000001 * 343));
  }
  else if ((sensor_sound.arrive[0] == 5 && sensor_sound.arrive[1] == 3) || (sensor_sound.arrive[0] == 3 && sensor_sound.arrive[1] == 5)) {
    time_delay_maximum = (int)(distance_5to3 / (0.000001 * 343));
  }
  else if ((sensor_sound.arrive[0] == 4 && sensor_sound.arrive[1] == 2) || (sensor_sound.arrive[0] == 2 && sensor_sound.arrive[1] == 4)) {
    time_delay_maximum = (int)(distance_4to2 / (0.000001 * 343));
  }
  else if ((sensor_sound.arrive[0] == 4 && sensor_sound.arrive[1] == 3) || (sensor_sound.arrive[0] == 3 && sensor_sound.arrive[1] == 4)) {
    time_delay_maximum = (int)(distance_4to3 / (0.000001 * 343));
  }
  if(sensor_sound.delay_time>time_delay_maximum){
    sensor_sound.delay_time=0;
  }
}
/******This funtion return the absolut angle on radians *******************************/
float determine_angle_radians(){
  float angle_radian=0;
  determine_time_delay();
  /*THE FIRST TO ARRIVE IS FRONT PIN*/
  if (sensor_sound.arrive[0] == 5) {
    /*THE SECOND TO ARRIVE IS LEFT PIN*/
    if (sensor_sound.arrive[1] == 2) {
      angle_radian = (2*PI) - angle_radians(sensor_sound.delay_time, distance_5to2);
    }
    /*THE SECOND TO ARRIVE IS RIGH PIN*/
    else if (sensor_sound.arrive[1] == 3) {
      angle_radian  = 0 + angle_radians(sensor_sound.delay_time, distance_5to3);
    }
    else {
      angle_radian  = 0;
    }
  }
  /*THE FIRST TO ARRIVE IS BACK PIN*/
  else if (sensor_sound.arrive[0] == 4) {
    /*THE SECOND TO ARRIVE IS LEFT PIN*/
    if (sensor_sound.arrive[1] == 2) {
       angle_radian = PI + angle_radians(sensor_sound.delay_time, distance_4to2);
    }
    /*THE SECOND TO ARRIVE IS RIGHT PIN*/
    else if (sensor_sound.arrive[1] == 3) {
      angle_radian = PI - angle_radians(sensor_sound.delay_time, distance_4to3);
    }
    else {
      angle_radian = PI;
    }
  }
  /*THE FIRST TO ARRIVE IS RIGHT PIN*/
  else if (sensor_sound.arrive[0] == 3) {
    /*THE SECOND TO ARRIVE IS FRONT PIN*/
    if (sensor_sound.arrive[1] == 5) {
      angle_radian = (PI/2)-angle_radians(sensor_sound.delay_time, distance_5to3);
    }
    /*THE SECOND TO ARRIVE IS BACK PIN*/
    else if (sensor_sound.arrive[1] == 4) {
      angle_radian = (PI/2)+angle_radians(sensor_sound.delay_time, distance_4to3);
    }
    else {
      angle_radian = (PI/2);
    }
  }
  /*THE FIRST TO ARRIVE IS LEFT PIN*/
  else if (sensor_sound.arrive[0] == 2) {
    /*THE SECOND TO ARRIVE IS FRONT PIN*/
    if (sensor_sound.arrive[1] == 5) {
      angle_radian = (3*PI/2)+angle_radians(sensor_sound.delay_time, distance_5to2);
    }
    /*THE SECOND TO ARRIVE IS BACK PIN*/
    else if (sensor_sound.arrive[1] == 4) {
      angle_radian = (3*PI/2)-angle_radians(sensor_sound.delay_time, distance_4to2);
    }
    else {
      angle_radian = (3*PI/2);
    }
  }
  return angle_radian;
}
/******This funtion return the absolut angle on radians coord *******************************/
float determine_angle_radians_quadrants() {
  float angle_radian = 0;
  determine_time_delay();
  /*THE FIRST TO ARRIVE IS FRONT PIN*/
  if (sensor_sound.arrive[0] == 5) {
    /*THE SECOND TO ARRIVE IS LEFT PIN*/
    if (sensor_sound.arrive[1] == 2) {
      angle_radian = 0 - angle_radians(sensor_sound.delay_time, distance_5to2);
    }
    /*THE SECOND TO ARRIVE IS RIGH PIN*/
    else if (sensor_sound.arrive[1] == 3) {
      angle_radian  = 0 + angle_radians(sensor_sound.delay_time, distance_5to3);
    }
    else {
      angle_radian  = 0;
    }
  }
  /*THE FIRST TO ARRIVE IS BACK PIN*/
  else if (sensor_sound.arrive[0] == 4) {
    /*THE SECOND TO ARRIVE IS LEFT PIN*/
    if (sensor_sound.arrive[1] == 2) {
       angle_radian = (0-PI) + angle_radians(sensor_sound.delay_time, distance_4to2);
    }
    /*THE SECOND TO ARRIVE IS RIGHT PIN*/
    else if (sensor_sound.arrive[1] == 3) {
      angle_radian = PI- angle_radians(sensor_sound.delay_time, distance_4to3);
    }
    else {
      angle_radian = PI;
    }
  }
  /*THE FIRST TO ARRIVE IS RIGHT PIN*/
  else if (sensor_sound.arrive[0] == 3) {
    /*THE SECOND TO ARRIVE IS FRONT PIN*/
    if (sensor_sound.arrive[1] == 5) {
      angle_radian = (PI/2)-angle_radians(sensor_sound.delay_time, distance_5to3);
    }
    /*THE SECOND TO ARRIVE IS BACK PIN*/
    else if (sensor_sound.arrive[1] == 4) {
      angle_radian = (PI/2)+angle_radians(sensor_sound.delay_time, distance_4to3);
    }
    else {
      angle_radian = (PI/2);
    }
  }
  /*THE FIRST TO ARRIVE IS LEFT PIN*/
  else if (sensor_sound.arrive[0] == 2) {
    /*THE SECOND TO ARRIVE IS FRONT PIN*/
    if (sensor_sound.arrive[1] == 5) {
      angle_radian = (0-(PI/2))+angle_radians(sensor_sound.delay_time, distance_5to2);
    }
    /*THE SECOND TO ARRIVE IS BACK PIN*/
    else if (sensor_sound.arrive[1] == 4) {
      angle_radian = (0-(PI/2))-angle_radians(sensor_sound.delay_time, distance_4to2);
    }
    else {
      angle_radian = (0-(PI/2));
    }
  }
  return angle_radian;
}
/******This funtion return the absolut angle on degrees *******************************/
int determine_angle_degrees() {
  int angle_degree = 0; 
  determine_time_delay();
  /*THE FIRST TO ARRIVE IS FRONT PIN*/
  if (sensor_sound.arrive[0] == 5) {
    /*THE SECOND TO ARRIVE IS LEFT PIN*/
    if (sensor_sound.arrive[1] == 2) {
      angle_degree = 360 - angle_degrees(sensor_sound.delay_time, distance_5to2);
    }
    /*THE SECOND TO ARRIVE IS RIGH PIN*/
    else if (sensor_sound.arrive[1] == 3) {
      angle_degree  = 0 + angle_degrees(sensor_sound.delay_time, distance_5to3);
    }
    else {
      angle_degree  = 0;
    }
  }
  /*THE FIRST TO ARRIVE IS BACK PIN*/
  else if (sensor_sound.arrive[0] == 4) {
    /*THE SECOND TO ARRIVE IS LEFT PIN*/
    if (sensor_sound.arrive[1] == 2) {
      angle_degree = 180 + angle_degrees(sensor_sound.delay_time, distance_4to2);
    }
    /*THE SECOND TO ARRIVE IS RIGHT PIN*/
    else if (sensor_sound.arrive[1] == 3) {
      angle_degree = 180 - angle_degrees(sensor_sound.delay_time, distance_4to3);
    }
    else {
      angle_degree = 180;
    }
  }
  /*THE FIRST TO ARRIVE IS RIGHT PIN*/
  else if (sensor_sound.arrive[0] == 3) {
    /*THE SECOND TO ARRIVE IS FRONT PIN*/
    if (sensor_sound.arrive[1] == 5) {
      angle_degree = 90-angle_degrees(sensor_sound.delay_time, distance_5to3);
    }
    /*THE SECOND TO ARRIVE IS BACK PIN*/
    else if (sensor_sound.arrive[1] == 4) {
      angle_degree = 90+angle_degrees(sensor_sound.delay_time, distance_4to3);
    }
    else {
      angle_degree = 90;
    }
  }
  /*THE FIRST TO ARRIVE IS LEFT PIN*/
  else if (sensor_sound.arrive[0] == 2) {
    /*THE SECOND TO ARRIVE IS FRONT PIN*/
    if (sensor_sound.arrive[1] == 5) {
      angle_degree = 270+angle_degrees(sensor_sound.delay_time, distance_5to2);
    }
    /*THE SECOND TO ARRIVE IS BACK PIN*/
    else if (sensor_sound.arrive[1] == 4) {
      angle_degree = 270-angle_degrees(sensor_sound.delay_time, distance_4to2);
    }
    else {
      angle_degree = 270;
    }
  }
  return angle_degree;
}
