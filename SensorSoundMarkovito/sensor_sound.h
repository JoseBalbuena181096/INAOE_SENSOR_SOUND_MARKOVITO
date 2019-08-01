/************************************************************
   Autor: Jose Angel Balbuena Palma.
   Puporse: This code was created for the detection
            of the sound source direction, with four
            microphones and arduino one to the service
            robot Markovito.
   Notes: The pins used for the inputs are the
                  at the pin 5 front sensor,
                  at the pin 4 back sensor,
                  at the pin 3 left sensor,
                  at the pin 2 right sensor.
           The logic state of the sensor is low,
           when this sensor is hit for a sound wave and
           is high when sensor isn't hit.
  Date: 30/05/2019.
**********************************************************/
#ifndef SENSOR_SOUND_H
#define SENSOR_SOUND_H
/********************** Libraries ************************/
#include <stdlib.h>//standard library 
#include <math.h>// math library 
/******************* AVR Libraries ***********************/
#include <avr/io.h>/** Library of avr I/O pins**/
#include <avr/interrupt.h>/*Library of avr interrupts*/
/******************* ROS Libraries ***********************/
#include<ros.h>/** Ros library **/
#include<std_msgs/String.h>/** Library String messages**/
/*********************************************************/
#define distance_5to2 0.1805/*** Distance between the sensor 5 and sensor 2 ****/
#define distance_5to3 0.1805/*** Distance between the sensor 5 and sensor 3 ****/
#define distance_4to2 0.155/*** Distance between the sensor 4 and sensor 2 ****/
#define distance_4to3 0.159/*** Distance between the sensor 4 and sensor 3 ****/
class SoundSensorMarkovito
{
  public:
    SoundSensorMarkovito();/***Contructor Class SensorSoundMarkovito ****/
    void Init();/*Method to initialize the sound sensor*/
    void RestartSensor();/*Method to restart the sensor data*/
    void PublishPosition();/*Method to publish the source sound position*/
    void PublishArrive();/*Method to publish the  arrive sound to robot*/
    void PublishAngleDegrees();/*Method to publish the source sound degrees angles position*/
    void PublishAngleRadians();/*Method to publish the source sound  radians angles position*/
    void PublishAngleRadiansQuadrants();/*Method to publish the source sound  reverse radians angles position*/
  private:
    int angle_markovito_degrees;
    float angle_markovito_radians;
    float angle_markovito_radians_quadrant;
};
#endif
