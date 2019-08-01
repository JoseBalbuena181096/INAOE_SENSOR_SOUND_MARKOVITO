#include "sensor_sound.h"
SoundSensorMarkovito SoundSensorM;
void setup() {
  // put your setup code here, to run once:
  SoundSensorM.Init();
}
void loop() {
 // put your main code here, to run repeatedly:
 SoundSensorM.PublishAngleRadiansQuadrants();
 SoundSensorM.RestartSensor();
}
