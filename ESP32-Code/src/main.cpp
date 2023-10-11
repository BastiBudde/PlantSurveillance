#include <Arduino.h>


#define MOISTURE_SENSOR1_PIN 4
#define MOISTURE_SENSOR2_PIN 5

#define WATER_LEVEL_SENSOR_PIN 6

#define PUMP_ENABEL_PIN 7

#define SOLENOID1_PIN 15
#define SOLENOID2_PIN 16
#define SOLENOID3_PIN 17
#define SOLENOID4_PIN 18


// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}