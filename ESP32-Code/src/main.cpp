#include <Arduino.h>

//Pin definitions

#define MOISTURE_SENSOR1_PIN 4
#define MOISTURE_SENSOR2_PIN 5

#define WATER_LEVEL_SENSOR_PIN 6

#define PUMP_ENABEL_PIN 7

#define SOLENOID1_PIN 15
#define SOLENOID2_PIN 16
#define SOLENOID3_PIN 17
#define SOLENOID4_PIN 18

TaskHandle_t appCommunicationTask;
TaskHandle_t plantSurveillanceTask;


// put function declarations here:

void setup() {
  Serial.begin(115200);

  pinMode(PUMP_ENABEL_PIN, OUTPUT);
  pinMode(SOLENOID1_PIN, OUTPUT);
  pinMode(SOLENOID2_PIN, OUTPUT);
  pinMode(SOLENOID3_PIN, OUTPUT);
  pinMode(SOLENOID4_PIN, OUTPUT);

  xTaskCreatePinnedToCore(
    appCommunicationCode,   /* Task function. */
    "appCommunications",     /* name of task. */
    10000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    1,           /* priority of the task */
    &appCommunicationTask,      /* Task handle to keep track of created task */
    0);          /* pin task to core 0 */ 

  xTaskCreatePinnedToCore(
    plantSurveillanceCode,   /* Task function. */
    "PlantSurveillance",     /* name of task. */
    10000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    1,           /* priority of the task */
    &plantSurveillanceTask,      /* Task handle to keep track of created task */
    1);          /* pin task to core 1 */ 
}

void appCommunicationCode(void *)
{
  return;
}

void plantSurveillanceCode(void *)
{
  return;
}

void loop() {
  // put your main code here, to run repeatedly:
}