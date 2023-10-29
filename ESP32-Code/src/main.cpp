#include <Arduino.h>

/*------------------USED PINS------------------*/
// PIN		TYPE		CONNECTED TO
//-----------------------------------------------
// 4		ADC1		Moisture Sensor 1: OUT
// 11		GPIO		Moisture Sensor 1: DIS
// 5		ADC1		Moisture Sensor 2: OUT
// 12		GPIO		Moisture Sensor 2: DIS
// 6		GPIO		Water Level Sensor

//Pin definitions
#define WATER_LEVEL_SENSOR_PIN 6

#define PUMP_ENABEL_PIN 7

#define SOLENOID1_PIN 15
#define SOLENOID2_PIN 16
#define SOLENOID3_PIN 17
#define SOLENOID4_PIN 18 

#define NUM_MOIST_SENSRS 2

#define NUM_MOISTURE_SAMPLES 6
#define NUM_WATERLEVEL_SAMPLES 6

#define US_IN_1M 60000*1000


///////////////////////////////////////////////////////////
/*-----------------------VARIABLES-----------------------*/
///////////////////////////////////////////////////////////

struct moistureSensor {
  uint8_t sensorPin;
  uint8_t disablePin;
  uint16_t reading;
};

//Array of struct moistureSensors that should contain every moisture sensor and its used pins on the µC dev board
moistureSensor moistSensrs[NUM_MOIST_SENSRS] = {
  {4, 11},
  {5, 13}
};


struct waterLevelSensor {
	uint8_t sensorPin;
	bool wtrLvlLow; //Sensor switch can be closed (water level low) or open
};

waterLevelSensor wtrLvlSensr = {6};


TaskHandle_t appCommunicationTask;
TaskHandle_t plantSurveillanceTask;
SemaphoreHandle_t xNewSensorData;  //Used for synchronization of sensor data between tasks
SemaphoreHandle_t xNewAppCommands;  //Used for synchronization app commands between tasks


///////////////////////////////////////////////////////////
/*-----------------------FUNCTIONS-----------------------*/
///////////////////////////////////////////////////////////

void readMoistSensr(moistureSensor s)
{
	digitalWrite(s.disablePin, LOW); //Enable power for moisture sensor to be read
	delay(100);

	//Take multiple samples of the voltage reading and take the average
	s.reading = 0;
	for(int i=0; i<NUM_MOISTURE_SAMPLES; i++)
	{
		s.reading += analogRead(s.sensorPin);
		delay(10);
	}
	s.reading /= NUM_MOISTURE_SAMPLES;

	digitalWrite(s.disablePin, HIGH); //Disable power for moisture sensor to be read
	return;
}


void readWtrLvlSensr(waterLevelSensor s)
{
	//Read sensor multiple times in case sensor is just at the tipping point
	for (int i = 0; i < NUM_WATERLEVEL_SAMPLES; i++)
	{
		if(digitalRead(s.sensorPin) == LOW){
			s.wtrLvlLow = false;
		}
		else if(digitalRead(s.sensorPin) == HIGH)
		{
			s.wtrLvlLow = true;
			return;
		}
	}
	
	return;
}

///////////////////////////////////////////////////////////
/*----------------------MAIN TASKS-----------------------*/
///////////////////////////////////////////////////////////

void plantSurveillanceCode(void *)
{
	uint64_t timeLastMoistureCheck = esp_timer_get_time();
	uint64_t timeBetweenMoistureChecks; // In flash speicher?

	while(true)
	{
		//Check if appCommunications signaled new commands from smartphone App
		if(xSemaphoreTake(xNewAppCommands, 100 / portTICK_PERIOD_MS)) // 100/portTICK_PERIOD_MS means this function checks over and over again for 100ms
		{
			
		}

		//Read measurement (Timer)
		if ( (esp_timer_get_time() - timeLastMoistureCheck) >= US_IN_1M * timeBetweenMoistureChecks )
		{
			//Read and update all moisture sensors
			for(int i = 0; i<NUM_MOIST_SENSRS; i++)
			{
				readMoistSensr(moistSensrs[i]);
			}

			readWtrLvlSensr(wtrLvlSensr); // Waterlevel Sensor
			
			//Let other task know that new Sensor dada is available
			xSemaphoreGive(xNewSensorData);
			timeLastMoistureCheck = esp_timer_get_time(); // Set time for next check
		}

	}

	return;
}

void appCommunicationCode(void *)
{
	while(true)
	{
		//Check if PlantSurveillance signaled new available sensor data
		if(xSemaphoreTake(xNewSensorData, 100 / portTICK_PERIOD_MS)) // 100/portTICK_PERIOD_MS means this function checks over and over again for 100ms
		{

		}
	}

	return;
}


///////////////////////////////////////////////////////////
/*-------------------------SETUP-------------------------*/
///////////////////////////////////////////////////////////

void setup() {

	Serial.begin(115200);
	delay(1000); //Take some time to open up the Serial Monitor

	//Set pin modes
	pinMode(moistSensrs[0].disablePin, OUTPUT);
	pinMode(moistSensrs[1].disablePin, OUTPUT);
	pinMode(WATER_LEVEL_SENSOR_PIN, INPUT);
	pinMode(PUMP_ENABEL_PIN, OUTPUT);
	pinMode(SOLENOID1_PIN, OUTPUT);
	pinMode(SOLENOID2_PIN, OUTPUT);
	pinMode(SOLENOID3_PIN, OUTPUT);
	pinMode(SOLENOID4_PIN, OUTPUT);

	analogSetAttenuation(ADC_11db); //enables ADC measurement range of 0V-3.3V
	//analogReadResolution(12);


	xNewSensorData = xSemaphoreCreateBinary();
	xNewAppCommands = xSemaphoreCreateBinary();

	

	//Create task for plant surveillance (sensor readings/evaluations, watering)
	xTaskCreatePinnedToCore(
		plantSurveillanceCode,   /* Task function. */
		"PlantSurveillance",     /* name of task. */
		10000,       /* Stack size of task */
		NULL,        /* parameter of the task */
		1,           /* priority of the task */
		&plantSurveillanceTask,      /* Task handle to keep track of created task */
		1);          /* pin task to core */ 

	//Create task for communications with smatrphone app
	xTaskCreatePinnedToCore(
		appCommunicationCode,   /* Task function. */
		"appCommunications",     /* name of task. */
		10000,       /* Stack size of task */
		NULL,        /* parameter of the task */
		1,           /* priority of the task */
		&appCommunicationTask,      /* Task handle to keep track of created task */
		0);          /* pin task to core */ 
}


void loop() {
}