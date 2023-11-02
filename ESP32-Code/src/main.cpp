#include <Arduino.h>

/*------------------USED PINS------------------*/
// PIN		TYPE		CONNECTED TO
//-----------------------------------------------
// 4		ADC1		Moisture Sensor 1: OUT
// 11		GPIO		Moisture Sensor 1: DIS
// 5		ADC1		Moisture Sensor 2: OUT
// 12		GPIO		Moisture Sensor 2: DIS
// 6		GPIO		Water Level Sensor
// 38		GPIO		Waper pump enable

//Pin definitions
#define WATER_LEVEL_SENSOR_PIN 6

#define PUMP_ENABEL_PIN 7

#define SOLENOID1_PIN 15
#define SOLENOID2_PIN 16
#define SOLENOID3_PIN 17
#define SOLENOID4_PIN 18 

#define NUM_MOIST_SENSORS 2

#define NUM_MOISTURE_SAMPLES 6
#define NUM_WATERLEVEL_SAMPLES 6

#define US_IN_1M 60000*1000


///////////////////////////////////////////////////////////
/*-----------------------VARIABLES-----------------------*/
///////////////////////////////////////////////////////////

bool startup;


//struct for a moisture Sensor containing used µC pins and last read value
struct plant {
	uint8_t sensorPin;
	uint8_t disablePin;
	uint8_t valvePin;
	uint16_t reading;
	uint16_t lowerLimit;
	uint16_t upperLimit;
};


//struct for a Water Level Sensor containing used µC pin and last read value
struct waterLevelSensor {
	uint8_t sensorPin;
	bool wtrLvlLow; //Sensor switch can be closed (water level low) or open
};


struct waterPump {
	uint8_t enablePin;
};


TaskHandle_t appCommunicationTask;
TaskHandle_t plantSurveillanceTask;
SemaphoreHandle_t xNewPlantData;  //Used for synchronization of sensor data between tasks
SemaphoreHandle_t xNewWaterLvlData;  //Used for synchronization of sensor data between tasks
SemaphoreHandle_t xNewAppCommands;  //Used for synchronization app commands between tasks

//Array of all moisture sensors
plant plants[NUM_MOIST_SENSORS] = {
  {4, 11},
  {5, 13}
};

waterLevelSensor waterLvlSensor = {6};

waterPump pump = {38};

uint64_t intervalMoistureCheck; //how often to check plant moisture


///////////////////////////////////////////////////////////
/*-----------------------FUNCTIONS-----------------------*/
///////////////////////////////////////////////////////////

void readMoistSensor(plant s)
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

void readEveryMoistSensor(plant s[])
{
	for(int i = 0; i<NUM_MOIST_SENSORS; i++)
	{
		readMoistSensor(s[i]);
	}
}


void readWaterLvlSensor(waterLevelSensor s)
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


void wateringPlant (waterPump w, plant p, waterLevelSensor s)
{	
	digitalWrite(w.enablePin, HIGH); //Enable pump
	
	//Wait until moisture upper limit is reached or water is empty
	while ((p.reading < p.upperLimit) && !s.wtrLvlLow)
	{
		readWaterLvlSensor(s); //update current water level
		readMoistSensor(p); // update current moisture level 	
	}

	digitalWrite(w.enablePin, LOW); // disable Pump
}


void initFlash()
{
	
}


///////////////////////////////////////////////////////////
/*----------------------MAIN TASKS-----------------------*/
///////////////////////////////////////////////////////////

void plantSurveillanceCode(void *)
{
	uint64_t timeLastMoistureCheck = esp_timer_get_time();
	

	while(true)
	{
		// Check if appCommunications task signaled new commands from smartphone App
		if(xSemaphoreTake(xNewAppCommands, 100 / portTICK_PERIOD_MS) || startup) // 100/portTICK_PERIOD_MS means this function checks over and over again for 100ms
		{
			//reload all settings from flash
		}

		//Check if moisture sensors need to be read
		if ( (esp_timer_get_time() - timeLastMoistureCheck) >= intervalMoistureCheck * US_IN_1M || startup)
		{
			readEveryMoistSensor(plants); // Read and update all moisture sensors
			xSemaphoreGive(xNewPlantData); 
			timeLastMoistureCheck = esp_timer_get_time(); // Set time for next check

			readWaterLvlSensor(waterLvlSensor);

			//Check if water is available for watering of plants
			if(!waterLvlSensor.wtrLvlLow)
			{
				//Check for every moisture sensor if plant needs to be watered
				for(int i = 0; i < NUM_MOIST_SENSORS; i++)
				{
					//If given moisture level is below allowed range -> water plant
					if(plants[i].reading < plants[i].lowerLimit)
					{
						wateringPlant(pump, plants[i], waterLvlSensor);
					}
				}
			}
			else
			{
				xSemaphoreGive(xNewWaterLvlData); // Let other task know that new Sensor dada is available
			}
			
		}

		startup = false;
	}
}

void appCommunicationCode(void *)
{
	while(true)
	{
		//Check if PlantSurveillance signaled new available sensor data
		if(xSemaphoreTake(xNewPlantData, 100 / portTICK_PERIOD_MS)) // 100/portTICK_PERIOD_MS means this function checks over and over again for 100ms
		{

		}
	}

	return;
}


///////////////////////////////////////////////////////////
/*-------------------------SETUP-------------------------*/
///////////////////////////////////////////////////////////

void setup() {
	startup = true;

	Serial.begin(115200);
	delay(1000); //Take some time to open up the Serial Monitor

	//Set pin modes
	pinMode(plants[0].disablePin, OUTPUT);
	pinMode(plants[1].disablePin, OUTPUT);
	pinMode(waterLvlSensor.sensorPin, INPUT);
	pinMode(PUMP_ENABEL_PIN, OUTPUT);
	pinMode(SOLENOID1_PIN, OUTPUT);
	pinMode(SOLENOID2_PIN, OUTPUT);
	pinMode(SOLENOID3_PIN, OUTPUT);
	pinMode(SOLENOID4_PIN, OUTPUT);

	analogSetAttenuation(ADC_11db); //enables ADC measurement range of 0V-3.3V
	//analogReadResolution(12);



	xNewPlantData = xSemaphoreCreateBinary();
	xNewWaterLvlData = xSemaphoreCreateBinary();
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