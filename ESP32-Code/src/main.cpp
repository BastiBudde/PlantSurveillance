#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WiFiMulti.h>


/*------------------USED PINS------------------*/
// PIN		TYPE		CONNECTED TO
//-----------------------------------------------
// 4		ADC1		Moisture Sensor 1: OUT
// 11		GPIO		Moisture Sensor 1: DIS
// 5		ADC1		Moisture Sensor 2: OUT
// 12		GPIO		Moisture Sensor 2: DIS
// 6		GPIO		Water Level Sensor
// 38		GPIO		Waper pump enable

///////////////////////////////////////////////////////////
/*------------------------DEFINES------------------------*/
///////////////////////////////////////////////////////////
#define DEBUG

// WIFI credentials
#define SSID "WLAN-E2E567"
#define PASSWORD "6529446175875462"
// #define MAC_ADRESS "34-85-18-4A-64-70" //Mac adress of uC

#define PORT 8088 //Port that uC is listening to

#define WATER_LEVEL_SENSOR_PIN 6

#define PUMP_ENABEL_PIN 7

#define SOLENOID1_PIN 15
#define SOLENOID2_PIN 16
#define SOLENOID3_PIN 17
#define SOLENOID4_PIN 18 

#define NUM_MOIST_SENSORS 4

#define NUM_MOISTURE_SAMPLES 6
#define NUM_WATERLEVEL_SAMPLES 6

#define US_IN_1M 60000000 // 1 Minute


///////////////////////////////////////////////////////////
/*-----------------------VARIABLES-----------------------*/
///////////////////////////////////////////////////////////

bool startup;
Preferences preferences;

//struct for a moisture Sensor containing used µC pins and last read value
struct plant {
	uint8_t sensorPin;
	uint8_t disablePin;
	uint8_t valvePin;      // For Valve
	uint16_t reading;
	uint16_t lowerLimit;
	uint16_t upperLimit;
	bool exists;
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
  {5, 13},
  {},
  {}
};

//Struct containing the Water level Sensor
waterLevelSensor waterLvlSensor = {6};

//Struct containing the water pump
waterPump pump = {38};


uint64_t intervalCheck; //how often to check plant moisture

//For wifi connection
WiFiMulti wiFiMulti; // NOT WiFiMulti but wiFiMulti otherwise error

// Use WiFiClient class to create TCP connections
WiFiServer server(PORT);
WiFiClient client;


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


void initSettings()
{
	preferences.begin("settings", false);

	if(!preferences.getULong64("intervalCheck", 0))
	{
		preferences.putULong64("intervalCheck", 30 * US_IN_1M); //default Value: 30 Minutes
	}

	//Plant 1 ----------------------------------------------------------------------------------
	if(preferences.getBool("Plant1Exists", 0) == 0){
		preferences.putBool("Plant1Exists", 0); //default Value: Plant doesn't exist
	}

	if(preferences.getUShort("Plant1Upper", 0) == 0){
		preferences.putUShort("Plant1Upper", 100); //default Value: 100% of soilmoiusture
	}
	
	if(preferences.getUShort("Plant1Lower", 101) == 101){
		preferences.putUShort("Plant1Lower", 0); //default Value: 0% of soilmoiusture
	}

	//Plant 2 ----------------------------------------------------------------------------------
	if(preferences.getBool("Plant2Exists", 0) == 0){
		preferences.putBool("Plant2Exists", 0); //default Value: Plant doesn't exist
	}

	if(preferences.getUShort("Plant2Upper", 0) == 0){
		preferences.putUShort("Plant2Upper", 100); //default Value: 100% of soilmoiusture
	}
	
	if(preferences.getUShort("Plant2Lower", 101) == 101){
		preferences.putUShort("Plant2Lower", 0); //default Value: 0% of soilmoiusture
	}
	
	//Plant 3 ----------------------------------------------------------------------------------
	if(preferences.getBool("Plant3Exists", 0) == 0){
		preferences.putBool("Plant3Exists", 0); //default Value: Plant doesn't exist
	}

	if(preferences.getUShort("Plant3Upper", 0) == 0){
		preferences.putUShort("Plant3Upper", 100); //default Value: 100% of soilmoiusture
	}
	
	if(preferences.getUShort("Plant3Lower", 101) == 101){
		preferences.putUShort("Plant3Lower", 0); //default Value: 0% of soilmoiusture
	}

	//Plant 4 ----------------------------------------------------------------------------------
	if(preferences.getBool("Plant4Exists", 0) == 0){
		preferences.putBool("Plant4Exists", 0); //default Value: Plant doesn't exist
	}

	if(preferences.getUShort("Plant4Upper", 0) == 0){
		preferences.putUShort("Plant4Upper", 100); //default Value: 100% of soilmoiusture
	}
	
	if(preferences.getUShort("Plant4Lower", 101) == 101){
		preferences.putUShort("Plant4Lower", 0); //default Value: 0% of soilmoiusture
	}
	
	preferences.end();
	
}

void loadSettings()
{
	preferences.begin("settings", false);

	intervalCheck = preferences.getULong64("intervalCheck", 0);
	//plant 1 ----------------------------------------------------------------------------------
	plants[1].exists = preferences.getBool("Plant1Exists", 0);
	plants[1].upperLimit = preferences.getUShort("Plant1Upper", 0);
	plants[1].lowerLimit = preferences.getUShort("Plant1Lower", 101);

	//plant 2 ----------------------------------------------------------------------------------
	plants[2].exists = preferences.getBool("Plant2Exists", 0);
	plants[2].upperLimit = preferences.getUShort("Plant2Upper", 0);
	plants[2].lowerLimit = preferences.getUShort("Plant2Lower", 101);

	//plant 3 ----------------------------------------------------------------------------------
	plants[3].exists = preferences.getBool("Plant3Exists", 0);
	plants[3].upperLimit = preferences.getUShort("Plant3Upper", 0);
	plants[3].lowerLimit = preferences.getUShort("Plant3Lower", 101);
	
	//plant 4 ----------------------------------------------------------------------------------
	plants[4].exists = preferences.getBool("Plant4Exists", 0);
	plants[4].upperLimit = preferences.getUShort("Plant4Upper", 0);
	plants[4].lowerLimit = preferences.getUShort("fourthlantLowerLimit", 101);
	
	preferences.end();
}

void setInterval(uint64_t value)
{
	preferences.begin("settings", false);
	preferences.putULong64("intervalCheck", value * US_IN_1M); // value * 1 Minute
	preferences.end();
}

void setPlant(char* limits)
{
	uint64_t plantCom[4];
	char* buffer;
	const char delimeter = ',';

	preferences.begin("settings", false);

	// Breaking down Data from App
	buffer = strtok(limits, &delimeter);

	switch(atoi(buffer))
	{
		case 1:	preferences.putUShort("Plant1Upper", atoi(strtok(NULL, &delimeter)));
				preferences.putUShort("Plant1Lower", atoi(strtok(NULL, &delimeter)));
				preferences.putBool("Plant1Exists", atoi(strtok(NULL, &delimeter)));
				break;
				
		case 2:	preferences.putUShort("Plant2Upper", atoi(strtok(NULL, &delimeter))); 
				preferences.putUShort("Plant2Lower", atoi(strtok(NULL, &delimeter)));
				preferences.putBool("Plant2Exists", atoi(strtok(NULL, &delimeter)));  
				break;
				
		case 3: preferences.putUShort("Plant3Upper", atoi(strtok(NULL, &delimeter)));
				preferences.putUShort("Plant3Lower", atoi(strtok(NULL, &delimeter)));
				preferences.putBool("Plant3Exists", atoi(strtok(NULL, &delimeter)));
				break;
				
		case 4: 
				preferences.putUShort("Plant4Upper", atoi(strtok(NULL, &delimeter)));
				preferences.putUShort("Plant4Lower", atoi(strtok(NULL, &delimeter)));
				preferences.putBool("Plant4Exists", atoi(strtok(NULL, &delimeter)));
				break;

		default: Serial.println("Error");
				 break;
	}
	
	preferences.end();
}

void evalMessage(String msg)
{
	const char delimeter1 = ':';
	char* command;
	uint64_t interval = 0;
	char* limits;
	
	// "plant:1,upperLimit,lowerLimit,exists"
	// "plant:2,upperLimit,lowerLimit,exists"
	// "plant:3,upperLimit,lowerLimit,exists"
	// "plant:4,upperLimit,lowerLimit,exists"
	// "interval:value"

	command = strtok((char*)msg.c_str(), &delimeter1); // separate Data of App

	if(strcmp(command, "interval") == 0)
	{	
		interval = atoi(strtok(NULL, &delimeter1));
		setInterval(interval);
	}
	else if(strcmp(command, "plant") == 0)
	{
		limits = strtok(NULL, &delimeter1);
		setPlant(limits);
	}
	else
	{
		Serial.println("Error in receiving Data");
	}
	
	
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
			loadSettings();
		}

		//Check if moisture sensors need to be read
		if ( (esp_timer_get_time() - timeLastMoistureCheck) >= intervalCheck * US_IN_1M || startup)
		{
			timeLastMoistureCheck = esp_timer_get_time(); // Set time for next check

			readEveryMoistSensor(plants); // Read and update all moisture sensors
			readWaterLvlSensor(waterLvlSensor);
			
			xSemaphoreGive(xNewPlantData);

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
	int64_t checkStart = esp_timer_get_time();


	while(true)
	{
		//Check if PlantSurveillance signaled new available sensor data
		if(xSemaphoreTake(xNewPlantData, 100 / portTICK_PERIOD_MS)) // 100/portTICK_PERIOD_MS means this function checks over and over again for 100ms
		{
			// Send Data if plant exists
			for (int i = 0; i<NUM_MOIST_SENSORS; i++)
			{
				if(plants[i].exists)
				{
					client.printf("plant:%d,%d", &i, &plants[i].reading);
				}
			}

			if(waterLvlSensor.wtrLvlLow)
			{
				client.printf("waterLevelLow:%d", 1);
			}
			else
			{
				client.printf("waterLevelLow:%d", 0);
			}
		}

		checkStart = esp_timer_get_time();
		while((esp_timer_get_time() - checkStart) <= 100000)
		{	
			// Saving available Data
			if (client.available() > 0)
			{
				//read back one line from the server
				String line = client.readStringUntil('\r');
				
				#ifdef DEBUG
				Serial.println(line);
				#endif
				
				evalMessage(line);
				xSemaphoreGive(xNewAppCommands);
			}
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
	Serial.printf("Serial started!\n");

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
	
	
	initSettings();
	loadSettings();


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

	// connecting to a WiFi network
	wiFiMulti.addAP(SSID, PASSWORD);

	Serial.println();
	Serial.println();
	Serial.print("Waiting for WiFi... \n");

	while(wiFiMulti.run() != WL_CONNECTED) {
		Serial.print(".");
		delay(500);
	}

	Serial.println("");
	Serial.println("WiFi connected\n");
	Serial.println("IP address: \n");
	Serial.println(WiFi.localIP());


	// while (!client.connect(HOST, PORT)) {
	//     Serial.println("Connection failed.\n");
	//     Serial.println("Waiting 5 seconds before retrying...\n");
	//     delay(1000);
	// }

	/*----------Following code is just to test TCP/IP communication----------*/

	server.begin(); //uC is now able to receive messages

	uint8_t data[30]; //for incomming data. max. 30 bytes

	while(1)
	{
		client = server.available();	// Create client from received message (possible because messages 
										// allways contain IP-Adress from sender)
		if (client) //if new client available
		{
			Serial.println("new client");
			
			/* check client is connected */
			while (client.connected())
			{
				if (client.available()) //Check if new message from client is available
				{
					/*Read message and print it*/
					int len = client.read(data, 30);
					if(len < 30)
					{
						data[len] = '\0';
					}else
					{
						data[30] = '\0';
					}
					Serial.print("client sent: ");
					Serial.println((char *)data);
					client.printf("Recieved!");
				}
			} 
		}
	}
    
}


void loop() {
}