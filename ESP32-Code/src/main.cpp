#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WiFiMulti.h>


/*------------------USED PINS------------------*/
// PIN		TYPE		CONNECTED TO
//-----------------------------------------------
// 4		ADC1		Moisture Sensor 1: OUT
// 5		ADC1		Moisture Sensor 2: OUT
// 6		ADC1		Moisture Sensor 3: OUT
// 7		ADC1		Moisture Sensor 4: OUT
// 15		GPIO		Moisture Sensor 1: DIS
// 16		GPIO		Moisture Sensor 2: DIS
// 17		GPIO		Moisture Sensor 1: DIS
// 18		GPIO		Moisture Sensor 2: DIS
// 37		GPIO		Water Level Sensor
// 2		GPIO		Waper pump enable

///////////////////////////////////////////////////////////
/*------------------------DEFINES------------------------*/
///////////////////////////////////////////////////////////
#define DEBUG

// WIFI credentials
#define SSID "WLAN-E2E567"
#define PASSWORD "6529446175875462"
// #define MAC_ADRESS "34-85-18-4A-64-70" //Mac adress of uC

#define PORT 8088 //Port that uC is listening to

// To select used moisture sensor
#define MOISTURE_AZDELIVERY
//#define MOISTURE_CRYTON

#define WATER_LEVEL_SENSOR_PIN 6

#define PUMP_ENABEL_PIN 7

#define SOLENOID1_PIN 15
#define SOLENOID2_PIN 16
#define SOLENOID3_PIN 17
#define SOLENOID4_PIN 18

#define NUM_PLANTS 4

#define NUM_MOISTURE_SAMPLES 3

#ifdef DEBUG
	#define US_IN_1MIN 10000000 // 1 Minute
#else
	#define US_IN_1MIN 60000000 // 1 Minute
#endif

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
	uint16_t rawADC;
	uint8_t reading;
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
TaskHandle_t consolePrintTask;
SemaphoreHandle_t xNewPlantData;    //Used for synchronization of sensor data between tasks
SemaphoreHandle_t xNewWaterLvlData; //Used for synchronization of sensor data between tasks
SemaphoreHandle_t xNewAppCommands;  //Used for synchronization app commands between tasks
SemaphoreHandle_t xNewConsolePrint; //Used for synchronizing printing data in console



//Array of all moisture sensors
plant plants[NUM_PLANTS] = {
  {4, 15},
  {5, 16},
  {6, 17},
  {7, 18}
};

//Struct containing the Water level Sensor
waterLevelSensor waterLvlSensor = {37};

//Struct containing the water pump
waterPump pump = {2};


uint64_t intervalCheck; //how often to check plant moisture

//For wifi connection
WiFiMulti wiFiMulti; // NOT WiFiMulti but wiFiMulti otherwise error

// Use WiFiClient class to create TCP connections
WiFiServer server(PORT);
WiFiClient client;


///////////////////////////////////////////////////////////
/*-----------------------FUNCTIONS-----------------------*/
///////////////////////////////////////////////////////////

double fmap(double x, double in_min, double in_max, double out_min, double out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint8_t rawADCtoMois(uint16_t rawADC)
{
	#if defined(MOISTURE_CRYTON)
		if((rawADC/4096.0) * 3.3 < 1.3)
		{
			return 0;
		}
		else
		{
			//return 100 - 100*((((rawADC-1737.6969) / 4095.0) * 3.3) / (2.3-1.4));
			return 100 - ((rawADC - 1737.6969) * (100 - 0) / (2854.7878 - 1737.6969));
			return (int)fmap((double)rawADC, 2854.7878, 1737.6969, 0.0, 100.0);
		}

	#elif defined(MOISTURE_AZDELIVERY)
		//return 100 - 100*(rawADC / 4095.0);
		return (int)fmap( (double)rawADC, 4095.0, 1106.0, 0.0, 100.0);	// Measurment in water: 1106 Analog reading, Measurment in dry soil: 4095 Analog reading 
																		// Mapped to parcentage between 0 and 100
	#endif
}

void readMoistSensor(plant* s)
{
	digitalWrite((*s).disablePin, LOW); //Enable power for moisture sensor to be read
	delay(600);
	uint64_t rawADC = 0;

	//Take multiple samples of the voltage reading and take the average
	for(int i=0; i<NUM_MOISTURE_SAMPLES; i++)
	{
		rawADC += analogRead((*s).sensorPin);
		delay(50);
	}
	//Serial.printf("Reading Gesamt: %d\n", readings / NUM_MOISTURE_SAMPLES);

	rawADC = rawADC / NUM_MOISTURE_SAMPLES;
	(*s).rawADC = (uint16_t)rawADC;
	(*s).reading = rawADCtoMois( (uint16_t)rawADC );

	digitalWrite((*s).disablePin, HIGH); //Disable power for moisture sensor to be read
	return;
}

void readEveryMoistSensor(plant s[])
{
	for(int i = 0; i<NUM_PLANTS; i++)
	{
		if(s[i].exists)
		{
			readMoistSensor(&s[i]);
		}
	}
}


void readWaterLvlSensor(waterLevelSensor* s)
{
	(*s).wtrLvlLow = !digitalRead((*s).sensorPin);
	return;
}


void wateringPlant (waterPump* w, plant* p, waterLevelSensor* s)
{
	digitalWrite((*w).enablePin, LOW); //Enable pump

	//Wait until moisture upper limit is reached or water is empty
	while (((*p).reading < (*p).upperLimit) && !(*s).wtrLvlLow)
	{
		readWaterLvlSensor(s); //update current water level
		readMoistSensor(p); // update current moisture level
	}

	if((*s).wtrLvlLow)
	{
		Serial.println("Ran out of water :(");
	}

	if((*p).reading >= (*p).upperLimit)
	{
		Serial.println("Watering finished :)");
	}

	digitalWrite((*w).enablePin, HIGH); // disable Pump
}


void initSettings(boolean reset = false)
{
	preferences.begin("settings", false);

	if(!preferences.getULong64("intervalCheck", 0)  || reset)
	{
		preferences.putULong64("intervalCheck", 30); //default Value: 30 Minutes
	}

	//Plant 1 ----------------------------------------------------------------------------------
	if(preferences.getBool("Plant1Exists", 0) == 0  || reset){
		preferences.putBool("Plant1Exists", 0); //default Value: Plant doesn't exist
	}

	if(preferences.getUShort("Plant1Upper", 0) == 0  || reset){
		preferences.putUShort("Plant1Upper", 100); //default Value: 100% of soilmoiusture
	}

	if(preferences.getUShort("Plant1Lower", 101) == 101  || reset){
		preferences.putUShort("Plant1Lower", 0); //default Value: 0% of soilmoiusture
	}

	//Plant 2 ----------------------------------------------------------------------------------
	if(preferences.getBool("Plant2Exists", 0) == 0  || reset){
		preferences.putBool("Plant2Exists", 0); //default Value: Plant doesn't exist
	}

	if(preferences.getUShort("Plant2Upper", 0) == 0  || reset){
		preferences.putUShort("Plant2Upper", 100); //default Value: 100% of soilmoiusture
	}

	if(preferences.getUShort("Plant2Lower", 101) == 101  || reset){
		preferences.putUShort("Plant2Lower", 0); //default Value: 0% of soilmoiusture
	}

	//Plant 3 ----------------------------------------------------------------------------------
	if(preferences.getBool("Plant3Exists", 0) == 0  || reset){
		preferences.putBool("Plant3Exists", 0); //default Value: Plant doesn't exist
	}

	if(preferences.getUShort("Plant3Upper", 0) == 0  || reset){
		preferences.putUShort("Plant3Upper", 100); //default Value: 100% of soilmoiusture
	}

	if(preferences.getUShort("Plant3Lower", 101) == 101  || reset){
		preferences.putUShort("Plant3Lower", 0); //default Value: 0% of soilmoiusture
	}

	//Plant 4 ----------------------------------------------------------------------------------
	if(preferences.getBool("Plant4Exists", 0) == 0  || reset){
		preferences.putBool("Plant4Exists", 0); //default Value: Plant doesn't exist
	}

	if(preferences.getUShort("Plant4Upper", 0) == 0  || reset){
		preferences.putUShort("Plant4Upper", 100); //default Value: 100% of soilmoiusture
	}

	if(preferences.getUShort("Plant4Lower", 101) == 101  || reset){
		preferences.putUShort("Plant4Lower", 0); //default Value: 0% of soilmoiusture
	}

	preferences.end();

}

void loadSettings()
{
	preferences.begin("settings", false);

	intervalCheck = preferences.getULong64("intervalCheck", 0);
	//plant 1 ----------------------------------------------------------------------------------
	plants[0].exists = preferences.getBool("Plant1Exists", 0);
	plants[0].upperLimit = preferences.getUShort("Plant1Upper", 0);
	plants[0].lowerLimit = preferences.getUShort("Plant1Lower", 101);

	//plant 2 ----------------------------------------------------------------------------------
	plants[1].exists = preferences.getBool("Plant2Exists", 0);
	plants[1].upperLimit = preferences.getUShort("Plant2Upper", 0);
	plants[1].lowerLimit = preferences.getUShort("Plant2Lower", 101);

	//plant 3 ----------------------------------------------------------------------------------
	plants[2].exists = preferences.getBool("Plant3Exists", 0);
	plants[2].upperLimit = preferences.getUShort("Plant3Upper", 0);
	plants[2].lowerLimit = preferences.getUShort("Plant3Lower", 101);

	//plant 4 ----------------------------------------------------------------------------------
	plants[3].exists = preferences.getBool("Plant4Exists", 0);
	plants[3].upperLimit = preferences.getUShort("Plant4Upper", 0);
	plants[3].lowerLimit = preferences.getUShort("Plant4Lower", 101);

	preferences.end();
}

void printSettings()
{
	Serial.printf("\n------------------------------------------------------------------------------------------------\n");
	Serial.printf("intervalCheck: %d \n", intervalCheck);
	for(int i = 0; i < NUM_PLANTS; i++)
	{
		Serial.printf("Plant %d:	upperLimit: %3d,	lowerLimit: %3d,	exists: %d \n", i+1, plants[i].upperLimit, plants[i].lowerLimit, plants[i].exists);
	}
	Serial.printf("------------------------------------------------------------------------------------------------\n\n");
}

void setInterval(uint64_t value)
{
	preferences.begin("settings", false);
	preferences.putULong64("intervalCheck", value); // value * 1 Minute
	preferences.end();
}


void setPlant(char* limits)
{
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

		case 3:	preferences.putUShort("Plant3Upper", atoi(strtok(NULL, &delimeter)));
				preferences.putUShort("Plant3Lower", atoi(strtok(NULL, &delimeter)));
				preferences.putBool("Plant3Exists", atoi(strtok(NULL, &delimeter)));
				break;

		case 4:	preferences.putUShort("Plant4Upper", atoi(strtok(NULL, &delimeter)));
				preferences.putUShort("Plant4Lower", atoi(strtok(NULL, &delimeter)));
				preferences.putBool("Plant4Exists", atoi(strtok(NULL, &delimeter)));
				break;

		default: Serial.printf("Invalid Plant Index: %d", buffer);
				 break;
	}

	preferences.end();
}

boolean evalMessage(String msg)
{
	const char* msg_c = msg.c_str();
	const char delimeter1 = ':';
	uint64_t interval = 0;
	char* limits;

	// "plant:1,upperLimit,lowerLimit,exists"
	// "plant:2,upperLimit,lowerLimit,exists"
	// "plant:3,upperLimit,lowerLimit,exists"
	// "plant:4,upperLimit,lowerLimit,exists"
	// "interval:value"
	strtok((char*)msg_c, &delimeter1); // separate Data of App

	//Serial.println(command);
	if(strcmp(msg_c, "interval") == 0)
	{
		interval = atoi(strtok(NULL, &delimeter1));
		setInterval(interval);
		return true;
	}
	else if(strcmp(msg_c, "plant") == 0)
	{
		limits = strtok(NULL, &delimeter1);
		setPlant(limits);
		return true;
	}
	else if(strcmp(msg_c, "ping") == 0)
	{
		return false;
	}
	else
	{
		Serial.println("Error in receiving Data");
		return false;
	}

}

void sendPlantData()
{
	Serial.printf("\n");
	// Send Data if plant exists
	for (int i = 0; i<NUM_PLANTS; i++)
	{
		if(plants[i].exists)
		{
			client.printf("plant:%d,%d\n", i+1, plants[i].reading);
			Serial.printf("Data send to app: \"plant:%d,%d\"\n", i+1, plants[i].reading);
		}
	}

}

void sendWaterLvLData()
{
	if(waterLvlSensor.wtrLvlLow)
	{
		client.printf("waterLevelLow:%d\n", 1);
	}
	else
	{
		client.printf("waterLevelLow:%d\n", 0);
	}
}

void printSensorData()
{
	Serial.printf("------------------------\n");
	Serial.println("Sensors read:");
	for (int i = 0; i<NUM_PLANTS; i++)
	{
		if(plants[i].exists)
		{
			Serial.printf("    Plant-%d moist.: %d %% (raw: %d)\n", i+1, plants[i].reading, plants[i].rawADC);
		}
	}
	Serial.printf("    Waterlevel: %s\n\n", waterLvlSensor.wtrLvlLow==true ? "low" : "high");
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
			#ifdef DEBUG
				printSettings();
			#endif
		}

		//Check if moisture sensors need to be read
		if ( (esp_timer_get_time() - timeLastMoistureCheck) >= intervalCheck * US_IN_1MIN || startup)
		{
			timeLastMoistureCheck = esp_timer_get_time(); // Set time for next check

			readEveryMoistSensor(plants); // Read and update all moisture sensors
			readWaterLvlSensor(&waterLvlSensor);
			printSensorData();
			//Serial.printf("Waterlevel low: %d\n", waterLvlSensor.wtrLvlLow);

			// Let other task know that new Sensor data is available
			xSemaphoreGive(xNewPlantData);
			xSemaphoreGive(xNewWaterLvlData);

			//Check if water is available for watering of plants
			if(!waterLvlSensor.wtrLvlLow)
			{
				//Check for every moisture sensor if plant needs to be watered
				for(int i = 0; i < NUM_PLANTS; i++)
				{
					//If given moisture level is below allowed range -> water plant
					if(plants[i].reading < plants[i].lowerLimit)
					{
						Serial.println("Watering Plants :)");
						wateringPlant(&pump, &plants[i], &waterLvlSensor);
					}
					else
					{
						Serial.println("Moisture OK :)");
					}
				}
			}
			else
			{
				Serial.println("Not enough Water :(");
			}
		}

		startup = false;
		vTaskDelay(1); //To let other freeRTOS tasks heve some CPU time
	}
}

void appCommunicationCode(void *)
{
	String line;
	boolean newMessage = false;

	while(true)
	{
		//Check if PlantSurveillance signaled new available sensor data
		if(xSemaphoreTake(xNewPlantData, 50 / portTICK_PERIOD_MS)) // 50/portTICK_PERIOD_MS means this function checks over and over again for 50ms
		{
			sendPlantData();
		}

		if(xSemaphoreTake(xNewWaterLvlData, 50 / portTICK_PERIOD_MS))
		{
			sendWaterLvLData();
		}


		/*Check if client still connected and wait for new one if not*/
		if(!client.connected())
		{
			Serial.println("Client is disconnected");
			client.stop();

			Serial.println("Waiting for client...");
			client = server.available(); // Create client from received message (possible because messages allways contain IP-Adress from sender)
			while(!client) //Wait until a client is connected
			{
				client = server.available();
				delay(100);
				vTaskDelay(1); //To let other freeROTS tasks heve some CPU time
			}
			Serial.print("New Client connected: ");
			Serial.print(client.localIP());
			Serial.print(":");
			Serial.println(client.localPort());

			//Greet new client and send newest data
			client.println("Connected!");
			sendPlantData();
			sendWaterLvLData();
		}


		/* check if client still connected */
		if (client.connected())
		{
			while(client.available()) //Check if new message from client is available
			{
				line = client.readStringUntil('\n'); //read back one line from the server

				//Print message to console
				#ifdef DEBUG
					Serial.printf("client sent:  ");
					Serial.printf("\"");
					Serial.printf(line.c_str()); // String to char array
					Serial.printf("\"\n");

					//client.println("Recieved!");
				#endif

				//Evaluate message
				newMessage = evalMessage(line);
			}

			if(newMessage)
			{
				xSemaphoreGive(xNewAppCommands); //let other task know new commands from App were send
				newMessage = false;
			}
		}

		vTaskDelay(1); //To let other freeROTS tasks heve some CPU time
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
	Serial.println("Serial started!");

	//Set pin modes (All pins input by default)
	pinMode(plants[0].disablePin, OUTPUT);
	pinMode(plants[1].disablePin, OUTPUT);
	pinMode(plants[2].disablePin, OUTPUT);
	pinMode(plants[3].disablePin, OUTPUT);
	pinMode(waterLvlSensor.sensorPin, INPUT);
	gpio_pulldown_en((gpio_num_t)waterLvlSensor.sensorPin);
	pinMode(pump.enablePin, OUTPUT);

	digitalWrite(pump.enablePin, HIGH); //Disable Pump
	digitalWrite(plants[0].disablePin, HIGH);
	digitalWrite(plants[1].disablePin, HIGH);
	digitalWrite(plants[2].disablePin, HIGH);
	digitalWrite(plants[3].disablePin, HIGH);

	// pinMode(SOLENOID1_PIN, OUTPUT);
	// pinMode(SOLENOID2_PIN, OUTPUT);
	// pinMode(SOLENOID3_PIN, OUTPUT);
	// pinMode(SOLENOID4_PIN, OUTPUT);

	analogSetAttenuation(ADC_11db); //enables ADC measurement range of 0V-3.3V
	//analogReadResolution(12); // Bit-Resolution


	xNewPlantData = xSemaphoreCreateBinary();
	xNewWaterLvlData = xSemaphoreCreateBinary();
	xNewAppCommands = xSemaphoreCreateBinary();


	initSettings();
	loadSettings();


	//Create task for plant surveillance (sensor readings/evaluations, watering)
	xTaskCreatePinnedToCore(
		plantSurveillanceCode,   /* Task function. */
		"plantSurveillance",     /* name of task. */
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
	Serial.println();
	Serial.println();
	Serial.println("Waiting for WiFi...");

	WiFi.begin(SSID, PASSWORD);

	while(!WiFi.isConnected())
	{
		delay(100);
	}

	Serial.println("");
	Serial.println("WiFi connected");
	Serial.print("IP address: ");
	Serial.println(WiFi.localIP());
	server.begin(); //uC is now able to receive messages via TCP/IP

}


void loop() {
	vTaskDelete(NULL); //Deletes task for this loop() funktion
}