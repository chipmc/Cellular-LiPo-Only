/*
* Project Environmental Sensor - Soil Sensor that can go an entire season on one set of LiPo batteries
* Description: Cellular Connected Boron with soil and environmental sensors powered by 18650 LiPo batteries for 150 days
* Method of operation - device will be awakened by a TPL5111 on the EN pin, will connect in Setup and will measure and report in loop
* Measurements will be reposrted via Webhook to Ubidots.  Once a response is received, the device will signal the TPL5111 to bring the EN pin low
* Sensor details ....
* Author: Chip McClelland chip@seeinsights.com
* Sponsor: Colorado State University
* License: GPL v3
* Date: 31- May 2019
*/

// v1.00 - Initial Release - Rough program outline
// v1.01 - Updated Readme and added GPL v3 license
// v1.02 - Added a flag that limits Webhooks to once per wake cycle
// v1.02a - a better way - moved to STATE rather than the send function
// v1.03 - Added a time valid check
// v1.04 - Added a step to store current minute in EEPROM
// v1.05 - Simplified the way we check for time to report
// v1.06 - Changed stayAwakeLong to 25 sec

#define SOFTWARERELEASENUMBER "1.05"               // Keep track of release numbers

// Included Libraries
// Add libraries for sensors here

// Initalize library objects here

// Define the memory map - note can be EEPROM or FRAM
namespace MEM_MAP {                                 // Moved to namespace instead of #define to limit scope
  enum Addresses {
    versionAddr           = 0x0,                    // Where we store the memory map version number - 8 Bits
    alertCountAddr        = 0x1,                    // Where we store our current alert count - 8 Bits
    resetCountAddr        = 0x2,                    // This is where we keep track of how often the Electron was reset - 8 Bits
    timeZoneAddr          = 0x3,                    // Store the local time zone data - 8 Bits
    controlRegisterAddr   = 0x4,                    // This is the control register for storing the current state - 8 Bits
    currentCountsTimeAddr = 0x5,                    // Time of last report - 32 bits
  };
};

#define MEMORYMAPVERSION 1                          // Lets us know if we need to reinitialize the memory map

// Prototypes and System Mode calls
SYSTEM_MODE(SEMI_AUTOMATIC);                        // This will enable user code to start executing automatically.
SYSTEM_THREAD(ENABLED);                             // Means my code will not be held up by Particle processes.
STARTUP(System.enableFeature(FEATURE_RESET_INFO));


// State Machine Variables
enum State { INITIALIZATION_STATE, ERROR_STATE, IDLE_STATE, MEASURING_STATE, REPORTING_STATE, RESP_WAIT_STATE, SLEEPING_STATE, LOW_BATTERY_STATE};
char stateNames[8][14] = {"Initialize", "Error", "Idle", "Measuring", "Reporting", "Response Wait", "Sleeping", "Low Battery"};
State state = INITIALIZATION_STATE;
State oldState = INITIALIZATION_STATE;

// Pin Constants
const int blueLED =       D7;                       // This LED is on the Electron itself

volatile bool watchDogFlag = false;                 // Flag is raised in the watchdog ISR

// Timing Variables
const int wakeBoundary = 0*3600 + 5*60 + 0;         // 0 hour 5 minutes 0 seconds
const unsigned long stayAwakeLong = 25000;          // In lowPowerMode, how long to stay awake every hour
const unsigned long webhookWait = 45000;            // How long will we wair for a WebHook response
const unsigned long resetWait = 30000;              // How long will we wait in ERROR_STATE until reset
const int publishFrequency = 1000;                  // We can only publish once a second
unsigned long stayAwakeTimeStamp = 0;               // Timestamps for our timing variables..
unsigned long stayAwake;                            // Stores the time we need to wait before napping
unsigned long webhookTimeStamp = 0;                 // Webhooks...
unsigned long resetTimeStamp = 0;                   // Resets - this keeps you from falling into a reset loop


// Program Variables
int resetCount;                                     // Counts the number of times the Electron has had a pin reset
int alertCount;                                     // Keeps track of non-reset issues - think of it as an indication of health
bool ledState = LOW;                                // variable used to store the last LED status, to toggle the light
bool readyForBed = false;                           // Checks to see if steps for sleep have been completed
bool waiting = false;
bool dataInFlight = true;
const char* releaseNumber = SOFTWARERELEASENUMBER;  // Displays the release on the menu
byte controlRegister;                               // Stores the control register values
bool verboseMode;                                   // Enables more active communications for configutation and setup

// Variables Related To Particle Mobile Application Reporting
char SignalString[64];                     // Used to communicate Wireless RSSI and Description
const char* radioTech[8] = {"Unknown","None","WiFi","GSM","UMTS","CDMA","LTE","IEEE802154"};
char soilMoisture1String[16];
char soilMoisture2String[16];
char soilMoisture3String[16];
char soilMoisture4String[16];
char soilMoisture5String[16];
char soilMoisture6String[16];
char precipitationCountString[16];
char soilTempInCString[16];
char humidityString[16];
char temperatureString[16];
char panelHumidityString[16];
char panelTemperatureString[16];
char batteryString[16];


// Time Period Related Variables
time_t currentCountTime;                            // Global time vairable
byte currentMinutePeriod;                           // control timing when using 5-min samp intervals

// Battery monitoring
float lowBattLimit=3.0;                             // Trigger for Low Batt State - LiPo voltage
bool lowPowerMode;                                  // Flag for Low Power Mode operations

// This section is where we will initialize sensor specific variables, libraries and function prototypes
float soilMoisture1;
float soilMoisture2;
float soilMoisture3;
float soilMoisture4;
float soilMoisture5;
float soilMoisture6;
int precipitationCount;
float soilTempInC;
float humidity;
float temperature;
float panelHumidity;
float panelTemperature;
float batteryVoltage;
int rat;
float strengthPercentage;
float qualityPercentage;

void setup()                                                      // Note: Disconnected Setup()
{
  char StartupMessage[64] = "Startup Successful";                 // Messages from Initialization
  state = IDLE_STATE;

  pinMode(blueLED, OUTPUT);                                       // declare the Blue LED Pin as an output

  char responseTopic[125];
  String deviceID = System.deviceID();                            // Multiple Electrons share the same hook - keeps things straight
  deviceID.toCharArray(responseTopic,125);
  Particle.subscribe(responseTopic, UbidotsHandler, MY_DEVICES);  // Subscribe to the integration response event

  Particle.variable("Signal", SignalString);                      // Particle variables that enable monitoring using the mobile app
  Particle.variable("ResetCount", resetCount);
  Particle.variable("Release",releaseNumber);
  Particle.variable("stateOfChg", batteryString);
  Particle.variable("lowPowerMode",lowPowerMode);
  Particle.variable("temperature", temperatureString);
  Particle.variable("humidity", humidityString);
  Particle.variable("SoilTemp",soilTempInCString);
  Particle.variable("SoilMoisture1",soilMoisture1String);
  Particle.variable("SoilMoisture2",soilMoisture2String);
  Particle.variable("SoilMoisture3",soilMoisture3String);
  Particle.variable("SoilMoisture4",soilMoisture4String);
  Particle.variable("SoilMoisture5",soilMoisture5String);
  Particle.variable("SoilMoisture6",soilMoisture6String);

  
  Particle.function("Measure-Now",measureNow);
  Particle.function("LowPowerMode",setLowPowerMode);
  Particle.function("Verbose-Mode",setVerboseMode);
  Particle.function("SetTimeZone",setTimeZone);

  if (MEMORYMAPVERSION != EEPROM.read(MEM_MAP::versionAddr)) {          // Check to see if the memory map is the right version
    EEPROM.put(MEM_MAP::versionAddr,MEMORYMAPVERSION);
    for (int i=1; i < 10; i++) {
      EEPROM.put(i,0);                                                  // Zero out the memory - new map or new device
    }
  }

  resetCount = EEPROM.read(MEM_MAP::resetCountAddr);                    // Retrive system recount data from FRAM
  if (System.resetReason() == RESET_REASON_PIN_RESET)                   // Check to see if we are starting from a pin reset
  {
    resetCount++;
    EEPROM.write(MEM_MAP::resetCountAddr, resetCount);                  // If so, store incremented number - watchdog must have done This
  }
  if (resetCount >=6) {                                                 // If we get to resetCount 4, we are resetting without entering the main loop
    EEPROM.write(MEM_MAP::resetCountAddr,4);                            // The hope here is to get to the main loop and report a value of 4 which will indicate this issue is occuring
    fullModemReset();                                                   // This will reset the modem and the device will reboot
  }

  // Load time variables
  int8_t tempTimeZoneOffset = EEPROM.read(MEM_MAP::timeZoneAddr);       // Load Time zone data from FRAM
  if (tempTimeZoneOffset <= 12 && tempTimeZoneOffset >= -12)  Time.zone((float)tempTimeZoneOffset);  // Load Timezone from FRAM
  else Time.zone(0);                                                    // Default is GMT in case proper value not in EEPROM
  time_t t = EEPROM.read(MEM_MAP::currentCountsTimeAddr);
  currentMinutePeriod = Time.minute(t);

  // And set the flags from the control register
  controlRegister = EEPROM.read(MEM_MAP::controlRegisterAddr);          // Read the Control Register for system modes so they stick even after reset
  lowPowerMode    = (0b00000001 & controlRegister);                     // Set the lowPowerMode
  verboseMode     = (0b00001000 & controlRegister);                     // Set the verboseMode
  
  takeMeasurements();                                                   // For the benefit of monitoring the device

  if (batteryVoltage <= lowBattLimit) state = LOW_BATTERY_STATE;         // Only connect if we have battery
  else if(!connectToParticle()) {
    state = ERROR_STATE;                                                // We failed to connect can reset here or go to the ERROR state for remediation
    snprintf(StartupMessage, sizeof(StartupMessage), "Failed to connect");
  }

  if(Particle.connected() && verboseMode) Particle.publish("Startup",StartupMessage,PRIVATE);   // Let Particle know how the startup process went
}

void loop()
{
  switch(state) {
  case IDLE_STATE:
    if (verboseMode && state != oldState) publishStateTransition();
    if (lowPowerMode && (millis() - stayAwakeTimeStamp) > stayAwake) state = SLEEPING_STATE;
    //if (Time.hour() != currentHourlyPeriod) state = MEASURING_STATE;    // We want to report on the hour but not after bedtime
    if ((Time.minute() % 5 == 0) && (Time.now() - currentCountTime > 60)) state = MEASURING_STATE; 
    waitUntil(meterParticlePublish);
    Particle.publish("Minutes",String(Time.minute()),PRIVATE);
    Particle.publish("Elapsed sec",String(Time.now() - currentCountTime),PRIVATE);
    if (batteryVoltage <= lowBattLimit) state = LOW_BATTERY_STATE;               // The battery is low - sleep
    break;

  case MEASURING_STATE:
    if (verboseMode && state != oldState) publishStateTransition();
    // Given you are coming up from a powered off state - you many need to introduce a non-blocking delay here to allow the sensors to warm up
    if (!takeMeasurements())
    {
      state = ERROR_STATE;
      if (verboseMode) {
        waitUntil(meterParticlePublish);
        if(Particle.connected()) Particle.publish("State","Error taking Measurements",PRIVATE);
      }
    }
    else state = REPORTING_STATE;
    break;

  case REPORTING_STATE:
    if (verboseMode && state != oldState) publishStateTransition();
    if (Particle.connected()) {
      if (Time.hour() == 12) Particle.syncTime();                         // Set the clock each day at noon
      sendEvent();                                                      // Send data to Ubidots if we haven't already
      state = RESP_WAIT_STATE;                                          // Wait for Response
    }
    else state = ERROR_STATE;
    break;

  case RESP_WAIT_STATE:
    if (verboseMode && state != oldState) publishStateTransition();
    if (!dataInFlight)                                                // Response received back to IDLE state
    {
      state = IDLE_STATE;
      stayAwake = stayAwakeLong;                                      // Keeps Electron awake after reboot - helps with recovery
      stayAwakeTimeStamp = millis();
    }
    else if (millis() - webhookTimeStamp > webhookWait) {             // If it takes too long - will need to reset
      resetTimeStamp = millis();
      Particle.publish("spark/device/session/end", "", PRIVATE);      // If the device times out on the Webhook response, it will ensure a new session is started on next connect
      state = ERROR_STATE;                                            // Response timed out
    } 
    break;

  case SLEEPING_STATE: {                                                // This state is triggered once the park closes and runs until it opens
    if (verboseMode && state != oldState) publishStateTransition();
    if (!readyForBed)                                                   // Only do these things once - at bedtime
    {
      if (Particle.connected()) {
        if (verboseMode) {
          waitUntil(meterParticlePublish);
          Particle.publish("State","Going to Sleep",PRIVATE);
        }
        delay(1000);                                                    // Time to send last update
        disconnectFromParticle();                                       // If connected, we need to disconned and power down the modem
      }
      EEPROM.write(MEM_MAP::resetCountAddr,resetCount);
      ledState = false;
      digitalWrite(blueLED,LOW);                                        // Turn off the LED
      readyForBed = true;                                               // Set the flag for the night
    }
    int wakeInSeconds = constrain(wakeBoundary - Time.now() % wakeBoundary, 1, wakeBoundary);
    // In your use case, substitute the line below with the instrcution to the TPL5111 to disable the device
    //System.sleep(SLEEP_MODE_SOFTPOWEROFF,secondsToHour);                // Very deep sleep till the next hour - then resets
    System.sleep(D6,RISING,wakeInSeconds);  
    state = IDLE_STATE;                                                  // need to go back to idle immediately after wakup
    connectToParticle();                                                 // Reconnect to Particle (not needed for stop sleep)
    } break;


  case LOW_BATTERY_STATE: {                                             // Sleep state but leaves the fuel gauge on
    if (verboseMode && state != oldState) publishStateTransition();
    if (Particle.connected()) {
      if (verboseMode) {
        waitUntil(meterParticlePublish);
        Particle.publish("State","Low Battery - Sleeping",PRIVATE);
      }
      delay(1000);                                                    // Time to send last update
      disconnectFromParticle();                                       // If connected, we need to disconned and power down the modem
    }
    ledState = false;
    digitalWrite(blueLED,LOW);                                        // Turn off the LED
    int secondsToHour = (60*(60 - Time.minute()));                    // Time till the top of the hour
    // In your use case, substitute the line below with the instrcution to the TPL5111 to disable the device
    System.sleep(SLEEP_MODE_DEEP,secondsToHour);                      // Very deep sleep till the next hour - then resets
    } break;

  case ERROR_STATE:                                                   // To be enhanced - where we deal with errors
    if (verboseMode && state != oldState) publishStateTransition();
    if (millis() > resetTimeStamp + resetWait)
    {
      if (resetCount <= 3) {                                          // First try simple reset
        if (Particle.connected()) Particle.publish("State","Error State - Reset", PRIVATE);    // Brodcast Reset Action
        delay(2000);
        System.reset();
      }
      else if (Time.now() - EEPROM.read(MEM_MAP::currentCountsTimeAddr) > 7200L) { //It has been more than two hours since a sucessful hook response
        if (Particle.connected()) Particle.publish("State","Error State - Lost Session", PRIVATE);  // Broadcast Reset Action
        delay(2000);
        EEPROM.write(MEM_MAP::resetCountAddr,0);                           // Zero the ResetCount
        fullModemReset();                                             // Full Modem reset and reboots
      }
      else {                                                          // If we have had 3 resets - time to do something more
        if (Particle.connected()) Particle.publish("State","Error State - Full Modem Reset", PRIVATE);            // Brodcase Reset Action
        delay(2000);
        EEPROM.write(MEM_MAP::resetCountAddr,0);                           // Zero the ResetCount
        fullModemReset();                                             // Full Modem reset and reboots
      }
    }
    break;
  }
}

void sendEvent()
{
  char data[512];                                                         // Store the date in this character array - not global
  snprintf(data, sizeof(data), "{\"Soilmoisture1\":%4.1f, \"Soilmoisture2\":%4.1f, \"Soilmoisture3\":%4.1f, \"Soilmoisture4\":%4.1f, \"Soilmoisture5\":%4.1f, \"Soilmoisture6\":%4.1f, \"Precipitation\": %i, \"Soiltemp\":%4.1f, \"Humidity\":%4.1f, \"Temperature\":%4.1f, \"Panelhumidity\":%4.1f, \"Paneltemperature\":%4.1f, \"Battery\":%4.1f, \"Radiotech\": %i, \"Signal\": %4.1f, \"Quality\": %4.1f, \"Resets\":%i, \"Alerts\":%i}", soilMoisture1, soilMoisture2, soilMoisture3, soilMoisture4, soilMoisture5, soilMoisture6, precipitationCount, soilTempInC, humidity, temperature, panelHumidity, panelTemperature, batteryVoltage, rat, strengthPercentage, qualityPercentage,resetCount, alertCount);
  Particle.publish("Cellular_LiPo_Hook", data, PRIVATE);  // If lowPowerMode - must have clear to send
  currentCountTime = Time.now();
  EEPROM.write(MEM_MAP::currentCountsTimeAddr, currentCountTime);
  dataInFlight = true;                                                // set the data inflight flag
  webhookTimeStamp = millis();
}

void UbidotsHandler(const char *event, const char *data)              // Looks at the response from Ubidots - Will reset Photon if no successful response
{                                                                     // Response Template: "{{hourly.0.status_code}}" so, I should only get a 3 digit number back
  char dataCopy[strlen(data)+1];                                      // data needs to be copied since if (Particle.connected()) Particle.publish() will clear it
  strncpy(dataCopy, data, sizeof(dataCopy));                          // Copy - overflow safe
  if (!strlen(dataCopy)) {                                            // First check to see if there is any data
    if (Particle.connected()) Particle.publish("Ubidots Hook", "No Data", PRIVATE);
    return;
  }
  int responseCode = atoi(dataCopy);                                  // Response is only a single number thanks to Template
  if ((responseCode == 200) || (responseCode == 201))
  {
    if (Particle.connected()) Particle.publish("State","Response Received", PRIVATE);
    EEPROM.write(MEM_MAP::currentCountsTimeAddr,Time.now());          // Record the last successful Webhook Response
    dataInFlight = false;                                             // Data has been received
  }
  else if (Particle.connected()) Particle.publish("Ubidots Hook", dataCopy, PRIVATE);                    // Publish the response code
}

// These are the functions that are part of the takeMeasurements call

bool takeMeasurements() {
  // Mocked up here for the call - need to replace with your real readings

  // SoilMoisture Measurements here
  soilMoisture1 = random(100);
  snprintf(soilMoisture1String,sizeof(soilMoisture1String), "%4.1f %%", soilMoisture1);
  soilMoisture2 = random(100);
  snprintf(soilMoisture2String,sizeof(soilMoisture2String), "%4.1f %%", soilMoisture2);
  soilMoisture3 = random(100);
  snprintf(soilMoisture3String,sizeof(soilMoisture3String), "%4.1f %%", soilMoisture3);
  soilMoisture4 = random(100);
  snprintf(soilMoisture4String,sizeof(soilMoisture4String), "%4.1f %%", soilMoisture4);
  soilMoisture5 = random(100);
  snprintf(soilMoisture5String,sizeof(soilMoisture5String), "%4.1f %%", soilMoisture5);
  soilMoisture6 = random(100);
  snprintf(soilMoisture6String,sizeof(soilMoisture6String), "%4.1f %%", soilMoisture6);

  // Number of times the precipitation counter has tipped
  precipitationCount = random(1000);
  snprintf(precipitationCountString,sizeof(precipitationCountString), "%i tips", precipitationCount);

  // Measure the soil temp
  soilTempInC = random(100);
  snprintf(soilTempInCString, sizeof(soilTempInCString), "%4.1f C", soilTempInC);

  // Meaure air temp and humidity
  humidity = random(100);
  snprintf(humidityString,sizeof(humidityString), "%4.1f %%", humidity);

  temperature = random(100);
  snprintf(temperatureString,sizeof(temperatureString), "%4.1f C", temperature);

  // Measure panel temp and humidity
  panelHumidity = random(100);
  snprintf(panelHumidityString,sizeof(panelHumidityString), "%4.1f %%", panelHumidity);

  panelTemperature = random(100);
  snprintf(panelTemperatureString,sizeof(panelTemperatureString), "%4.1f C", panelTemperature);

  // Get battery voltage level
  batteryVoltage = 4.0;                      // Voltage level of battery
  snprintf(batteryString, sizeof(batteryString), "%4.1f %%", batteryVoltage);

  if (Cellular.ready()) getSignalStrength();                          // Test signal strength if the cellular modem is on and ready

  return 1;
}

void getSignalStrength()
{
  // New Boron capability - https://community.particle.io/t/boron-lte-and-cellular-rssi-funny-values/45299/8
  CellularSignal sig = Cellular.RSSI();
  rat = sig.getAccessTechnology();
  //float strengthVal = sig.getStrengthValue();
  strengthPercentage = sig.getStrength();
  //float qualityVal = sig.getQualityValue();
  qualityPercentage = sig.getQuality();
  snprintf(SignalString,sizeof(SignalString), "%s S:%2.0f%%, Q:%2.0f%% ", radioTech[rat], strengthPercentage, qualityPercentage);
}


// These functions control the connection and disconnection from Particle
bool connectToParticle() {
  Cellular.on();
  Particle.connect();
  // wait for *up to* 5 minutes
  for (int retry = 0; retry < 300 && !waitFor(Particle.connected,1000); retry++) {
    // Code I want to run while connecting
    Particle.process();
  }
  if (Particle.connected()) {
    waitFor(Time.isValid, 60000);
    return 1;                               // Were able to connect successfully
  }
  else return 0;                                                    // Failed to connect
}

bool disconnectFromParticle()
{
  Particle.disconnect();                                          // Otherwise Electron will attempt to reconnect on wake
  Cellular.off();
  delay(1000);                                                    // Bummer but only should happen once an hour
  return true;
}

bool notConnected() {
  return !Particle.connected();                             // This is a requirement to use waitFor
}


// These are the particle functions that allow you to configure and run the device
// They are intended to allow for customization and control during installations
// and to allow for management.

int measureNow(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    state = MEASURING_STATE;
    return 1;
  }
  else return 0;
}

int setVerboseMode(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    verboseMode = true;
    controlRegister = EEPROM.read(MEM_MAP::controlRegisterAddr);
    controlRegister = (0b00001000 | controlRegister);                    // Turn on verboseMode
    EEPROM.write(MEM_MAP::controlRegisterAddr,controlRegister); // Write it to the register
    Particle.publish("Mode","Set Verbose Mode",PRIVATE);
    return 1;
  }
  else if (command == "0")
  {
    verboseMode = false;
    controlRegister = EEPROM.read(MEM_MAP::controlRegisterAddr);
    controlRegister = (0b11110111 & controlRegister);                    // Turn off verboseMode
    EEPROM.write(MEM_MAP::controlRegisterAddr,controlRegister); // Write it to the register
    Particle.publish("Mode","Cleared Verbose Mode",PRIVATE);
    return 1;
  }
  else return 0;
}

int setTimeZone(String command)
{
  char * pEND;
  char data[256];
  int8_t tempTimeZoneOffset = strtol(command,&pEND,10);                       // Looks for the first integer and interprets it
  if ((tempTimeZoneOffset < -12) | (tempTimeZoneOffset > 12)) return 0;   // Make sure it falls in a valid range or send a "fail" result
  Time.zone((float)tempTimeZoneOffset);
  EEPROM.write(MEM_MAP::timeZoneAddr,tempTimeZoneOffset);                             // Store the new value in FRAMwrite8
  snprintf(data, sizeof(data), "Time zone offset %i",tempTimeZoneOffset);
  Particle.publish("Time",data,PRIVATE);
  delay(1000);
  Particle.publish("Time",Time.timeStr(),PRIVATE);
  return 1;
}


int setLowPowerMode(String command)                                   // This is where we can put the device into low power mode if needed
{
  if (command != "1" && command != "0") return 0;                     // Before we begin, let's make sure we have a valid input
    controlRegister = EEPROM.read(MEM_MAP::controlRegisterAddr);
  if (command == "1")                                                 // Command calls for setting lowPowerMode
  {
    Particle.publish("Mode","Low Power",PRIVATE);
    controlRegister = (0b00000001 | controlRegister);                  // If so, flip the lowPowerMode bit
    lowPowerMode = true;
  }
  else if (command == "0")                                            // Command calls for clearing lowPowerMode
  {
    Particle.publish("Mode","Normal Operations",PRIVATE);
    controlRegister = (0b1111110 & controlRegister);                  // If so, flip the lowPowerMode bit
    lowPowerMode = false;
  }
  EEPROM.write(MEM_MAP::controlRegisterAddr,controlRegister); // Write it to the register
  return 1;
}

// Utility functions here...
void publishStateTransition(void)
{
  char stateTransitionString[40];
  snprintf(stateTransitionString, sizeof(stateTransitionString), "From %s to %s", stateNames[oldState],stateNames[state]);
  oldState = state;
  if(Particle.connected()) {
    waitUntil(meterParticlePublish);
    Particle.publish("State Transition",stateTransitionString, PRIVATE);
  }
  Serial.println(stateTransitionString);
}

bool meterParticlePublish(void)
{
  static unsigned long lastPublish = 0;
  if(millis() - lastPublish >= publishFrequency) {
    lastPublish = millis();
    return 1;
  }
  else return 0;
}

void fullModemReset() {  // Adapted form Rikkas7's https://github.com/rickkas7/electronsample

	Particle.disconnect(); 	                                         // Disconnect from the cloud
	unsigned long startTime = millis();  	                           // Wait up to 15 seconds to disconnect
	while(Particle.connected() && millis() - startTime < 15000) {
		delay(100);
	}
	// Reset the modem and SIM card
	// 16:MT silent reset (with detach from network and saving of NVM parameters), with reset of the SIM card
	Cellular.command(30000, "AT+CFUN=16\r\n");
	delay(1000);
	// Go into deep sleep for 10 seconds to try to reset everything. This turns off the modem as well.
	System.sleep(SLEEP_MODE_DEEP, 10);
}
