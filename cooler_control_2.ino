//#include <WiFi.h>
#include <DallasTemperature.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <Adafruit_MCP4725.h>
#include <ArduinoOTA.h>
#include <SDL_Arduino_TCA9545.h>
#include <TSL2561.h>
#include <Ticker.h>


#define ONE_WIRE_BUS 14
#define TEMPERATURE_PRECISION 12
#define MAX_SRV_CLIENTS 1
#define WATER_SENSOR A0
#define AButtonPin 27 //blue
#define BButtonPin 33 //blue and white
#define CButtonPin 15 //green
#define BButtonPin 32 //green and white

/* define event bits */
#define TASK_1_BIT        ( 1 << 0 ) //1
#define TASK_2_BIT        ( 1 << 1 ) //10
#define TASK_3_BIT        ( 1 << 2 ) //100
#define ALL_SYNC_BITS (TASK_1_BIT | TASK_2_BIT | TASK_3_BIT) //111



#include "Bme_Functions.h"  // Functions for the BME280 sensors
#include "HelperFunctions.h" // misc functions called to help out
#include "Time_Functions.h" // Functions to interact with time
#include "DS1820_Sensors.h" // Dallas one wire tempeture structure and functions
//#include "Messages.h" // this is where messages are defined
//#include "ButtonTask.h" // this implements the button task
//#include "MainTask.h" // this implements the message target core work at handling messages
#include "ButtonFunctions.h"

using namespace BME280_Sensors;

Ticker ticker;

Adafruit_MCP4725 dac;
Adafruit_MCP4725 dac2;
Adafruit_MCP4725 dac3;
Adafruit_MCP4725 dac4;
Adafruit_MCP4725 dac5;

SDL_Arduino_TCA9545 TCA9545;

TSL2561 tsl(TSL2561_ADDR_FLOAT);

Button_funtions button_status;

DS1820_Sensors tempSensor[2]; //max of five sensors right now at 750ms calculation time for the tempurter.


DS1820_Sensors* core1Thermometer = &tempSensor[0];
DS1820_Sensors* core2Thermometer = &tempSensor[1];

DS1820_Sensors* sensorSet[] = {core1Thermometer, core2Thermometer};

BME_Sensors systemsensor;
BME_Sensors housesensor;
BME_Sensors outsidesensor;


int8_t rslt = BME280_OK;

//MainTask mainTask;
//ButtonTask buttonATask(&mainTask, AButtonPin);

WiFiServer t_server(23);
WiFiClient t_serverClients[MAX_SRV_CLIENTS];

//TaskManager taskManager;

float waterLevelVoltage, rpm = 0;
int water_count_off, water_count_on = 0;
int cool_on, cool_off, cool_count, cool_timer = 0;
String Tdebug = "";
int runState = 1;
int buttonState1 = 0;
int buttonState2 = 0;
int buttonState3 = 0;
int buttonState4 = 0;
int buttonState5 = 0;
int button_status_update = 0;
//int buttonState = digitalRead(pushButton1);

//int buttonState2 = digitalRead(pushButton2);
//int buttonState3 = digitalRead(pushButton3);

unsigned int localPort = 2390;      // local port to listen for UDP packets
uint16_t ir, full, visible, lux;
int rpm_time = 0;
int systemoverride = 0;
int count = 0;
float chiiler_temp = 13;

int top, top1, old_t_count, t_count, LineCheck = 0;

int start_check = 0;

//float waterLevelVoltage =0;
int volume = 0;

int ac_count = 0;

boolean ticker_reached;
int timer_task_count = 0;
int timer_task_count_2 = 0;
int timer_task_count_3 = 0;
int timer_task_count_4 = 0;
int timer_task_count_5 = 0;
int timer_task_count_6 = 0;
int timer_task_count_7 = 0;
int timer_task_count_8 = 0;
int timer_task_count_9 = 0;
int timer_task_count_10 = 0;
int task_count = 0;

uint8_t config;

const char *ssid = "@HomeB702";
const char *password = "3gvxv9fpnfpsur";

// foreward delcare functions passed to task constructors now required
void ticker_handler();
void OnUpdateTaskGetNTPTime(uint32_t deltaTime);
void OnUpdateTime(uint32_t deltaTime);
void OnUpdateButtons(uint32_t deltaTime);
void PrintTime(int i);
void PrintButtonStat(int i);
void PrintSysVars(int i);
void HandelTelNet(uint32_t deltaTime);
void OnUpdateTaskGetDallasTemp(uint32_t deltaTime);
void OnUpdateTaskGetBus_1(uint32_t deltaTime);
void OnUpdateTaskGetBus_1_1(uint32_t deltaTime);
void OnUpdateTaskGetBus_2(uint32_t deltaTime);
void OnUpdateTaskUpdateCounter(uint32_t deltaTime);
void OnUpdateWaterLevel(uint32_t deltaTime);
void OnHeartBeatOn(uint32_t deltaTime);
void OnHeartBeatOff(uint32_t deltaTime);
bool  scan_i2c_devices();

/*
  FunctionTask taskGetNTPTime(OnUpdateTaskGetNTPTime, MsToTaskTime(3600000)); // turn on task every 3600000 ms (one hour)
  FunctionTask taskUpdateTime(OnUpdateTime, MsToTaskTime(1000)); // turn on task every 1,000ms (one second)
  FunctionTask taskGetDallasTemp(OnUpdateTaskGetDallasTemp, MsToTaskTime(3300)); // turn on task every 3015ms
  FunctionTask taskUpdateCounter(OnUpdateTaskUpdateCounter, MsToTaskTime(1020)); // turn on task every 3025ms
  FunctionTask taskCheckButtons(OnUpdateButtons, MsToTaskTime(1040)); // turn on task every 100ms
  FunctionTask taskUpdateWaterLevel(OnUpdateWaterLevel, MsToTaskTime(5000)); // turn on task every 3030ms
  FunctionTask taskHandelTelNet(HandelTelNet, MsToTaskTime(1060)); // turn on task every 3030ms
  FunctionTask heartBeatOnTask(OnHeartBeatOn, MsToTaskTime(400));
  FunctionTask heartBeatOffTask(OnHeartBeatOff, MsToTaskTime(600));
*/




void setup() {
  // set up serial port
  Serial.begin(115200);

  delay (100); //give time for sensors to power up


  // set up ic2 bus and multiplexer
  Wire.pins(4, 5); //SDA (white),SCL (yellow)
  //  Wire.begin(23, 22, 400000);  //SDA (white),SCL (yellow)
  Wire.setClock(400000L);

  Serial.println("setting up the TCA9545 Bus multiplexer");
  TCA9545.TCA9545SetConfig();

  TCA9545.write_control_register(TCA9545_CONFIG_BUS0);
  while ( !scan_i2c_devices() ) {
    delay ( 500 );
    Serial.print ( "." );
    Tdebug += "stuck in bus 0 ";
    Tdebug += "\n\r";
    HandelTelNet(1);
    yield();
  }

  TCA9545.write_control_register(TCA9545_CONFIG_BUS1);
  while ( !scan_i2c_devices() ) {
    delay ( 500 );
    Serial.print ( "." );
    Tdebug += "stuck in bus 1 ";
    Tdebug += "\n\r";
    HandelTelNet(1);
    yield();
  }

  TCA9545.write_control_register(TCA9545_CONFIG_BUS2);
  while ( !scan_i2c_devices() ) {
    delay ( 500 );
    Serial.print ( "." );
    Tdebug += "stuck in bus 2 ";
    Tdebug += "\n\r";
    HandelTelNet(1);
    yield();
  }

  TCA9545.write_control_register(TCA9545_CONFIG_BUS3);
  while ( !scan_i2c_devices() ) {
    delay ( 500 );
    Serial.print ( "." );
    Tdebug += "stuck in bus 3 ";
    Tdebug += "\n\r";
    HandelTelNet(1);
    yield();
  }


  TCA9545.write_control_register(TCA9545_CONFIG_BUS3);
  dac.begin(0x60);
  dac.setVoltage(0, false);
  dac2.begin(0x61);
  dac2.setVoltage(0, false);
  dac3.begin(0x62);
  dac3.setVoltage(0, false);
  dac4.begin(0x63);
  dac4.setVoltage(0, false);

  TCA9545.write_control_register(TCA9545_CONFIG_BUS2);
  dac5.begin(0x62);
  dac5.setVoltage(0, false);

  TCA9545.write_control_register(TCA9545_CONFIG_BUS0);
  tsl.begin();
  tsl.setGain(TSL2561_GAIN_0X);
  tsl.setTiming(TSL2561_INTEGRATIONTIME_101MS); //set up the onewire bus

  // set up BMP280 sensors
  Serial.println("Setting up the BME280 Sensors");
  BMESetup();

  Serial.println("Setting up the Dallas one wire sensors");
  OneWireSetup();

  //set up wifi server
  Serial.println ( ". starting wifi" );
  WiFi.mode(WIFI_STA);
  WiFi.begin ( ssid, password );


  // Set a static IP (optional)
  IPAddress ip(192, 168, 0, 6);
  IPAddress gateway(192, 168, 0, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.config(ip, gateway, subnet);


  // Wait for connection
  int wifi_count = 0;
  while ( WiFi.status() != WL_CONNECTED ) {
    delay ( 500 );
    Serial.print ( "." );
    wifi_count++;
    if (wifi_count == 10) {
      break;
    }
  }


  /*
    unsigned long wifi_start = millis();
    do {
      if  ( WiFi.status() == WL_CONNECTED ) {
        Serial.println ( "" );
        Serial.print ( "Connected to " );
        Serial.println ( ssid );
        Serial.print ( "IP address: " );
        Serial.println ( WiFi.localIP() );
        Serial.println("Ready");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
        break;
      }
    }
    while ((millis() - wifi_start) < 75000);

  */


  //start UART and the server
  t_server.begin();
  t_server.setNoDelay(true);

  Serial.print("Ready! Use 'telnet ");
  Serial.print(WiFi.localIP());
  Serial.println(" 23' to connect");

  //set up UDP poet for NTP
  Serial.println("Starting UDP");
  udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(localPort);

  while (t_epoch == 0) {
    t_epoch = read_NTP();
    //delay (5000);
    Serial.print ( "." );
  }





  Serial.println("------------- Starting Tasks-----------");
  /* taskManager.StartTask(&taskGetNTPTime);
    taskManager.StartTask(&taskUpdateTime);
    taskManager.StartTask(&taskGetDallasTemp);
    taskManager.StartTask(&taskUpdateCounter);
    taskManager.StartTask(&taskCheckButtons);
    taskManager.StartTask(&mainTask);
    //taskManager.StartTask(&buttonATask);
    //taskManager.StartTask(&heartBeatOnTask);
    taskManager.StartTask(&taskUpdateWaterLevel);
    taskManager.StartTask(&taskHandelTelNet);
    //  pinMode(LED, OUTPUT);
    //  pinMode(LEDPIN, OUTPUT);
    //  writeLED(false);
  */
  ticker_reached = false;

  //call ticker_handler() in 1 second
  ticker.attach(1, ticker_handler);

  Serial.println("-------------finished Starting Tasks-----------");
  //delay(10000);

  // No authentication by default
  ArduinoOTA.setPassword((const char *)"123");

  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();

  Serial.println("-------------Starting-----------");


}

void OneWireSetup(void)
{
  // start serial port
  Serial.println("Dallas Temperature IC Control Library Setup");

  // Start up the library
  sensors.begin();

  // locate devices on the bus
  Serial.print("Locating devices...");
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" devices.");

  // report parasite power requirements
  Serial.print("Parasite power is: ");
  if (sensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");

  // Assign address manually. The addresses below will beed to be changed
  // to valid device addresses on your bus. Device address can be retrieved
  // by using either oneWire.search(deviceAddress) or individually via
  // sensors.getAddress(deviceAddress, index)
  //insideThermometer = { 0x28, 0x1D, 0x39, 0x31, 0x2, 0x0, 0x0, 0xF0 };
  //outsideThermometer   = { 0x28, 0x3F, 0x1C, 0x31, 0x2, 0x0, 0x0, 0x2 };


  /*{
    {"indoor", {0x28, 0xFF, 0x4C, 0xDA, 0x70, 0x16, 0x03, 0xD1}},
    {"Outdoor",{0x28, 0xFF, 0x37, 0xD8, 0x70, 0x16, 0x03, 0xCB}}
    };
  */
  //core1Thermometer->deviceaddress = 0x28, 0xFF, 0x4C, 0xDA, 0x70, 0x16, 0x03, 0xD1;
  //core2Thermometer->deviceaddress = 0x28, 0xFF, 0x37, 0xD8, 0x70, 0x16, 0x03, 0xCB;

  // Search for devices on the bus and assign based on an index. Ideally,
  // you would do this to initially discover addresses on the bus and then
  // use those addresses and manually assign them (see above) once you know
  // the devices on your bus (and assuming they don't change).
  //
  // method 1: by index
  //if (!sensors.getAddress(insideThermometer, 0)) Serial.println("Unable to find address for Device 0");
  if (!sensors.getAddress(core1Thermometer->deviceaddress, 0)) Serial.println("Unable to find address for Device 0");
  core1Thermometer->setDeviceNum(0);
  core1Thermometer->printAddress();
  core1Thermometer->setresolution();
  core1Thermometer->printData();

  //if (!sensors.getAddress(outsideThermometer, 1)) Serial.println("Unable to find address for Device 1");
  if (!sensors.getAddress(core2Thermometer->deviceaddress, 1)) Serial.println("Unable to find address for Device 0");
  core2Thermometer->setDeviceNum(1);
  core2Thermometer->printAddress();
  core2Thermometer->setresolution();
  core2Thermometer->printData();


}

void BMESetup(void)
{
  // start serial port
  Serial.println("BME280 IC Control Library Setup");

  // Start up the library
  //bme280_dev.begin();

  // indoor navigation
  //   Serial.println("-- Indoor Navigation Scenario --");
  //   Serial.println("normal mode, 16x pressure / 2x temperature / 1x humidity oversampling,");
  //   Serial.println("0.5ms standby period, filter 16x");
  //   bme.setSampling(Adafruit_BME280::MODE_NORMAL,
  //                   Adafruit_BME280::SAMPLING_X2,  // temperature
  //                   Adafruit_BME280::SAMPLING_X16, // pressure
  //                   Adafruit_BME280::SAMPLING_X1,  // humidity
  //                   Adafruit_BME280::FILTER_X16,
  //                   Adafruit_BME280::STANDBY_MS_0_5 );

  // Bus 1
  TCA9545.write_control_register(TCA9545_CONFIG_BUS1);

  systemsensor.dev.dev_id = BME280_I2C_ADDR_PRIM;
  systemsensor.dev.intf = BME280_I2C_INTF;
  systemsensor.dev.read = user_i2c_read;
  systemsensor.dev.write = user_i2c_write;
  systemsensor.dev.delay_ms = user_delay_ms;
  bme280_init(&systemsensor.dev);
  rslt = systemsensor.SetSensorsNormalSettings(&systemsensor.dev);
  if (rslt == 0) {
    Serial.println("system bme280 sensor intilized");
    systemsensor.PrintBmeSettings(&systemsensor.dev);
  } else {
    Serial.println("system bme280 sensor failed to intilized");
  }
  delay (100); //give time for sensors to power up
  //outsidesensor.BME_Sensors();
  outsidesensor.dev.dev_id = BME280_I2C_ADDR_SEC;
  outsidesensor.dev.intf = BME280_I2C_INTF;
  outsidesensor.dev.read = user_i2c_read;
  outsidesensor.dev.write = user_i2c_write;
  outsidesensor.dev.delay_ms = user_delay_ms;
  bme280_init(&outsidesensor.dev);
  rslt = outsidesensor.SetSensorsNormalSettings(&outsidesensor.dev);
  if (rslt == 0) {
    Serial.println("outside bme280 sensor intilized");
    outsidesensor.PrintBmeSettings(&outsidesensor.dev);
  } else {
    Serial.println("outside bme280 sensor failed to intilized");
  }
  delay (100); //give time for sensors to power up
  // Bus 2
  TCA9545.write_control_register(TCA9545_CONFIG_BUS2);
  //BME_Sensors();
  housesensor.dev.dev_id = BME280_I2C_ADDR_SEC;
  housesensor.dev.intf = BME280_I2C_INTF;
  housesensor.dev.read = user_i2c_read;
  housesensor.dev.write = user_i2c_write;
  housesensor.dev.delay_ms = user_delay_ms;
  bme280_init(&housesensor.dev);
  rslt = housesensor.SetSensorsNormalSettings(&housesensor.dev);
  if (rslt == 0) {
    Serial.println("house bme280 sensor intilized");
    housesensor.PrintBmeSettings(&housesensor.dev);
  } else {
    Serial.println("house bme280 sensor failed to intilized");
  }
}

void OnUpdateTime(uint32_t deltaTime) {
  //lwdWhere = OnUpdateTime_task;
  Serial.println("Update Time ");
  Tdebug += "Update Time ";
  Tdebug += "\n\r";
  if (rpm_time == 60) {
    //rpm = getrpmstat();
    rpm_time = 1;
  } else {
    rpm_time++;
  }

  if (runState == 2) {
    if ((( ht_time >= 5) && (mt_time >= 30)) && (( ht_time <= 6) && (mt_time <= 59))) {
      Tdebug += "Early Morning Time Shedule";
      if ((mt_time == 00) || (mt_time == 15) || (mt_time == 30) || (mt_time == 45)) {
        if (outsidesensor.celsius >= housesensor.celsius) {
          buttonState1 = 1;
        }
        buttonState3 = 1;
        button_status.update = 1;
      }
      if ((mt_time == 8) || (mt_time == 23) || (mt_time == 38) || (mt_time == 53)) {
        buttonState1 = 0;
        buttonState3 = 0;
        button_status.update = 1;
      }
    }

    if (( ht_time >= 8)  && ( ht_time <= 19)) {
      Serial.print("Day Time Schedule ");
      Tdebug += "Day Time Schedule ";
      if ((mt_time == 00) || (mt_time == 15) || (mt_time == 30) || (mt_time == 45)) {
        if (outsidesensor.celsius >= housesensor.celsius) {
          buttonState1 = 1;
        }
        buttonState3 = 1;
        //buttonState2 = 1;
        button_status.update = 1;
      }
      if ((mt_time == 10) || (mt_time == 23) || (mt_time == 38) || (mt_time == 53)) {
        buttonState1 = 0;
        //buttonState3 = 0;
        //buttonState2 = 0;
        button_status.update = 1;
      }
    }

    if ((( ht_time >= 20) && (mt_time >= 00)) || (( ht_time <= 5) && (mt_time <= 29))) {
      Tdebug += "Night Time Schedule ";
      if ((mt_time == 00) || (mt_time == 20) || (mt_time == 40)) {
        if (outsidesensor.celsius >= housesensor.celsius) {
          buttonState1 = 1;
        }
        buttonState3 = 1;
        button_status.update = 1;
      }
      if ((mt_time == 12)  || (mt_time == 32) || (mt_time == 52)) {
        buttonState1 = 0;
        buttonState3 = 0;
        button_status.update = 1;
      }
    }
  }

  if (runState == 1) {
    // turn off everything with timer count down of 120 cycles
    if (cool_off == 1) {
      cool_timer ++;
      if (cool_timer == 120) {
        cool_off = 0;
        cool_on = 0;
        buttonState1 = 0;
        buttonState2 = 0;
        buttonState3 = 0;
        buttonState4 = 0;
        buttonState5 = 0;
        button_status.update = 1;
        cool_timer = 0;
      }
    }
  }

  if (runState == 1)  {
    chiiler_temp = ((core1Thermometer ->avgTemp + core2Thermometer ->avgTemp) /2);
    if (( ht_time >= 5) && ( ht_time <= 7)) {
      if (( housesensor.celsius <= 19) && (cool_on = 1)) {
      cool_off = 1;
      cool_count ++;
      }
    } 
    if (( ht_time >= 8)  && ( ht_time <= 21)) {
      if (( housesensor.celsius <= 20) && (cool_on = 1)) {
      cool_off = 1;
      cool_count ++;
      }
    } 
    if (( ht_time >= 22) && ( ht_time <= 5)) {
      if (( housesensor.celsius <= 20) && (cool_on = 1)) {
      cool_off = 1;
      cool_count ++;
      }  
    } 
    if (cool_count == 200)   {
      //  Turn waterpad pump on befor the blower stats to soak waterpads
      if ((outsidesensor.celsius >= housesensor.celsius ) && (systemsensor.humidity <= 90)) {
        buttonState1 = 1;
        //if (chiiler_temp >= 16.50){
        //  buttonState2 = 1;
        //} else if (chiiler_temp <= 16){
        //  buttonState2 = 0;
        //}
        button_status.update = 1;
      }
      // turn off waterpad pump if humidity of system is greter than 87% of outside is cooler than inside

      if ((outsidesensor.celsius + 2 <= housesensor.celsius ) || (systemsensor.humidity >= 87)) {
        buttonState1 = 0;
        button_status.update = 1;
      }
    } else if (cool_count == 300) {
      // turn on blower after the water pad has had time to soak
      if (cool_on == 0) {
        cool_on = 1;
        ac_count = 1;
        buttonState2 = 1;
        buttonState3 = 1;
        buttonState5 = 1;
        button_status.update = 1;
        cool_count = 0;
      }
    }


    if (( ht_time >= 5) && ( ht_time <= 7)) {
      if ((cool_count <= 300) && (housesensor.celsius >= 20.25)) {
      cool_count++;
      }
    }

    if (( ht_time >= 8)  && ( ht_time <= 21)) {
      if ((cool_count <= 300) && (housesensor.celsius >= 20.25)) {
      cool_count++;
      }
    }

    if (( ht_time <= 5) || ( ht_time >= 22) ) {
      if ((cool_count <= 300) && (housesensor.celsius >= 21.25)) {
      cool_count++;
      }
    }
    

    if (cool_count >= 301) {
      cool_count = 0;
      if (ac_count == 1){
        ac_count = 0;
        buttonState2 = 0;
        button_status.update = 1;
      }
      else if (ac_count == 0){
        ac_count = 1;
        buttonState2 = 1;
        button_status.update = 1;
      }
    }
  }


  Serial.print(" Cool on Counter: ");
  Serial.print(cool_count);
  Serial.print(" Cool off Counter: ");
  Serial.println(cool_timer);

  t_time = UpdateTime();
  Tdebug += "Time = ";
  Tdebug += calcDigits(ht_time);
  Tdebug += ":";
  Tdebug += calcDigits(mt_time);
  Tdebug += ":";
  Tdebug += calcDigits(st_time);
  Tdebug += "\n\r";
}

void OnUpdateTaskGetDallasTemp(uint32_t deltaTime) {
  //lwdWhere = OnUpdateTaskGetDallasTemp_task;
  //ESP.wdtDisable();
  // call sensors.requestTemperatures() to issue a global temperature
  // request to all devices on the bus
  Serial.println("Requesting temperatures of the one wire sensors");
  Tdebug += "Requesting temperatures of the one wire sensors";
  Tdebug += "\n\r";
  //sensors.requestTemperatures();
  //Serial.println("DONE");

  for (auto& set : tempSensor)
  {
    int attempts = 0;
    double newTemp;
    //->Serial1.print("geting the temperatures for ");
    //set.printAddress();
    //->Serial1.println();
    do {
      newTemp = set.getTemp();
      //->Serial1.println(newTemp);
      attempts++;
    } while (!set.addHiTemp(newTemp) and (attempts < 5));
    set.hiTemp = set.hiArray[0];

    attempts = 0;
    do {
      newTemp = set.getTemp();
      attempts++;
    } while (!set.addLoTemp(newTemp) and (attempts < 5));
    set.loTemp = set.loArray[0];
    set.avgTemp = (set.hiTemp + set.loTemp) / 2.0;
    set.updateDailyHiandLow();
  }
}

void OnUpdateTaskGetBus_1(uint32_t deltaTime) {
  //Serial.println("------------------------------");
  //ESP.wdtDisable();
  // Bus 1
  TCA9545.write_control_register(TCA9545_CONFIG_BUS1);
  config = TCA9545_CONFIG_BUS1;
  Serial.print("Bus 1 Control Register:");
  Serial.println(config & 0x0F, HEX);
  Tdebug += "Bus 1 Control Register:";
  Tdebug += config & 0x0F, HEX;
  Tdebug += "\n\r";

  rslt = outsidesensor.UpdateRedings(&outsidesensor.dev);
  //rslt = systemsensor.UpdateRedings(&systemsensor.dev);


}

void OnUpdateTaskGetBus_1_1(uint32_t deltaTime) {
  //Serial.println("------------------------------");
  //ESP.wdtDisable();
  // Bus 1
  TCA9545.write_control_register(TCA9545_CONFIG_BUS1);
  config = TCA9545_CONFIG_BUS1;
  Serial.print("Bus 1 Control Register:");
  Serial.println(config & 0x0F, HEX);
  Tdebug += "Bus 1 Control Register:";
  Tdebug += config & 0x0F, HEX;
  Tdebug += "\n\r";

  //rslt = outsidesensor.UpdateRedings(&outsidesensor.dev);
  rslt = systemsensor.UpdateRedings(&systemsensor.dev);


}
void OnUpdateTaskGetBus_2(uint32_t deltaTime) {
  //Serial.println("------------------------------");
  // Bus 1
  TCA9545.write_control_register(TCA9545_CONFIG_BUS2);
  config = TCA9545_CONFIG_BUS2;
  Serial.print("Bus 2 Control Register:");
  Serial.println(config & 0x0F, HEX);
  Tdebug += "Bus 2 Control Register:";
  Tdebug += config & 0x0F, HEX;
  Tdebug += "\n\r";
  rslt = housesensor.UpdateRedings(&housesensor.dev);
}

void OnUpdateTaskUpdateCounter(uint32_t deltaTime) {
  //lwdWhere = OnUpdateTaskUpdateCounter_task;
  Serial.println("Update Counter ");
  Tdebug += "Update Counter ";
  Tdebug += "\n\r";
  count++;
  if (t_count > 3) {
    start_check = 1;
  }
  t_count = count;
  //->Serial1.println(count);


}
/*
  / conflict with the onewire lib prevent this working
  void OnHeartBeatOn(uint32_t deltaTime) {
  //lwdWhere = OnHeartBeatOff_task;
  HeartbeatMessage message(true);
  if (!mainTask.SendAsyncMessage(message))
  {
    Serial.println(">> message buffer overflow <<");
  }

  // toggle tasks
  taskManager.StopTask(&heartBeatOnTask);
  taskManager.StartTask(&heartBeatOffTask);
  }


  void OnHeartBeatOff(uint32_t deltaTime) {
  //lwdWhere = OnHeartBeatOff_task;
  HeartbeatMessage message(false);
  if (!mainTask.SendAsyncMessage(message))
  {
    Serial.println(">> message buffer overflow <<");
  }

  // toggle tasks
  taskManager.StopTask(&heartBeatOffTask);
  taskManager.StartTask(&heartBeatOnTask);
  }

*/

void OnUpdateButtons(uint32_t deltaTime) {
  //lwdWhere = OnUpdateTaskUpdateButtons_task;
  Serial.println("Update Buttons ");
  Tdebug += "Update Buttons ";
  Tdebug += "\n\r";

  /*
    Serial.println("-----------TCA9545_CONFIG_BUS0-------------------");
    TCA9545.write_control_register(TCA9545_CONFIG_BUS0);
    scan_i2c_devices();

    Serial.println("-----------TCA9545_CONFIG_BUS1-------------------");
    TCA9545.write_control_register(TCA9545_CONFIG_BUS1);
    scan_i2c_devices();

    Serial.println("-----------TCA9545_CONFIG_BUS2-------------------");
    TCA9545.write_control_register(TCA9545_CONFIG_BUS2);
    scan_i2c_devices();

    Serial.println("-----------TCA9545_CONFIG_BUS3-------------------");
    TCA9545.write_control_register(TCA9545_CONFIG_BUS3);
    scan_i2c_devices();

  */

  button_status.abutton = buttonState1;
  button_status.bbutton = buttonState2;
  button_status.cbutton = buttonState3;
  button_status.dbutton = buttonState4;
  button_status.ebutton = buttonState5;

  Serial.print("Run State: ");
  Serial.print (runState);
  Serial.print(" button update State: ");
  Serial.print (button_status.update);
  Serial.print(" Pump: ");
  Serial.print (button_status.abutton);
  Serial.print(" Blower: ");
  Serial.print(button_status.cbutton);
  Serial.print(" Water: ");
  Serial.println(button_status.dbutton);
  Tdebug += "Run State: ";
  Tdebug += runState;
  Tdebug += " button update State: ";
  Tdebug += button_status.update;
  Tdebug += " Pump: ";
  Tdebug += button_status.abutton;
  Tdebug += " Blower: ";
  Tdebug += button_status.cbutton;
  Tdebug += " Water: ";
  Tdebug += button_status.dbutton;
  Tdebug += "\n\r";



  if (button_status.update == 1) {
    Tdebug += "deteced buton state change";
    Tdebug += "\n\r";
    TCA9545.write_control_register(TCA9545_CONFIG_BUS3);
    // waterpad pump

    if (button_status.abutton == 1) {
      dac.setVoltage(4095, false);
    } else if (button_status.abutton == 0) {
      dac.setVoltage(0, false);
    }

    // AC Unit
    if (button_status.bbutton == 1) {
      dac2.setVoltage(4095, false);
    } else if (button_status.bbutton == 0) {
      dac2.setVoltage(0, false);
    }
    //blower
    if (button_status.cbutton == 1) {
      dac3.setVoltage(4095, false);
    } else if (button_status.cbutton == 0) {
      dac3.setVoltage(0, false);
    }

    //water
    if (button_status.dbutton == 1) {
      dac4.setVoltage(4095, false);
    } else if (button_status.dbutton == 0) {
      dac4.setVoltage(0, false);
    }

    TCA9545.write_control_register(TCA9545_CONFIG_BUS2);

    //circ pump
    if (button_status.ebutton == 1) {
      dac5.setVoltage(4095, false);
    } else if (button_status.ebutton == 0) {
      dac5.setVoltage(0, false);
    }
    button_status.update = 0;
  }
}

int getrpmstat() {
  //int rmpcount = mainTask.Abutton_count;
  //mainTask.Abutton_count = 0;
  int rmpcount = 0;
  return rmpcount;
}

float resistanceToVolume(float resistance, float zeroResistance, float calResistance, float calVolume) {
  if (resistance > zeroResistance || (zeroResistance - calResistance) == 0.0) {
    // Stop if the value is above the zero threshold, or no max resistance is set (would be divide by zero).
    return 0.0;
  }
  // Compute scale factor by mapping resistance to 0...1.0+ range relative to maxResistance value.
  float scale = (zeroResistance - resistance) / (zeroResistance - calResistance);
  // Scale maxVolume based on computed scale factor.
  return calVolume * scale;
}

void OnUpdateWaterLevel(uint32_t deltaTime) {
  //lwdWhere = OnUpdateWaterLevel_task;
  Serial.println("Update Water level ");
  Tdebug += "Update Water level ";

  waterLevelVoltage = analogRead(WATER_SENSOR);
  Serial.print("waterLevelVoltage: ");
  Serial.println(waterLevelVoltage);
  volume = waterLevelVoltage;
  Tdebug += volume;
  Tdebug += "\n\r";

  if ((waterLevelVoltage <= 50) && ((runState == 1) || (runState == 2))) {
    //TCA9545.write_control_register(TCA9545_CONFIG_BUS3);
    //dac4.setVoltage(4095, false);
    buttonState4 = 1;
    button_status.update = 1;
    button_status.water = 1;
  } else {
    if ((waterLevelVoltage >= 350) && (button_status.water == 1)) {
      //TCA9545.write_control_register(TCA9545_CONFIG_BUS3);
      //dac4.setVoltage(0, false);
      buttonState4  = 0;
      button_status.update = 1;
      button_status.water = 0;
    }
  }
}

String millis2time() {
  String Time = "";
  unsigned long ss;
  byte mm, hh;
  ss = millis() / 1000;
  hh = ss / 3600;
  mm = (ss - hh * 3600) / 60;
  ss = (ss - hh * 3600) - mm * 60;
  if (hh < 10)Time += "0";
  Time += (String)hh + ":";
  if (mm < 10)Time += "0";
  Time += (String)mm + ":";
  if (ss < 10)Time += "0";
  Time += (String)ss;
  return Time;
}

void OnUpdateLuxSensor(uint32_t deltaTime)
{
  Serial.println("Update LUX sensor ");
  Tdebug += "Update LUX sensor ";
  Tdebug += "\n\r";
  // Bus 0
  TCA9545.write_control_register(TCA9545_CONFIG_BUS0);
  config = TCA9545_CONFIG_BUS0;
  Serial.print("Bus 0 Control Register:");
  Serial.print(config & 0x0F, HEX);

  uint32_t lum = tsl.getFullLuminosity();
  //  uint32_t lum = 0;
  ir = lum >> 16;
  full = lum & 0xFFFF;
  visible = full - ir;
  //lux = 0;
  lux = tsl.calculateLux(full, ir);

  Serial.print(F("IR: ")); Serial.print(ir);   Serial.print(F("\t\t"));
  Serial.print(F("Full: ")); Serial.print(full);   Serial.print(F("\t"));
  Serial.print(F("Visible: ")); Serial.print(full - ir);   Serial.print(F("\t"));
  Serial.print(F("Lux: ")); Serial.println(tsl.calculateLux(full, ir));
}

String PrintOutside () {

  String Tnetstring = " Outside Status ";
  Tnetstring += " C ";
  Tnetstring += outsidesensor.celsius;
  Tnetstring += " F ";
  Tnetstring += outsidesensor.fahrenheit;
  Tnetstring += " %RH ";
  Tnetstring += outsidesensor.humidity;
  Tnetstring += " Ha in ";
  Tnetstring += outsidesensor.Mb;
  Tnetstring += " Dew Point ";
  Tnetstring += outsidesensor.DewPoint;
  Tnetstring += " WetBulb C ";
  Tnetstring += outsidesensor.WetBulb;
  Tnetstring += " Altitude ";
  Tnetstring += outsidesensor.Altitude;
  Tnetstring += " SeaLevel Pressure ";
  Tnetstring += outsidesensor.SeaLevelPressure / 100;
  Tnetstring += "\n\r";
  return Tnetstring;
}

String PrintSystem() {

  String TSnetstring = " System Status ";
  TSnetstring += "  C ";
  TSnetstring += systemsensor.celsius;
  TSnetstring += " F ";
  TSnetstring += systemsensor.fahrenheit;
  TSnetstring += " %RH ";
  TSnetstring += systemsensor.humidity;
  TSnetstring += " Ha in ";
  TSnetstring += systemsensor.Mb;
  TSnetstring += " Dew Point ";
  TSnetstring += systemsensor.DewPoint;
  TSnetstring += " WetBulb C ";
  TSnetstring += systemsensor.WetBulb;
  TSnetstring += " Altitude ";
  TSnetstring += systemsensor.Altitude;
  TSnetstring += " SeaLevel Pressure ";
  TSnetstring += systemsensor.SeaLevelPressure / 100;
  TSnetstring += "\n\r";
  return TSnetstring;
}

String PrintHouse() {

  String THnetstring = " House Status ";
  THnetstring += "   C ";
  THnetstring += housesensor.celsius;
  THnetstring += " F ";
  THnetstring += housesensor.fahrenheit;
  THnetstring += " %RH ";
  THnetstring += housesensor.humidity;
  THnetstring += " Ha in ";
  THnetstring += housesensor.Mb;
  THnetstring += " Dew Point ";
  THnetstring += housesensor.DewPoint;
  THnetstring += " WetBulb C ";
  THnetstring += housesensor.WetBulb;
  THnetstring += " Altitude ";
  THnetstring += housesensor.Altitude;
  THnetstring += " SeaLevel Pressure ";
  THnetstring += housesensor.SeaLevelPressure / 100;
  THnetstring += "\n\r";
  return THnetstring;
}

String PrintLight() {

  String Tlnetstring = " Full Sensor ";
  Tlnetstring += "    Full Light ";
  Tlnetstring += full;
  Tlnetstring += "   Visible Light ";
  Tlnetstring += visible;
  Tlnetstring += "   Infrared ";
  Tlnetstring += ir;
  Tlnetstring += "   Lux ";
  Tlnetstring += lux;
  Tlnetstring += "\n\r";
  //  t_serverClients[i].println(Tlnetstring);
  //  t_serverClients[i].println(" ");
  return Tlnetstring;
}


String PrintWater() {

  String TWnetstring = " Water Status   ";
  TWnetstring += "   Temp of left ";
  TWnetstring += core2Thermometer ->avgTemp;
  TWnetstring += "   Temp of right ";
  TWnetstring += core1Thermometer->avgTemp;
  TWnetstring += "   Temp of chiller ";
  TWnetstring += chiiler_temp;
  TWnetstring += "   Water Volume ";
  TWnetstring += volume;
  TWnetstring += "\n\r";
  //  t_serverClients[i].println(TWnetstring);
  //  t_serverClients[i].println(" ");
  return TWnetstring;
}

String PrintTime() {

  String TTnetstring = "Current Time:  ";
  TTnetstring += Print_time(ht_time, mt_time, st_time);
  TTnetstring += "\n\r";
  //  t_serverClients[i].println(TTnetstring);
  TTnetstring += "Runtime Time:  ";
  TTnetstring += millis2time();
  TTnetstring += "\n\r";
  TTnetstring += ESP.getFreeHeap();
  TTnetstring += "\n\r";
  //  t_serverClients[i].println(TTnetstring);
  //  t_serverClients[i].println(" ");
  return TTnetstring;
}

String PrintSysVars() {

  String TVnetstring = " System Variables ";
  TVnetstring += "Cool_on ";
  TVnetstring += cool_on;
  TVnetstring += "   Cool_off ";
  TVnetstring += cool_off;
  TVnetstring += "   Cool_count ";
  TVnetstring += cool_count;
  TVnetstring += "   cool_off_timer ";
  TVnetstring += cool_timer;
  TVnetstring += "   Override ";
  TVnetstring += systemoverride;
  TVnetstring += "\n\r";
  //  t_serverClients[i].println(TVnetstring);
  //  t_serverClients[i].println(" ");
  return TVnetstring;
}

String PrintButtonStat() {

  String TBnetstring = " Switch Status ";
  TBnetstring += "   Manule ";
  TBnetstring += runState;
  TBnetstring += "   Pump   ";
  TBnetstring += buttonState1;
  TBnetstring += "   A/C    ";
  TBnetstring += buttonState2;
  TBnetstring += "   Blower ";
  TBnetstring += buttonState3;
  TBnetstring += "   Circ   ";
  TBnetstring += buttonState5;
  TBnetstring += "   Water  ";
  TBnetstring += buttonState4;
  TBnetstring += "\n\r";
  //  t_serverClients[i].println(TBnetstring);
  //  t_serverClients[i].println(" ");
  return TBnetstring;
}

String GetOutPut() {
  String output = PrintTime();
  output += "\n\r";
  output += PrintLight();
  output += "\n\r";
  output += PrintHouse();
  output += "\n\r";
  output += PrintOutside();
  output += "\n\r";
  output += PrintSystem();
  output += "\n\r";
  output += PrintButtonStat();
  output += "\n\r";
  output += PrintSysVars();
  output += "\n\r";
  output += PrintWater();
  output += "\n\r";
  return output;
}

void HandelTelNet(uint32_t deltaTime) {
  //lwdWhere = HandelTelNet_task;
  uint8_t i;
  //check if there are any new clients
  if (t_server.hasClient()) {
    for (i = 0; i < MAX_SRV_CLIENTS; i++) {
      //find free/disconnected spot
      if (!t_serverClients[i] || !t_serverClients[i].connected()) {
        if (t_serverClients[i]) t_serverClients[i].stop();
        t_serverClients[i] = t_server.available();
        //->Serial11.print("New client: "); Serial1.print(i);
        t_serverClients[i].println(" hello new client");
        t_serverClients[i].println(" *******************  Menu  *******************");
        t_serverClients[i].println(" 1. Reset the ESP2866 ");
        t_serverClients[i].println(" 2. Display system status");
        t_serverClients[i].println(" 3. Debug output");
        t_serverClients[i].println(" 4. Maunual Control");
        t_serverClients[i].println(" 5.  Recalibrate");
        t_serverClients[i].println(" 6. Exit");
        t_serverClients[i].println(" ");
        t_serverClients[i].println(" Please select an option,   Enter m at any time to redisplay the menu ");
        t_serverClients[i].flush();  // clear input buffer, else you get strange characters
        continue;
      }
    }
    //no free/disconnected spot so reject
    WiFiClient t_serverClient = t_server.available();
    t_serverClient.stop();
  }
  //check clients for data
  for (i = 0; i < MAX_SRV_CLIENTS; i++) {
    if (t_serverClients[i] && t_serverClients[i].connected()) {
      if ((top == 1) && (t_count > old_t_count)) {
        t_serverClients[i].write(27); // ESC
        t_serverClients[i].print("[H"); // cursor to home
        String tinfo = GetOutPut();
        t_serverClients[i].print(tinfo);
        old_t_count = t_count;
      }
      if ((top1 == 1) && (t_count > old_t_count)) {
        t_serverClients[i].print(Tdebug);
        Tdebug = "";
        old_t_count = t_count;
      }
      if (t_serverClients[i].available()) {

        char var_input[5] = {t_serverClients[i].read()};
        t_serverClients[i].flush();
        if ((var_input[0] == 'm') || (var_input[0] == 'q')) {
          top = 0;
          top1 = 0;
          old_t_count = 0;
          t_serverClients[i].write(27);
          t_serverClients[i].print("[2J"); // clear screen
          t_serverClients[i].println(" *******************  Menu  *******************");
          t_serverClients[i].println(" 1. Reset the ESP2866 ");
          t_serverClients[i].println(" 2. Display system status");
          t_serverClients[i].println(" 3. Debug output");
          t_serverClients[i].println(" 4. Maunual Control");
          t_serverClients[i].println(" 5. Recalibrate");
          t_serverClients[i].println(" 6. Exit");
          t_serverClients[i].println(" ");
          t_serverClients[i].println(" Please select an option,   Enter m at any time to redisplay the menu ");
          LineCheck = 0;

        } else if (var_input[0] == '1') {
          t_serverClients[i].println("Stoping tasks");

          t_serverClients[i].println("Restarting the CPU");
          t_serverClients[i].stop();
          ESP.restart();

        } else if (var_input[0] == '2') {
          if (LineCheck == 0) {
            t_serverClients[i].write(27);
            t_serverClients[i].print("[2J"); // clear screen
            LineCheck = 1;
          }
          t_serverClients[i].write(27); // ESC
          t_serverClients[i].print("[H"); // cursor to home
          String info = GetOutPut();
          t_serverClients[i].print(info);
          top = 1;
          old_t_count = t_count;

        } else if (var_input[0] == '3') {
          t_serverClients[i].println(Tdebug);
          t_serverClients[i].println("\n\r");
          top1 = 1;
          old_t_count = t_count;

        } else if (var_input[0] == '4') {
          if (LineCheck == 0) {
            t_serverClients[i].write(27);
            t_serverClients[i].print("[2J"); // clear screen
            LineCheck = 1;
          }
          t_serverClients[i].write(27); // ESC
          t_serverClients[i].print("[H"); // cursor to home
          t_serverClients[i].println("to turn system on, enter 8  to turn system off, enter 9 ");
          t_serverClients[i].print("current status: ");
          t_serverClients[i].println(buttonState1);

        } else if (var_input[0] == '8')  {
          t_serverClients[i].println("Manuly turnning on");
          runState = 1;
          buttonState1 = 0;
          buttonState2 = 0;
          buttonState3 = 0;
          buttonState4 = 0;
          buttonState5 = 0;
          button_status.update = 1;
          t_serverClients[i].println("to turn system on, enter 8  to turn system off, enter 9 ");
          t_serverClients[i].print("current status: ");
          t_serverClients[i].println(runState);

        } else if (var_input[0] == '9') {
          t_serverClients[i].println("Manuly turnning off");
          runState = 0;
          buttonState1 = 0;
          buttonState2 = 0;
          buttonState3 = 0;
          buttonState4 = 0;
          buttonState5 = 0;
          button_status.update = 1;
          t_serverClients[i].println("to turn system on, enter on  to turn system off, enter off ");
          t_serverClients[i].print("current status: ");
          t_serverClients[i].println(runState);

        } else if (var_input[0] == 'h') {
          t_serverClients[i].println("Manuly turnning pump off");
          buttonState1 = 1;
          t_serverClients[i].print("current status: ");
          t_serverClients[i].println(buttonState1);

        } else if (var_input[0] == 'a') {
          runState = 1;

        } else if (var_input[0] == 'd') {
          t_serverClients[i].println("Manuly turnning pump on");
          buttonState1 = 0;
          t_serverClients[i].print("current status: ");
          t_serverClients[i].println(buttonState1);

        } else if (var_input[0] == '5') {

          t_serverClients[i].println("not enabled yet");



        } else if (var_input[0] == '6') {
          t_serverClients[i].stop();


        } else {
          t_serverClients[i].println("Did not usderstand the option entered");
          t_serverClients[i].println(var_input[0]);
        }
      }
    }
  }
  Tdebug = "";
}



bool  scan_i2c_devices()
{
  byte error, address;
  int nDevices;


  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 185; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      Tdebug += "I2C device found at address 0x";
      if (address < 16)
        Serial.print("0");
      Tdebug += "0";
      Serial.print(address, HEX);
      Tdebug += address;
      Serial.println("  !");
      Tdebug += " !";
      Tdebug += "\n\r";
      nDevices++;
      return true;
    }
    else if (error == 4)
    {
      Serial.print("Unknow error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
      return false;
    }
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
    Tdebug += "No I2C devices found\n";
    Tdebug += "\n\r";
    return true;
  }
  else {
    Serial.println("done\n");
    Tdebug += "done";
    Tdebug += "\n\r";
  }

}

void ticker_handler() {
  ticker_reached = true;
}

void loop() {
  //Serial.println("------------------------------");

  if (ticker_reached) {
    ticker_reached = false;
    OnUpdateTime(1);
    yield();
    OnUpdateButtons(1);
    yield();
    OnUpdateTaskUpdateCounter(1);
    yield();
    HandelTelNet(1);
    yield();
    OnUpdateWaterLevel(1);
    yield();
    OnUpdateTaskGetDallasTemp(1);
    yield();
    OnUpdateTaskGetBus_2(1);
    yield();
    OnUpdateTaskGetBus_1(1);
    yield();
    OnUpdateTaskGetBus_1_1(1);
    yield();
    OnUpdateLuxSensor(1);
    yield();
    Serial.print("Task Counter is: ");
    Serial.println(task_count);
    Tdebug += "Task Counter is: ";
    Tdebug += task_count;
    Tdebug += "\n\r";
    task_count++;
  }

/*

  if ((task_count == 60) || (task_count == 120) || (task_count == 180) || (task_count == 240) || (task_count == 300)) {
    OnUpdateTaskGetDallasTemp(1);
    yield();
  }
  if ((task_count == 65) || (task_count == 125) || (task_count == 185) || (task_count == 245) || (task_count == 305)) {
    OnUpdateTaskGetBus_2(1);
    yield();
  }
  if ((task_count == 70) || (task_count == 130) || (task_count == 190) || (task_count == 250) || (task_count == 310)) {
    OnUpdateTaskGetBus_1(1);
    yield();
  }
  if ((task_count == 75) || (task_count == 135) || (task_count == 195) || (task_count == 255) || (task_count == 315)) {
    OnUpdateTaskGetBus_1_1(1);
    yield();
  }
  if ((task_count == 80) || (task_count == 140) || (task_count == 200) || (task_count == 260) || (task_count == 320)) {
    OnUpdateLuxSensor(1);
    yield();
  }

  */
  if  (task_count == 321) {
    OnUpdateTaskGetNTPTime(1);
    if ( WiFi.status() != WL_CONNECTED ) {
      WiFi.disconnect();

      //set up wifi server
      Serial.println ( ". starting wifi" );
      WiFi.mode(WIFI_STA);
      WiFi.begin ( ssid, password );

      IPAddress ip(192, 168, 0, 6);
      IPAddress gateway(192, 168, 0, 1);
      IPAddress subnet(255, 255, 255, 0);
      WiFi.config(ip, gateway, subnet);


      // Wait for connection
      int wifi_count = 0;
      while ( WiFi.status() != WL_CONNECTED ) {
        delay ( 500 );
        Serial.print ( "." );
        wifi_count++;
        if (wifi_count == 10) {
          break;
        }
      }


    }
    task_count = 0;

  }

  ArduinoOTA.handle();

}
