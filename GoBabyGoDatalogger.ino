// GoBabyGo Datalogger Code
// Create on: February 9, 2018
// Author: John Gillis (jgillis@jgillis.com)

#define DEBUG_MODE 1 // 0 for normal and 1 for debug; when debugging, it won't go into sleep mode
#define CELLULAR_ENABLE 0 // 1 for normal operations (without NeoPixel) and 0 for disabling cellular use (with NeoPixel)

#include <Adafruit_SleepyDog.h>
#include <SoftwareSerial.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SharpDistSensor.h>

#include <SPI.h>
#include <SD.h>

#include <Wire.h>
#include "RTClib.h"

#if CELLULAR_ENABLE
#include "Adafruit_FONA.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_FONA.h"
#include "mqttConfig.h"

#else
#include <Adafruit_NeoPixel.h>

#endif

/*************************** FONA Pins ***************************************/
#if CELLULAR_ENABLE

// Default pins for Feather 32u4 FONA
#define FONA_RX  9
#define FONA_TX  8
#define FONA_RST 4
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);

Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

#endif

/*************************** Datalogger Pins *********************************/

// Pins specific to the datalogger shield
#define FONA_KEY 12
#define INFRARED_POWER 12
#define PWR_SWITCH A3

#define STAIR_SENSOR_PIN A10
#define STAIR_CUTOFF_DISTANCE 200 // will cut off power via relays when the distance is greater than this (in mm)
#define WALL_SENSOR_PIN A5
#define WALL_LED_MAP_MAX 10 // distance (in mm) in which the led display will begin display
#define WALL_LED_MAP_MIN 1 // distance (in mm) in which the led display will be at full display
#define MEDIUM_FILTER_WINDOW_SIZE 5 // Window size of the median filter (odd number, 1 = no filtering)

#define RELAYS_PIN A4

#define NEOPIXEL_PIN 11

#define BNO055_IMU_ADDRESS 55

#define SD_CARD_CHIP_SELECT_PIN 5
#define DATA_LOGGER_FILE "datalog.csv" // string name of the csv file to write on the SD Card

/*************************** Datalogger Thresholds ***************************/

#define SLEEP_MODE_DELAY 1000 // delay in ms between checks of the car power state 
#define PWR_OFF_THRES 200 // from 0 to 1023 relative to ground to 3.3V (through voltage divider on datalogger shield)

#define DELAY_BETWEEN_LOOPS 20 // delay in ms between computation loops

/************ Global State (you don't need to change this!) ******************/
#if CELLULAR_ENABLE

// Setup the FONA MQTT class by passing in the FONA class and MQTT server and login details.
Adafruit_MQTT_FONA mqtt(&fona, MQTT_SERVER, MQTT_SERVERPORT, MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD);

// FONAconnect is a helper function that sets up the FONA and connects to
// the GPRS network. See the fonahelper.cpp tab above for the source!
boolean FONAconnect(const __FlashStringHelper *apn, const __FlashStringHelper *username, const __FlashStringHelper *password);

/****************************** Feeds ***************************************/
// Notice MQTT paths for Ubidots follow the form: v1.6/devices/{LABEL_DEVICE}/{LABEL_VARIABLE}
#define FEED_SENSOR_PATH_UBIDOTS "/v1.6/devices/gobabygo-datalogger"

// Setup a feed for the device
Adafruit_MQTT_Publish device_feed = Adafruit_MQTT_Publish(&mqtt, FEED_SENSOR_PATH_UBIDOTS, MQTT_QOS_1);

#endif

/*************************** Sensor Variables *******************************/

Adafruit_BNO055 bno = Adafruit_BNO055(BNO055_IMU_ADDRESS);

// Create an object instance of the SharpDistSensor class for stair and wall sensors
SharpDistSensor stairSensor(STAIR_SENSOR_PIN, MEDIUM_FILTER_WINDOW_SIZE);
SharpDistSensor wallSensor(WALL_SENSOR_PIN, MEDIUM_FILTER_WINDOW_SIZE);

#if !(CELLULAR_ENABLE)

#define PIXEL_COUNT 16
Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

#endif

struct DevicesConnected {
  bool fona = false;
  bool mqtt = false;
  bool sdcard = false;
  bool rtc = false;
  bool imu = false;
} devicesConnected;

struct SensorData {
  uint32_t timestamp = 0; // which is DateTime::unixtime * 1000
  unsigned long uptime = 0; // which is the numeber of milliseconds since last reboot

  double quaternion_w, quaternion_x, quaternion_y, quaternion_z = 0;

  unsigned int stair_distance, wall_distance = 0;
} sensorData;

File datalogFile;

RTC_PCF8523 realTimeClock;

/*************************** Sketch Code ************************************/

// How many transmission failures in a row we're willing to be ok with before reset
uint8_t txfailures = 0;

#define WATCHDOGTIMEOUT 8000 // in ms

#define MAXFONARETRIES 1 // max number of retries to connect to FONA
#define MAXMQTTRETRIES 2 // max number of tries to connect to MQTT

#define SAMPLE_PERIOD 100 // Desired sample period in ms for the SD card
#define CELLULAR_PERIOD 2000 // Desired sample period in ms
uint32_t last_sample_time = 0;
uint32_t last_cellular_time = 0;


void setup() {
  // Watchdog is set to 8 seconds
  Watchdog.enable(WATCHDOGTIMEOUT);
  
  Serial.begin(115200);

  Serial.println(F("GoBabyGo datalogger by John Gillis (jgillis@jgillis.com)"));
  
  // Enable Infrared Power
  pinMode(INFRARED_POWER, OUTPUT);
  digitalWrite(INFRARED_POWER, HIGH);

#if CELLULAR_ENABLE
  // Enable Fona Key Power
  pinMode(FONA_KEY, OUTPUT);
  digitalWrite(FONA_KEY, HIGH);

  Watchdog.reset();
  
  // Initialise the FONA module
  int fonaConnectAttempts = 0;
  while (!FONAconnect(F(FONA_APN), F(FONA_USERNAME), F(FONA_PASSWORD)) && fonaConnectAttempts < MAXFONARETRIES ) {
    Serial.println("Retrying FONA");
    fonaConnectAttempts++;
  }

  if(fonaConnectAttempts>=MAXFONARETRIES) {
    Serial.println(F("Failed to connect to cellular!"));
    devicesConnected.fona = false;
  } else {
    Serial.println(F("Connected to Cellular!"));
    devicesConnected.fona = true;
  }

  Watchdog.reset();
  delay(1000);  // wait a few seconds to stabilize connection
  Watchdog.reset();

#else

  // Initialize all pixels to 'off'
  strip.begin();
  strip.show();
  
#endif

  // Init the SD Card
  // see if the card is present and can be initialized:
  if (!SD.begin(SD_CARD_CHIP_SELECT_PIN)) {
    Serial.println("SD card failed or not present.");
    devicesConnected.sdcard = false;
  } else {
    Serial.println("SD card initialized.");
    
    // Init datalog file
    if(!SD.exists(DATA_LOGGER_FILE)) {
      Serial.println("Creating data logger file...");
      
      // Create the file and add the header row
      datalogFile = SD.open(DATA_LOGGER_FILE, FILE_WRITE);
      datalogFile.println("timestamp,uptime,quaternion_w,quaternion_x,quaternion_y,quaternion_z,stair_distance,wall_distance"); // header row
      datalogFile.close();

      // Check that the file was created successfully
      if(SD.exists(DATA_LOGGER_FILE)) {
        Serial.println("Data logger file ready.");
        devicesConnected.sdcard = true;
      } else {
        Serial.println("Couldn't create data logger file.");
        devicesConnected.sdcard = false;
      }
      
    } else {
      // Already created and ready to go
      Serial.println("Data logger file ready.");
      devicesConnected.sdcard = true;
    }
  }

  // Init the real time clock
  realTimeClock.begin();
  // Check that the real time clock is actually connected
  Wire.beginTransmission(PCF8523_ADDRESS);
  byte error = Wire.endTransmission();
  if(error != 0) {
    Serial.println("Couldn't find RTC");
    devicesConnected.rtc = false;
  } else {
    if (!realTimeClock.initialized()) {
      Serial.println("RTC is NOT running!");
      // following line sets the RTC to the date & time this sketch was compiled
      // Note: the new adjusted time will be off of the true unix time by the time zone of the compiler
      realTimeClock.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
    Serial.println("RTC ready.");
    devicesConnected.rtc = true;
  }

  // Init the sensors
  // TODO : finsih all sensors init
  if(!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("No BNO055 detected!");
    devicesConnected.imu = false;
  } else {
    Serial.print("IMU connected!");
    devicesConnected.imu = true;
  }

  // Init stair and wall infrared sensors and relays
  stairSensor.setModel(SharpDistSensor::GP2Y0A60SZLF_5V);
  wallSensor.setModel(SharpDistSensor::GP2Y0A60SZLF_5V);
  pinMode(RELAYS_PIN, OUTPUT);
  digitalWrite(RELAYS_PIN, LOW);


  // Print out the IP Address for debugging
  printIPAddress();
}

void loop() {
  // Make sure to reset watchdog every loop iteration!
  Watchdog.reset();

  // Read all sensors
  readSensors();

  // Write to SD Card
  writeToSDCard();

  // Update braking feedback
  updateBrakingFeedback();

  // Publish over Cellular
  publishOverCellular();

  // Handle sleep mode
  handleSleepMode();

  // Wait until next loop start time
  Watchdog.reset();
  delay(DELAY_BETWEEN_LOOPS);
  Watchdog.reset();
}

// Reads the sensors into the sensorData struct
void readSensors() {
  // Read the current time from real time clock
  if(devicesConnected.rtc) {
    sensorData.timestamp = (uint32_t)( realTimeClock.now().unixtime() ); // time (in seconds) since midnight 1/1/1970
  } else {
    sensorData.timestamp = (uint32_t)( (DateTime(F(__DATE__), F(__TIME__)) + TimeSpan((int32_t)(millis()/1000))).unixtime() ); // time adjusted from at compile since last boot (in seconds) since midnight 1/1/1970
  }

  // Read uptime
  sensorData.uptime = millis();

  // Read IMU
  if(devicesConnected.imu) {
    imu::Quaternion currentQuat = bno.getQuat();
    sensorData.quaternion_w = currentQuat.w();
    sensorData.quaternion_x = currentQuat.x();
    sensorData.quaternion_y = currentQuat.y();
    sensorData.quaternion_z = currentQuat.z();
  }

  // Read stair and wall distance sensors
  sensorData.stair_distance = stairSensor.getDist();
  sensorData.wall_distance = wallSensor.getDist();

  // TODO
  
  return;
}

// Writes the latest sensor data onto the SD Card
void writeToSDCard() {
  if((millis() - last_sample_time) > SAMPLE_PERIOD) {

    if(devicesConnected.sdcard == false) {
      // Not connected to SD Card and file correctly so don't bother trying to write data
      return;
    }
    
    // Write sensor data to SD card
    Serial.print("Writing to the SD card...");

    datalogFile = SD.open(DATA_LOGGER_FILE, FILE_WRITE);
    String dataToWrite = "";


    // timestamp
    dataToWrite += String(sensorData.timestamp) + ",";
    // uptime
    dataToWrite += String(sensorData.uptime) + ",";

    // quaternion_w
    dataToWrite += String(sensorData.quaternion_w, 8) + ",";
    // quaternion_x
    dataToWrite += String(sensorData.quaternion_x, 8) + ",";
    // quaternion_y
    dataToWrite += String(sensorData.quaternion_y, 8) + ",";
    // quaternion_z
    dataToWrite += String(sensorData.quaternion_z, 8) + ",";

    // stair_distance
    dataToWrite += String(sensorData.stair_distance) + ",";
    // wall_distance
    dataToWrite += String(sensorData.wall_distance);
    
    
    datalogFile.println(dataToWrite); // header row
    datalogFile.close();

    last_sample_time = millis();
  }
  
  return;
}

// Updates the state of the braking relays and leds
void updateBrakingFeedback() {
  // Update Relays and LEDs
  if(sensorData.stair_distance > STAIR_CUTOFF_DISTANCE) {
    digitalWrite(RELAYS_PIN, HIGH);
    #if !(CELLULAR_ENABLE)
    colorWipeLeds(strip.Color(255, 0, 0), 5);  // Red
    #endif
  } else {
    digitalWrite(RELAYS_PIN, LOW);
    #if !(CELLULAR_ENABLE)
    colorWipeLeds(strip.Color(0, 0, 0), 5);    // Black/off
    #endif
  }
  
  return;
}

// Publishes over cellular via MQTT
void publishOverCellular() {
  #if CELLULAR_ENABLE
  
  if(!((millis() - last_cellular_time) > CELLULAR_PERIOD)) {
    return;
  }

  if(devicesConnected.fona == false) {
    // Not connected to FONA so don't bother trying to send data
    return;
  }


  Watchdog.reset();

  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.
  MQTT_connect();

  Watchdog.reset();

  if(devicesConnected.mqtt == false) {
    // Not connected to MQTT so don't bother trying to send data
    return;
  }

  // TODO: add in client based timestamp sending according to Ubidots documentation

  // Format data for Ubidots using the example pattern: {"temperature":[{"value": 10, "timestamp":1464661369000}, {"value": 12, "timestamp":1464661369999}], "humidity": 50}
  // (https://ubidots.com/docs/api/mqtt.html#publish-values-to-a-device)
  String dataToPublish = "{:";

  // Load in sensors
  
  if(devicesConnected.imu) {
    dataToPublish += "\"quaternion_w\":" + String(sensorData.quaternion_w, 8) + ",";
    dataToPublish += "\"quaternion_x\":" + String(sensorData.quaternion_x, 8) + ",";
    dataToPublish += "\"quaternion_y\":" + String(sensorData.quaternion_y, 8) + ",";
    dataToPublish += "\"quaternion_z\":" + String(sensorData.quaternion_z, 8) + ",";
  }

  dataToPublish += "\"stair_distance\":" + String(sensorData.stair_distance) + ",";
  dataToPublish += "\"wall_distance\":" + String(sensorData.wall_distance);
  
  dataToPublish += "}";
  Serial.print(dataToPublish.c_str());

  #if CELLULAR_ENABLE
  if (! device_feed.publish(dataToPublish.c_str())) {
    Serial.println(F("Failed"));
    txfailures++;
    devicesConnected.mqtt = false;
  } else {
    Serial.println(F("OK!"));
    txfailures = 0;
  }
  #endif

  Watchdog.reset();

  last_cellular_time = millis();

  #endif
  
  return;
}

// Checks if the car is off and enters a low-power sleep mode (which takes about 5 seconds)
// When car is turned back on, the function will exit the sleep mode and reset the code (about 20 seconds)
void handleSleepMode() {
  // Need to feed the watchdog throughout (except for when low-power sleeping)
  Watchdog.reset();

  // TODO
  // Check for Big Button inactivity

  // If car is off
  if( analogRead(PWR_SWITCH) < PWR_OFF_THRES && ( !(DEBUG_MODE) )) {

    Serial.println("Entering LOW POWER SLEEP MODE!");
    
    // Prepare for sleep mode
    prepareForSleep();
    
    // While car is off
    while( analogRead(PWR_SWITCH) < PWR_OFF_THRES ) {
      // Low-power sleep for the defined time constant
      Watchdog.sleep(SLEEP_MODE_DELAY);
    }

    // Reset Arduino using the Watchdog (to start back up the FONA and MQTT)
    resetSystemUsingWatchdog();
    
  }

  // Will never reach here since the system is reset
  return;
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  if(devicesConnected.fona == false) {
    // Fona is not connected to cellular so don't bother trying MQTT
    devicesConnected.mqtt = false;
    return;
  }

  #if CELLULAR_ENABLE
  
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  Watchdog.reset();

  int mqttConnectionAttempts = 0;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in one second...");
    mqtt.disconnect();
    mqttConnectionAttempts++;

    if(mqttConnectionAttempts >= MAXMQTTRETRIES) {
      Serial.println("MQTT connection failed!");
      devicesConnected.mqtt = false;
      return;
    }
    
    Watchdog.reset();
    delay(1000);  // wait a second before trying again
    Watchdog.reset();
  }
  Serial.println("MQTT Connected!");
  devicesConnected.mqtt = true;

  #endif
}

// Causes a system reset using the Watchdog
void resetSystemUsingWatchdog() {
  Watchdog.enable(15); // Set watchdog to 15ms
  delay(30); // Delay for 30ms causing a system reset
}

// Prepares for low-power mode by putting the FONA and non-essential sensors to sleep
void prepareForSleep() {
  Watchdog.reset();
  
  // Disconnect MQTT
  #if CELLULAR_ENABLE
    mqtt.disconnect();
  #endif
  
  delay(1000);  // wait 1 second
  Watchdog.reset();
  
  // Turn off FONA
  digitalWrite(FONA_KEY, LOW);
  delay(2000);
  digitalWrite(FONA_KEY, HIGH);
  Watchdog.reset();
  delay(2000);
  digitalWrite(FONA_KEY, LOW);
  Watchdog.reset();
  
  // Disable Infrared Power
  digitalWrite(INFRARED_POWER, LOW);
  
  // Disable IMU
  // - not done since it automatically goes into pretty good low power mode
  // - however, the datasheet references a suspend mode
  //     which is more efficent but not exposed through the library
}

// Prints the public IP Address for debugging
void printIPAddress() {
    #if CELLULAR_ENABLE
    
    // read website URL
        uint16_t statuscode;
        int16_t length;
        char url[80] = "dynamicdns.park-your-domain.com/getip";

        //flushSerial();
        //Serial.println(F("NOTE: in beta! Use small webpages to read!"));
        //Serial.println(F("URL to read (e.g. www.adafruit.com/testwifi/index.html):"));
        //Serial.print(F("http://")); readline(url, 79);
        Serial.println(url);

        Serial.println(F("****"));
        if (!fona.HTTP_GET_start(url, &statuscode, (uint16_t *)&length)) {
          Serial.println("Failed!");
          return;
        }
        while (length > 0) {
          while (fona.available()) {
            char c = fona.read();

            // Serial.write is too slow, we'll write directly to Serial register!
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
            loop_until_bit_is_set(UCSR0A, UDRE0); /* Wait until data register empty. */
            UDR0 = c;
#else
            Serial.write(c);
#endif
            length--;
            if (! length) break;
          }
        }
        Serial.println(F("\n****"));
        fona.HTTP_GET_end();

        #endif
}

#if !(CELLULAR_ENABLE)
// Fill the dots one after the other with a color
void colorWipeLeds(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}
#endif
