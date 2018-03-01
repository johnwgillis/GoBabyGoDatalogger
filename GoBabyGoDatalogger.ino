// GoBabyGo Datalogger Code
// Create on: February 9, 2018
// Author: John Gillis (jgillis@jgillis.com)

#include <Adafruit_SleepyDog.h>
#include <SoftwareSerial.h>
#include "Adafruit_FONA.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_FONA.h"
#include "mqttConfig.h"

#define DEBUG_MODE 1 // 0 for normal and 1 for debug; when debugging, it won't go into sleep mode

/*************************** FONA Pins ***************************************/

// Default pins for Feather 32u4 FONA
#define FONA_RX  9
#define FONA_TX  8
#define FONA_RST 4
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);

Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

/*************************** Datalogger Pins *********************************/

// Pins specific to the datalogger shield
#define FONA_KEY 12
#define INFRARED_POWER 11
#define PWR_SWITCH A3

/*************************** Datalogger Thresholds ***************************/

#define SLEEP_MODE_DELAY 1000 // delay in ms between checks of the car power state 
#define PWR_OFF_THRES 200 // from 0 to 1023 relative to ground to 3.3V (through voltage divider on datalogger shield)

#define DELAY_BETWEEN_LOOPS 1000 // delay in ms between computation loops

/************ Global State (you don't need to change this!) ******************/

// Setup the FONA MQTT class by passing in the FONA class and MQTT server and login details.
Adafruit_MQTT_FONA mqtt(&fona, "things.ubidots.com", 1883, "A1E-80mievRXk6jxxqypokytwVda5U6dNN", "");

// FONAconnect is a helper function that sets up the FONA and connects to
// the GPRS network. See the fonahelper.cpp tab above for the source!
boolean FONAconnect(const __FlashStringHelper *apn, const __FlashStringHelper *username, const __FlashStringHelper *password);

/****************************** Feeds ***************************************/
// Notice MQTT paths for Cayenne follow the form: v1/<username>/things/<client_id>/data/<channel_id>
#define FEED_SENSOR_PATH_CAYENNE "v1/" MQTT_USERNAME "/things/" MQTT_CLIENT_ID "/data"

// Notice MQTT paths for Ubidots follow the form: v1.6/devices/{LABEL_DEVICE}/{LABEL_VARIABLE}
#define FEED_SENSOR_PATH_UBIDOTS "/v1.6/devices/gobabygo-datalogger/dummy-counter"

// Setup a feed called dummy_counter for publishing on channel 1
Adafruit_MQTT_Publish dummy_counter = Adafruit_MQTT_Publish(&mqtt, FEED_SENSOR_PATH_UBIDOTS, MQTT_QOS_1);

/*************************** Sketch Code ************************************/

// How many transmission failures in a row we're willing to be ok with before reset
uint8_t txfailures = 0;
#define MAXTXFAILURES 10

#define WATCHDOGTIMEOUT 8000 // in ms
#define MQTTCONNECTINITWAIT 5 // in seconds
#define MAXMQTTRETRIES 5 // in times so about (5*2^5+5*2^4+5*2^3+5*2^2+5*2^1+5*2^0)/60 = 5.25 minutes until reset

void setup() {
  // Watchdog is set to 8 seconds
  Watchdog.enable(WATCHDOGTIMEOUT);
  
  Serial.begin(115200);

  Serial.println(F("GoBabyGo FONA MQTT datalogger by John Gillis (jgillis@jgillis.com)"));

  // Enable Infrared Power
  pinMode(INFRARED_POWER, OUTPUT);
  digitalWrite(INFRARED_POWER, HIGH);

  Watchdog.reset();
  
  // Initialise the FONA module
  
  while (! FONAconnect(F(FONA_APN), F(FONA_USERNAME), F(FONA_PASSWORD))) {
    Serial.println("Retrying FONA");
  }

  Serial.println(F("Connected to Cellular!"));

  Watchdog.reset();
  delay(2000);  // wait a few seconds to stabilize connection
  Watchdog.reset();

  
  printIPAddress();
}

uint32_t x=0; // dummy counter

void loop() {
  // Make sure to reset watchdog every loop iteration!
  Watchdog.reset();

  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.
  MQTT_connect();

  Watchdog.reset();
  
  readSensorsAndPublish();
  
  // Check if over MAXTXFAILURES limit
  if( txfailures > MAXTXFAILURES ) {
    // Reset Arduino using the Watchdog
    resetSystemUsingWatchdog();
  }

  Watchdog.reset();

  updateDistanceAndStopCheck();

  Watchdog.reset();

  // Check if sleep mode is needed
  handleSleepMode();

  //Rate limit loop time
  Watchdog.reset();
  delay(DELAY_BETWEEN_LOOPS);
  Watchdog.reset();
}

// Reads the sensors and publishes the data to MQTT and SD card
void readSensorsAndPublish() {
  
  // Now we can publish stuff!
  x++;
  Serial.print(F("\nSending dummy_counter val "));
  Serial.print(x);
  Serial.print("...");
  
  // Format data for Cayenne using the patter: type,unit=value
  // (https://mydevices.com/cayenne/docs/cayenne-mqtt-api/#cayenne-mqtt-api-mqtt-messaging-topics)
  //String dataToPublish = "analog_sensor," + String(x++, 3); // Displays value with 3 decimal places

  String dataToPublish = "{\"value\":" + String(x) + "}";
  Serial.print(dataToPublish.c_str());
  
  if (! dummy_counter.publish(dataToPublish.c_str())) {
    Serial.println(F("Failed"));
    txfailures++;
  } else {
    Serial.println(F("OK!"));
    txfailures = 0;
  }

  // TOOD : fill in with real sensor logging

  // Read Sensors

  // Log to SD Card

  // Log to MQTT
  
  return;
}


// Processes distance data and outputs via relays and LEDs
void updateDistanceAndStopCheck() {
  // TODO

  // Read Distances

  // Read Motor Output Direction

  // Update LEDs

  // Update Relays
  
  return;
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  Watchdog.disable();

  int retrySeconds = MQTTCONNECTINITWAIT;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in "+ String(retrySeconds) +" seconds...");
    mqtt.disconnect();
    
    if( retrySeconds > (MQTTCONNECTINITWAIT * pow(2,MAXMQTTRETRIES) ) ) {
      // Reset due to MAX MQTT RETRIES reached
      resetSystemUsingWatchdog();
    }
    
    delay(retrySeconds*1000);  // wait
    
    retrySeconds = retrySeconds * 2; // Progressive backoff
  }
  Serial.println("MQTT Connected!");
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

// Causes a system reset using the Watchdog
void resetSystemUsingWatchdog() {
  Watchdog.enable(15); // Set watchdog to 15ms
  delay(30); // Delay for 30ms causing a system reset
}

// Prepares for low-power mode by putting the FONA and non-essential sensors to sleep
void prepareForSleep() {
  Watchdog.reset();
  
  // Disconnect MQTT
  mqtt.disconnect();
  
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
}
