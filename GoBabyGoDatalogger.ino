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
Adafruit_MQTT_FONA mqtt(&fona, MQTT_SERVER, MQTT_SERVERPORT, MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD);

// FONAconnect is a helper function that sets up the FONA and connects to
// the GPRS network. See the fonahelper.cpp tab above for the source!
boolean FONAconnect(const __FlashStringHelper *apn, const __FlashStringHelper *username, const __FlashStringHelper *password);

/****************************** Feeds ***************************************/
// Notice MQTT paths for Ubidots follow the form: v1.6/devices/{LABEL_DEVICE}/{LABEL_VARIABLE}
#define FEED_SENSOR_PATH_UBIDOTS "/v1.6/devices/gobabygo-datalogger"

// Setup a feed for the device
Adafruit_MQTT_Publish device_feed = Adafruit_MQTT_Publish(&mqtt, FEED_SENSOR_PATH_UBIDOTS, MQTT_QOS_1);

/*************************** Sketch Code ************************************/

// How many transmission failures in a row we're willing to be ok with before reset
uint8_t txfailures = 0;

#define WATCHDOGTIMEOUT 8000 // in ms

#define MAXFONARETRIES 1 // max number of retries to connect to FONA
#define MAXMQTTRETRIES 2 // max number of tries to connect to MQTT

bool fonaConnected = false;
bool mqttConnected = false;

struct SensorData {
  uint32_t timestamp = 0; // which is DateTime::unixtime * 1000
  int dummy_counter = 0;
} sensorData;

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
  int fonaConnectAttempts = 0;
  while (!FONAconnect(F(FONA_APN), F(FONA_USERNAME), F(FONA_PASSWORD)) && fonaConnectAttempts < MAXFONARETRIES ) {
    Serial.println("Retrying FONA");
    fonaConnectAttempts++;
  }

  if(fonaConnectAttempts>=MAXFONARETRIES) {
    Serial.println(F("Failed to connect to cellular!"));
    fonaConnected = false;
  } else {
    Serial.println(F("Connected to Cellular!"));
    fonaConnected = true;
  }

  Watchdog.reset();
  delay(1000);  // wait a few seconds to stabilize connection
  Watchdog.reset();

  
  printIPAddress();
}

uint32_t x=0; // dummy counter

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
  // TODO: make this run only on certain iterations
  publishOverCellular();

  // Handle sleep mode
  handleSleepMode();

  // Wait until next loop start time
  Watchdog.reset();
  delay(DELAY_BETWEEN_LOOPS); // TODO: dynamically calculate delay for fixed rate times
  Watchdog.reset();
  
}

// Reads the sensors into the sensorData struct
void readSensors() {
  // TODO

  // "Read" a dummy sensor
  sensorData.dummy_counter++;
  Serial.print(F("\nRead in dummy_counter val "));
  Serial.println(sensorData.dummy_counter);
  
  return;
}

// Writes the latest sensor data onto the SD Card
void writeToSDCard() {
  // TODO
  return;
}

// Updates the state of the braking relays and leds
void updateBrakingFeedback() {
  // TODO

  // Update LEDs

  // Update Relays
  
  return;
}

// Publishes over cellular via MQTT
void publishOverCellular() {
  //TODO

  Watchdog.reset();

  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.
  MQTT_connect();

  Watchdog.reset();

  if(mqttConnected == false) {
    // Not connected to MQTT so don't bother trying to send data
    return;
  }

  Serial.print(F("\nSending dummy_counter val "));
  Serial.print(sensorData.dummy_counter);
  Serial.print("...");

  // Format data for Ubidots using the example pattern: {"temperature":[{"value": 10, "timestamp":1464661369000}, {"value": 12, "timestamp":1464661369999}], "humidity": 50}
  // (https://ubidots.com/docs/api/mqtt.html#publish-values-to-a-device)
  String dataToPublish = "{\"dummy_counter\":" + String(sensorData.dummy_counter) + "}";
  Serial.print(dataToPublish.c_str());
  
  if (! device_feed.publish(dataToPublish.c_str())) {
    Serial.println(F("Failed"));
    txfailures++;
  } else {
    Serial.println(F("OK!"));
    txfailures = 0;
  }

  Watchdog.reset();
  
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
  if(fonaConnected == false) {
    // Fona is not connected to cellular so don't bother trying MQTT
    mqttConnected = false;
    return;
  }
  
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
      mqttConnected = false;
      return;
    }
    
    Watchdog.reset();
    delay(1000);  // wait a second before trying again
    Watchdog.reset();
  }
  Serial.println("MQTT Connected!");
  mqttConnected = true;
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
