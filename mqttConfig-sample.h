// GoBabyGo Datalogger Code - MQTT Config
// Create on: February 9, 2018
// Author: John Gillis (jgillis@jgillis.com)

// Change the following 0 to 1 for use as the real config
#if 0

/************************* MQTT Config ************************************/

#define MQTT_SERVER           "mqtt.mydevices.com"  // Cayenne server name.
#define MQTT_SERVERPORT       1883  // Cayenne IO port.
#define MQTT_USERNAME         ""  // Cayenne username
#define MQTT_PASSWORD         ""  // Cayenne password
#define MQTT_CLIENT_ID        ""  // Cayenne client id


/************************* APN Config *************************************/

// Optionally configure a GPRS APN, username, and password.
// You might need to do this to access your network's GPRS/data
// network.  Contact your provider for the exact APN, username,
// and password values.  Username and password are optional and
// can be removed, but APN is required.
#define FONA_APN       ""
#define FONA_USERNAME  ""
#define FONA_PASSWORD  ""

#endif
