////////////////////////////////////////////////
//                                            //
//  CREATED BY  : Jérémy Noverraz             //
//  CREATED ON  : 03.02.2022                  //
//  VERSION     : 1.0.8                       //                                      
//  DESCRIPTION : BME680 bsec data over MQTT  //
//  LICENCE : GNU                             //
//                                            //
////////////////////////////////////////////////

#include <Arduino.h>                        //Arduino library
#include <WiFi.h>                           // WiFi library
extern "C" {                                // for the bsec_init() function
  #include "freertos/FreeRTOS.h"            // for the FreeRTOS API
  #include "freertos/timers.h"              // for the FreeRTOS timers
}
#include <AsyncMqttClient.h>
#include <Wire.h>
#include "bsec.h"
#include <SPI.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>



#define WIFI_SSID "SwissLabsBox2"  // WiFi SSID
#define WIFI_PASS "JP3YMhAdx4rbvyru3S"  // WPA2 password

//Mosquitto broker on HomeAssistant
#define MQTT_HOST IPAddress(192,168,1,4) //IP of the MQTT broker
#define MQTT_PORT 1883 //Port of the MQTT broker
#define MQTT_USER "helitekmqttuser" //MQTT user
#define MQTT_PASS "W3lc0m32h3l1t3k" //MQTT password

//Temperature MQTT topics
#define TEMP_TOPIC "helitek/sensor/temperature/temperature" //MQTT topic for temperature
#define TEMP_STATE_TOPIC "helitek/sensor/temperature/state" //MQTT topic for temperature state

//Humidity MQTT topics
#define HUM_TOPIC "helitek/sensor/humidity/humidity" //MQTT topic for humidity
#define HUM_STATE_TOPIC "helitek/sensor/humidity/state" //MQTT topic for humidity state

//Pressure MQTT topics
#define PRESS_TOPIC "helitek/sensor/pressure/pressure"
#define PRESS_STATE_TOPIC "helitek/sensor/pressure/state"

//GasResistance MQTT topics
#define GAS_TOPIC "helitek/sensor/gas/gas"
#define GAS_STATE_TOPIC "helitek/sensor/gas/state"

//CO2 MQTT topics
#define CO2_TOPIC "helitek/sensor/co2/co2"
#define CO2_STATE_TOPIC "helitek/sensor/co2/state"

//BVOC_TOPIC MQTT topics
#define BVOC_TOPIC "helitek/sensor/bvoc/bvoc"
#define BVOC_STATE_TOPIC "helitek/sensor/bvoc/state"

//IAQ MQTT topics
#define IAQ_TOPIC "helitek/sensor/iaq/iaq"
#define IAQ_STATE_TOPIC "helitek/sensor/iaq/state"

//IAQ_ACCURACY MQTT topics
#define IAQ_ACCURACY_TOPIC "helitek/sensor/iaq/accuracy"
#define IAQ_ACCURACY_STATE_TOPIC "helitek/sensor/iaq/accuracy/state"

//IAQ_STATIC MQTT topics
#define IAQ_STATIC_TOPIC "helitek/sensor/iaq/static"
#define IAQ_STATIC_STATE_TOPIC "helitek/sensor/iaq/static/state"

//Raw_temperature MQTT topics
#define RAW_TEMP_TOPIC "helitek/sensor/raw_temperature/raw_temperature"
#define RAW_TEMP_STATE_TOPIC "helitek/sensor/raw_temperature/state"

//Raw_humidity MQTT topics
#define RAW_HUM_TOPIC "helitek/sensor/raw_humidity/raw_humidity"
#define RAW_HUM_STATE_TOPIC "helitek/sensor/raw_humidity/state"

// BULTIN LED PIN
#define LED_BUILTIN 2


//Helpers functions declaration
void checkIaqSensorStatus(void);
void errLeds(void);

// Create an object of the class bsec
Bsec iaqSensor;

// String output;
String output;

//BME680 sensor settings
#define BME680_I2C_BUS 0 // I2C bus
#define BME680_I2C_SPEED 400000 // I2C speed
#define BME680_I2C_TIMEOUT 1000 // I2C timeout


//BME680 sensor calibration data
#define BME680_CAL_DATA_SIZE 32 // Calibration data size

// MQTT client
AsyncMqttClient mqttClient; // Create AsyncMqttClient object from AsyncMqttClient.h class
TimerHandle_t mqttReconnectTimer; // MQTT reconnect timer
TimerHandle_t wifiReconnectTimer; // Wifi reconnect timer

// Some variables
unsigned long previousMillis = 0; // will store last time temperature was updated
const long interval = 3000; // interval at which to publish a temperature reading (in ms)
const float temperatureOffset = 3.00; // temperature offset for calibration purposes (in °C) 


void connectToWifi() {
  // Connect to WiFi access point.
  // If you want to connect to a network with an existing name, try
  //     WiFi.begin(ssid, pass);
  // If you want to connect to a network with an unknown name, try
  //     WiFi.begin(ssid);
  // If the network is unknown, it will be created with the supplied
  //     password.
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
}

// OTA functions
void connectToOTA(){
  // Connect to OTA server
  ArduinoOTA.setHostname("Helitek-BME680");
  ArduinoOTA.setPasswordHash("5f4dcc3b5aa765d61d8327deb882cf99");
  ArduinoOTA.setMdnsEnabled(true);
  ArduinoOTA.setPort(3232);
  ArduinoOTA.setTimeout(20000);
  ArduinoOTA.begin();
  Serial.println("OTA ready");
}

void connectToMqtt(){
  Serial.println("Connecting to MQTT...");
  while (!mqttClient.connected()) {
    Serial.print(".");
    delay(500);
    mqttClient.connect();
  }

}

void WifiEvent(WiFiEvent_t  event){
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch(event){
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

// MQTT event callback
void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

// MQTT event callback
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}
// MQTT event callback
void onMqttPublish(uint16_t packetId) {
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}
// MQTT event callback
void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}
// MQTT event callback
void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void setup() { // Setup function
  //Serial begin 9600
  Serial.begin(9600);
  Serial.println();

  //Wire begin
  Wire.begin();

  iaqSensor.begin(BME680_I2C_ADDR_SECONDARY, Wire);

  //Check if sensor is connected and working properlyand print the BSEC library version
  output = "\nBSEC library version " + String(iaqSensor.version.major) + "." + String(iaqSensor.version.minor) + "." + String(iaqSensor.version.major_bugfix) + "." + String(iaqSensor.version.minor_bugfix); // Print library version
  Serial.println(output); // Print library version
  checkIaqSensorStatus(); // Check sensor status

  //Set sensor settings
  bsec_virtual_sensor_t sensorList[10] = {
    BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_RAW_HUMIDITY,
    BSEC_OUTPUT_RAW_GAS,
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_STATIC_IAQ,
    BSEC_OUTPUT_CO2_EQUIVALENT,
    BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
  };

  iaqSensor.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_LP);
  checkIaqSensorStatus();

  // Set up the timer that will reconnect to MQTT
  mqttReconnectTimer = xTimerCreate("mqttReconnectTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt)); // Create a timer that will be used to reconnect to MQTT.  Set the timer to run every 2 seconds.
  wifiReconnectTimer = xTimerCreate("wifiReconnectTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));// Create a timer that will be used to reconnect to Wi-Fi.  Set the timer to run every 2 seconds.

  // Set up the WiFi event handler
  WiFi.onEvent(WifiEvent); // Register the WiFi event handler

  mqttClient.onConnect(onMqttConnect); // Register MQTT connect handler
  mqttClient.onDisconnect(onMqttDisconnect); // Register MQTT disconnect handler
  mqttClient.onPublish(onMqttPublish); // Register MQTT publish handler
  mqttClient.onSubscribe(onMqttSubscribe); // Register MQTT subscribe handler
  mqttClient.onUnsubscribe(onMqttUnsubscribe); //

  
  
  mqttClient.setServer(MQTT_HOST, MQTT_PORT); // Set MQTT server and port
  mqttClient.setCredentials(MQTT_USER, MQTT_PASS);  // Set MQTT credentials
  
  connectToWifi(); // Connect to Wi-Fi
  connectToOTA(); // Connect to OTA server

  // Print the header
  output = "Timestamp [ms], raw temperature [°C], pressure [hPa], raw relative humidity [%], gas [Ohm], IAQ, IAQ accuracy, temperature [°C], relative humidity [%], Static IAQ, CO2 equivalent, breath VOC equivalent";
  
}

void loop() {
  //Handle OTA
  ArduinoOTA.handle(); //OTA handling
  unsigned long time_trigger = millis();
  if (iaqSensor.run()) { // If new data is available
    output = String(time_trigger);
    output += ", " + String(iaqSensor.rawTemperature);
    output += ", " + String(iaqSensor.pressure);
    output += ", " + String(iaqSensor.rawHumidity);
    output += ", " + String(iaqSensor.gasResistance);
    output += ", " + String(iaqSensor.iaq);
    output += ", " + String(iaqSensor.iaqAccuracy);
    output += ", " + String(iaqSensor.temperature);
    output += ", " + String(iaqSensor.humidity);
    output += ", " + String(iaqSensor.staticIaq);
    output += ", " + String(iaqSensor.co2Equivalent);
    output += ", " + String(iaqSensor.breathVocEquivalent);
  } else {
    checkIaqSensorStatus();
  }
    // Delay of 500ms to avoid flooding the serial port before MQTT connection is established
    delay(500); // Wait for 500ms before next reading


    //Publish an MQTT message on the topic temperature
    uint16_t packetIdPub1 = mqttClient.publish(TEMP_TOPIC, 1, true, String(iaqSensor.temperature - temperatureOffset).c_str());
    Serial.printf("Publish at QOS 1 on topic : %s, the packetId: %i\n", TEMP_TOPIC, packetIdPub1);
    Serial.printf("Message: %.2f \n", iaqSensor.temperature);

    //Publish an MQTT message on the topic humidity
    uint16_t packetIdPub2 = mqttClient.publish(HUM_TOPIC, 1, true, String(iaqSensor.humidity).c_str());
    Serial.printf("Publish at QOS 1 on topic : %s, the packetId: %i\n", HUM_TOPIC, packetIdPub2);
    Serial.printf("Message: %.2f \n", iaqSensor.humidity);

    //Publish an MQTT message on the topic pressure
    uint16_t packetIdPub3 = mqttClient.publish(PRESS_TOPIC, 1, true, String(iaqSensor.pressure).c_str());
    Serial.printf("Publish at QOS 1 on topic : %s, the packetId: %i\n", PRESS_TOPIC, packetIdPub3);
    Serial.printf("Message: %.2f \n", iaqSensor.pressure);

    //Publish an MQTT message on the topic gas_resistance
    uint16_t packetIdPub4 = mqttClient.publish(GAS_TOPIC, 1, true, String(iaqSensor.gasResistance).c_str());
    Serial.printf("Publish at QOS 1 on topic : %s, the packetId: %i\n", GAS_TOPIC, packetIdPub4);
    Serial.printf("Message: %.2f \n", iaqSensor.gasResistance);
    
    //Publish an MQTT message on the topic co2_equivalent
    uint16_t packetIdPub5 = mqttClient.publish(CO2_TOPIC, 1, true, String(iaqSensor.co2Equivalent).c_str());
    Serial.printf("Publish at QOS 1 on topic : %s, the packetId: %i\n", CO2_TOPIC, packetIdPub5);
    Serial.printf("Message: %.2f \n", iaqSensor.co2Equivalent);

    //Publish an MQTT message on the topic breath_voc_equivalent
    uint16_t packetIdPub6 = mqttClient.publish(BVOC_TOPIC, 1, true, String(iaqSensor.breathVocEquivalent).c_str());
    Serial.printf("Publish at QOS 1 on topic : %s, the packetId: %i\n", BVOC_TOPIC, packetIdPub6);
    Serial.printf("Message: %.2f \n", iaqSensor.breathVocEquivalent);

    // Publish an MQTT message on the topic IAQ
    uint16_t packetIdPub7 = mqttClient.publish(IAQ_TOPIC, 1, true, String(iaqSensor.iaq).c_str());
    Serial.printf("Publish at QOS 1 on topic : %s, the packetId: %i\n", IAQ_TOPIC, packetIdPub7);
    Serial.printf("Message: %.2f \n", iaqSensor.iaq);

    // Publish an MQTT message on the topic IAQ_accuracy
    uint16_t packetIdPub8 = mqttClient.publish(IAQ_ACCURACY_TOPIC, 1, true, String(iaqSensor.iaqAccuracy).c_str());
    Serial.printf("Publish at QOS 1 on topic : %s, the packetId: %i\n", IAQ_ACCURACY_TOPIC, packetIdPub8);
    Serial.printf("Message: %d \n", iaqSensor.iaqAccuracy);

    // Publish an MQTT message on the topic raw_temperature
    uint16_t packetIdPub9 = mqttClient.publish(RAW_TEMP_TOPIC, 1, true, String(iaqSensor.rawTemperature).c_str());
    Serial.printf("Publish at QOS 1 on topic : %s, the packetId: %i\n", RAW_TEMP_TOPIC, packetIdPub9);
    Serial.printf("Message: %.2f \n", iaqSensor.rawTemperature);

    // Publish an MQTT message on the topic raw_humidity
    uint16_t packetIdPub10 = mqttClient.publish(RAW_HUM_TOPIC, 1, true, String(iaqSensor.rawHumidity).c_str());
    Serial.printf("Publish at QOS 1 on topic : %s, the packetId: %i\n", RAW_HUM_TOPIC, packetIdPub10);
    Serial.printf("Message: %.2f \n", iaqSensor.rawHumidity);

    // Publish an MQTT message on the topic iaq_static
    uint16_t packetIdPub11 = mqttClient.publish(IAQ_STATIC_TOPIC, 1, true, String(iaqSensor.staticIaq).c_str());
    Serial.printf("Publish at QOS 1 on topic : %s, the packetId: %i\n", IAQ_STATIC_TOPIC, packetIdPub11);
    Serial.printf("Message: %.2f \n", iaqSensor.staticIaq);


    
    

  }
// Helper function definitions
void checkIaqSensorStatus(void)
{
  if (iaqSensor.status != BSEC_OK) {
    if (iaqSensor.status < BSEC_OK) {
      output = "BSEC error code : " + String(iaqSensor.status);
      Serial.println(output);
      for (;;)
        errLeds(); /* Halt in case of failure */
    } else {
      output = "BSEC warning code : " + String(iaqSensor.status);
      Serial.println(output);
    }
  }

  if (iaqSensor.bme680Status != BME680_OK) {
    if (iaqSensor.bme680Status < BME680_OK) {
      output = "BME680 error code : " + String(iaqSensor.bme680Status);
      Serial.println(output);
      for (;;)
        errLeds(); /* Halt in case of failure */
    } else {
      output = "BME680 warning code : " + String(iaqSensor.bme680Status);
      Serial.println(output);
    }
  }
}

void errLeds(void)
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
}
