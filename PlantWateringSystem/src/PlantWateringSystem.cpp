/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "/Users/pedrodanielsanchez/Documents/IoT/PlantWateringSystem/PlantWateringSystem/src/PlantWateringSystem.ino"
/*
 * Project PlantWateringSystem
 * Description: Smart Plant Watering System
 * Author: Pedro Sanchez
 * Date: 9-Nov-2021 
 */

#include <Adafruit_MQTT.h>

#include "Adafruit_MQTT/Adafruit_MQTT.h"
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"
#include "Adafruit_GFX_RK.h"
#include "Adafruit_SSD1306_RK.h"
#include "Adafruit_BME280.h"
#include "JsonParserGeneratorRK.h"

#include "credentials.h"


void setup();
void loop();
int readDisplayMoisture();
void bme280Setup();
void displaySetup();
void turnLED(bool onOff);
void MQTT_connect();
#line 20 "/Users/pedrodanielsanchez/Documents/IoT/PlantWateringSystem/PlantWateringSystem/src/PlantWateringSystem.ino"
#define OLED_RESET D4
Adafruit_SSD1306 display(OLED_RESET);
Adafruit_BME280 bme; //this is for I2C device

/************ Global State (you don't need to change this!) ***   ***************/
TCPClient TheClient;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_SPARK mqtt(&TheClient, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

/****************************** Feeds ***************************************/
// Setup Feeds to publish or subscribe
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
Adafruit_MQTT_Publish   mqttMoistureVal = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/moisturepair");
Adafruit_MQTT_Subscribe mqttButtonVal = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/mybutton01");

/************Declare Variables*************/

struct data
{
  int value1;
  int value2;
};
data myData; // data structure

void myPayload(data myData); 

unsigned long last, lastTime;
int value1 = 0;
int value2 = 0;
int myButton;
bool onOff = false;

const int MOIST_PIN = A1;
const int BME290_addr = 0x76;
const int OLED_addr = 0x3C;
char t_time[40];
int moist;
float tempF, humidRH;

SYSTEM_MODE(SEMI_AUTOMATIC);

void setup()
{
  pinMode(MOIST_PIN, INPUT); // Moisture pin
  pinMode(D7, OUTPUT);       // Argon built-in LED pin

  Serial.begin(9600);
  waitFor(Serial.isConnected, 15000); //wait for Serial Monitor to startup

  bme280Setup();  // Initialize bme280
  displaySetup(); // Initialize SSD1306 display

  Time.zone(-6);         // CST = -6   CDT = -5
  Particle.syncTime();   // Get time fron Particle cloud
  

  //Connect to WiFi without going to Particle Cloud
  WiFi.connect();
  while (WiFi.connecting())
  {
    Serial.printf(".");
  }

  // Setup MQTT subscription for onoff feed.
  mqtt.subscribe(&mqttButtonVal);
}

void loop()
{
  // Validate connected to MQTT Broker
  MQTT_connect();

  // Ping MQTT Broker every 2 minutes to keep connection alive
  if ((millis() - last) > 120000)
  {
    Serial.printf("Pinging MQTT \n");
    if (!mqtt.ping())
    {
      Serial.printf("Disconnecting \n");
      mqtt.disconnect();
    }
    last = millis();
  }

  // Publish to cloud every 6 seconds

  if ((millis() - lastTime > 6000)) {
    if (mqtt.Update()) {
      moist = readDisplayMoisture();
      // sample data
      myData.value1 = moist;
      myData.value2 = -1 * moist; 

      myPayload(myData);
      tempF = (bme.readTemperature() * (9/5.0)) + 32; //deg F
      humidRH = bme.readHumidity();  // %RH
    }
    lastTime = millis();
  }

  // 'Wait for incoming subscription packets' busy subloop
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(1000)))
  {
    if (subscription == &mqttButtonVal)
    {
      value2 = atoi((char *)mqttButtonVal.lastread);
      Serial.printf("Received %i from Adafruit.io feed ButtonOnOff \n", value2);
      if (value2 == 0)
      {
        onOff = !onOff;
        turnLED(onOff); // Turn on or Off =>  D7 led
      }
    }
  }
}

int readDisplayMoisture() {
  int moist;

  moist = analogRead(MOIST_PIN);  // Analog READ

  Serial.printf("Moist Reading = %i   Temp: %0.2f    Humidity: %0.2f \n", moist, tempF, humidRH);

  display.setCursor(3, 0);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.printf("Moisture: %i\n", moist);
  display.printf("Temp: %0.2f F\n", tempF);
  display.printf("Humidity: %0.2f %\n", humidRH);
  display.display();
  return moist;
}

void bme280Setup() {
  bool status = bme.begin(BME290_addr);
  if (status == false) {
  Serial.printf("BME280 at address 0x%02X failed to start", BME290_addr); }
}

void displaySetup() {
  display.begin(SSD1306_SWITCHCAPVCC, OLED_addr);
  display.display();
  delay(2000);
  display.clearDisplay();

  display.drawPixel(10, 10, WHITE);
  display.display();
  delay(2000);
  display.clearDisplay();
  Serial.printf("Started Moisture Reading\n");
}

void myPayload(data myData)
{
  JsonWriterStatic<256> jw;
  {
    JsonWriterAutoObject obj(&jw);
    jw.insertKeyValue("value1", myData.value1);
    jw.insertKeyValue("value2", myData.value2);
  }

  Serial.printf("Publishing value1 = %i, value2 = %i \n",
                                myData.value1, myData.value2);
  mqttMoistureVal.publish(jw.getBuffer());
  //Particle.publish("env-vals", jw.getBuffer(), PRIVATE);
}

void turnLED(bool onOff)
{
  if (onOff)
  {
    digitalWrite(D7, HIGH);
  }
  else
  {
    digitalWrite(D7, LOW);
  }
}

// Function to connect and reconnect as necessary to the MQTT server.
void MQTT_connect()
{
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected())
  {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  while ((ret = mqtt.connect()) != 0)
  { // connect will return 0 for connected
    Serial.printf("%s\n", (char *)mqtt.connectErrorString(ret));
    Serial.printf("Retrying MQTT connection in 5 seconds..\n");
    mqtt.disconnect();
    delay(5000); // wait 5 seconds
  }
  Serial.printf("MQTT Connected!\n");
}