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
#include "Grove_Air_quality_Sensor.h"

#include "IoT_Timer.h"
#include "credentials.h"


#define OLED_RESET D4
Adafruit_SSD1306 display(OLED_RESET);  // I2C oled display
Adafruit_BME280 bme;                   // I2C temp_pressure_humidity

TCPClient TheClient;

// Setup the MQTT client class by passing in 
// the WiFi client and MQTT server and login details.
Adafruit_MQTT_SPARK mqtt(&TheClient, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// Setup Feeds to publish or subscribe
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>

// Publis feeds
Adafruit_MQTT_Publish TempWPS = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperaturePWS");
Adafruit_MQTT_Publish HumidityWPS = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humidityPWS");
Adafruit_MQTT_Publish PressureWPS = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/pressurePWS");
Adafruit_MQTT_Publish AirQualityWPS = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/airqualityPWS");
Adafruit_MQTT_Publish MoistureWPS = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/moisturePWS");
Adafruit_MQTT_Publish DustWPS = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/dustPWS");
// Subscribe feeds
Adafruit_MQTT_Subscribe ButtonWPS = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/buttonWPS");

struct data
{
  int value1;
  int value2;
};
data myData; // data structure

//void myPayload(data myData); 

String DateTime, TimeOnly;

int value1 = 0;
int value2 = 0;

int myButton;
bool onOff = false;

const int MOIST_PIN      = A1;
const int BME290_addr    = 0x76;
const int OLED_addr      = 0x3C;
const int PUMP_PIN       = 11;
const int DUST_SENS_PIN  = A3;
const int AQS_PIN        = A2;

AirQualitySensor aqSensor(AQS_PIN);

const int DUST_SENSOR_READ = 30000; // every 30 sec

int moist; // moisture var

float tempF, humidRH, pressurePas;  // temp and humidity vars

String airQuality;     // airquality string message var

// Dust vars
unsigned long lastInterval;
unsigned long lowpulseoccupancy = 0;
unsigned long lowpulseoccupancyLast = 0;

unsigned long last_lpo = 0;
unsigned long duration;
float ratio = 0;
float concentration = 0;

// Timer vars
IoT_Timer checkMoisture;
IoT_Timer pingMQQT;
IoT_Timer readDustSensor;
IoT_Timer writeToCloud;

SYSTEM_MODE(SEMI_AUTOMATIC);

void setup()
{
  pinMode(MOIST_PIN, INPUT);  // Moisture device mode
  pinMode(D7, OUTPUT);        // Argon built-in LED mode
  pinMode(PUMP_PIN, OUTPUT);  // PUMP mode
  pinMode(DUST_SENS_PIN, INPUT); // Dust Sensor Input mode

  Serial.begin(9600);
  waitFor(Serial.isConnected, 15000); //wait for Serial Monitor to startup

  bme280Setup();  // Initialize bme280
  displaySetup(); // Initialize SSD1306 display
  aqsSetup();     // Initialize Air Quality Sensor

  Time.zone(-6);         // CST = -6   CDT = -5 ( Austin, Texas )
  Particle.syncTime();   // Get time fron Particle cloud
  

  //Connect to WiFi without going to Particle Cloud
  WiFi.connect();
  while (WiFi.connecting())
  {
    Serial.printf(".");
  }

  Serial.printf("\n\nStarted Smart Plant Watering System V1.0\n");
  // Setup MQTT subscription for onoff feed.
  mqtt.subscribe(&ButtonWPS);

  pingMQQT.startTimer(100);       // first time check immediately
  checkMoisture.startTimer(6000); // first time check immediately
  readDustSensor.startTimer(DUST_SENSOR_READ);
  writeToCloud.startTimer(14000); // Every 14 seconds
}

void loop()
{
  DateTime = Time.timeStr(); 
  TimeOnly = DateTime.substring(11, 19);

  MQTT_connect(); 
  keepAliveMQQT();    
  autoWaterPlant();
  duration = pulseIn(DUST_SENS_PIN, LOW);
  lowpulseoccupancy = lowpulseoccupancy + duration;
  readDustLevel();

  if (writeToCloud.isTimerReady()) { // every 14 secs, values are published to the cloud
    tempF = (bme.readTemperature() * (9 / 5.0)) + 32; //deg F
    humidRH = bme.readHumidity();                     // %RH
    pressurePas = bme.readPressure();
    airQuality = getAirQuality();
    Serial.printlnf("Air Quality is : %s", airQuality.c_str());
    if (mqtt.Update()) {
      // moist is calculates every 6 seconds, use last value
      // dust is read every 30 seconds, use last value: lowpulseoccupancyLast
      // Publish data to feeds
      // Publis feeds
      TempWPS.publish(tempF);
      HumidityWPS.publish(humidRH);
      PressureWPS.publish(pressurePas);
      AirQualityWPS.publish(airQuality);
      MoistureWPS.publish(moist);
      DustWPS.publish(concentration);

      writeToCloud.startTimer(14000); // Write to cloud every 14 seconds;
    }
  }

  // 'Wait for incoming subscription packets' busy subloop
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(1000)))
  {
    if (subscription == &ButtonWPS)
    {
      value2 = atoi((char *)ButtonWPS.lastread);
      Serial.printf("Received %i from Adafruit.io feed ButtonOnOff \n", value2);
      if (value2 == 0)
      {
        onOff = !onOff;
        turnLED(onOff);  // Turn on or Off =>  D7 led
        waterThePlant(); // Autowater the plant
      }
    }
  }
}

void readDustLevel() {
  if (readDustSensor.isTimerReady()) {
    getDustSensorReadings();
    lowpulseoccupancyLast = lowpulseoccupancy;
    lowpulseoccupancy = 0;
    readDustSensor.startTimer(DUST_SENSOR_READ); 
  }
}

void getDustSensorReadings() {
  if (lowpulseoccupancy == 0) {
    lowpulseoccupancy = last_lpo;
  }
  else {
    last_lpo = lowpulseoccupancy;
  }
  ratio = lowpulseoccupancy / (DUST_SENSOR_READ * 10.0);
  concentration = 1.1 * pow(ratio, 3) - 3.8 * pow(ratio, 2) + 520 * ratio + 0.62;
  Serial.printf("LPO: %li\n", lowpulseoccupancy);
  Serial.printf("Ratio: %f%%\n", ratio);
  Serial.printf("Concentration: %f pcs/L\n", concentration);
}

    void autoWaterPlant()
{
  // Publish to cloud every 6 seconds

  // if ((millis() - lastTime > 6000)) {
  if (checkMoisture.isTimerReady()) {
    moist = readDisplayMoisture();
    if (moist > 2850) {
      waterThePlant();
    }
    // lastTime = millis();
    checkMoisture.startTimer(6000);
  }
}

void waterThePlant() {
  Serial.printf("Turn on the PUMP\n");
  digitalWrite(PUMP_PIN, HIGH);
  delay(500);
  digitalWrite(PUMP_PIN, LOW);
  delay(2000);
}

void keepAliveMQQT()
{
  // Ping MQTT Broker every 2 minutes to keep connection alive
  if (pingMQQT.isTimerReady()) {
    Serial.printf("Pinging MQTT \n");
    if (!mqtt.ping()) {
      Serial.printf("Disconnecting \n");
      mqtt.disconnect();
    }
    pingMQQT.startTimer(120000);
  }
}

String getAirQuality()
{
  int quality = aqSensor.slope();
  String qual = "None";

  if (quality == AirQualitySensor::FORCE_SIGNAL) {
    qual = "Danger";
  }
  else if (quality == AirQualitySensor::HIGH_POLLUTION) {
    qual = "High Pollution";
  }
  else if (quality == AirQualitySensor::LOW_POLLUTION){
    qual = "Low Pollution";
  }
  else if (quality == AirQualitySensor::FRESH_AIR) {
    qual = "Fresh Air";
  }
  return qual;
}

int readDisplayMoisture() {
  int moist;

  moist = analogRead(MOIST_PIN);  

  Serial.printf("Moist Reading = %i   Temp: %0.2f F  Humidity: %0.2f \n", moist, tempF, humidRH);

  display.setCursor(3, 0);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.printf("%s\n", DateTime.c_str());
  display.printf("Moisture: %i\n", moist);
  display.printf("Temp: %0.2f F\n", tempF);
  display.printf("Humidity: %0.2f \n", humidRH);
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

void aqsSetup() {
  if (aqSensor.init()) {
    Serial.println("Air Quality Sensor ready.\n");
  }
  else {
    Serial.println("Air Quality Sensor ERROR!\n");
  }
}

void turnLED(bool onOff)
{
  if (onOff) {
    digitalWrite(D7, HIGH);
  }
  else {
    digitalWrite(D7, LOW);
  }
}

// Function to connect and reconnect as necessary to the MQTT server.
void MQTT_connect()
{
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
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

// void myPayload(data myData)
// {
//   JsonWriterStatic<256> jw;
//   {
//     JsonWriterAutoObject obj(&jw);
//     jw.insertKeyValue("value1", myData.value1);
//     jw.insertKeyValue("value2", myData.value2);
//   }

//   Serial.printf("Publishing value1 = %i, value2 = %i \n",
//                 myData.value1, myData.value2);
//   mqttMoistureVal.publish(jw.getBuffer());
// }