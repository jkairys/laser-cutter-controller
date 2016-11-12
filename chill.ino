#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27,16,2); 


// For DS18B20 Sensors
#include <OneWire.h>
#include <DallasTemperature.h>

// For DHT-22 Sensor
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>


// Pin mappings
#define LCD_SDA           0  // D3
#define LCD_SCL           2  // D4
#define PIN_BTN_RED       3  // RX
#define PIN_BTN_BLACK     A0 // A0 ??
#define PIN_DHT22         13 // D7
#define PIN_TEMPS         4  // D2
#define PIN_K1            14 // D5 (purple)
#define PIN_K2            12 // D6 (blue)

// Data wire is plugged into port 2 on the Arduino
#define TEMPERATURE_PRECISION 12
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(PIN_TEMPS);
// DS18B20 Sensors - Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
DeviceAddress aTrog1  = { 0x28, 0xFF, 0xD5, 0x96, 0x74, 0x16, 0x03, 0xF9 };
DeviceAddress aChill1 = { 0x28, 0xFF, 0x80, 0x95, 0x74, 0x16, 0x03, 0x36 };


// DHT22 config
#define DHTTYPE           DHT22
DHT_Unified dht(PIN_DHT22, DHTTYPE);

const char* ssid = "mushroom";
const char* password = "gumboots";

float tAmbient = 0.0;
float hAmbient = 0.0;

float tChill1 = 0.0;
float tTrog1 = 0.0;

void readAmbient(){
  sensors_event_t event;  
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println("Error reading ambient temperature!");
  }else {
    tAmbient = event.temperature;
  }
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println("Error reading ambient humidity!");
  }else {
    hAmbient = event.relative_humidity;
  }
  //Serial.println("Ambient: " + String(tAmbient,1) + "C @ " + String(hAmbient,1) + "%");
}

void readTemperatures(){
  sensors.requestTemperatures(); 
  tTrog1 = sensors.getTempC(aTrog1);
  tChill1 = sensors.getTempC(aChill1);
  readAmbient();
}

void logSensors(){
  
  Serial.println("{\n" 
    + "\n\t\"tTrog1\": " + String(tTrog1,1) 
    + "\n\t\"tChill1\": " + String(tChill1,1) 
    + "\n\t\"tAmbient\": " + String(tAmbient,1) 
    + "\n\t\"hAmbient\": " + String(hAmbient,1) 
    +"\n}");
}

void displayTemperatures(){
  lcd.clear();
  lcd.print(String(tTrog1,1));

  lcd.setCursor(0,1);
  lcd.print(String(tChill1,1));
  
}

void setup() {

  pinMode(PIN_BTN_RED,   INPUT);
  pinMode(PIN_BTN_BLACK, INPUT);
  pinMode(PIN_K1,        OUTPUT);
  pinMode(PIN_K2,        OUTPUT);

  Serial.begin(115200);
  Serial.println("Booting");
  
  sensors.begin();
  sensors.setResolution(aChill1, TEMPERATURE_PRECISION);
  sensors.setResolution(aTrog1, TEMPERATURE_PRECISION);

  // initialise & validate dht22 sensor is working ok
  dht.begin();
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println("Temperature");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  dht.humidity().getSensor(&sensor);
  Serial.println("Humidity");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  
  Wire.begin(LCD_SDA, LCD_SCL);
  lcd.init();                     
  lcd.backlight();
  lcd.print("WiFi Connecting");
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  
  lcd.clear();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  
  lcd.print("Connected");
  lcd.setCursor(0,1);
  lcd.print(WiFi.localIP());
  delay(2000);
  
  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname("trogtroller");

  // No authentication by default
  //ArduinoOTA.setPassword((const char *)"123");

  ArduinoOTA.onStart([]() {
    lcd.clear();
    lcd.print("Updating F/W");
  });
  
  ArduinoOTA.onEnd([]() {
    lcd.clear();
    lcd.print("Update Complete");
  });
  
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    lcd.setCursor(0,1);
    String tmp;
    float pct = progress;
    pct = 100 * pct / total;
    tmp = String(pct, 0 ) + "%   ";
    char p[16];
    tmp.toCharArray(p, 16);
    lcd.print(p);
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
  
}

unsigned long nextRead = millis();

void loop() {
  ArduinoOTA.handle();
  if(millis() > nextRead){
    readTemperatures();
    displayTemperatures();
    logSensors();
    nextRead = millis() + 5000;
  }
}
