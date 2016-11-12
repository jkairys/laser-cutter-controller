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


#include <SoftPWM.h>

#include <TimeLib.h>
#include <NtpClientLib.h>

// MQTT
#include <PubSubClient.h>
WiFiClient espClient;
PubSubClient client(espClient);



// Pin mappings
#define LCD_SDA           0  // D3
#define LCD_SCL           2  // D4
#define PIN_BTN_RED       13  // D1
#define PIN_BTN_BLACK     5 // A0 ??
#define PIN_DHT22         12 // D7
#define PIN_TEMPS         4  // D2
#define PIN_K1            14 // D5 (purple)
#define PIN_K2            A0 // D6 (blue)

// Data wire is plugged into port 2 on the Arduino
#define TEMPERATURE_PRECISION 12
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(PIN_TEMPS);
// DS18B20 Sensors - Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
DeviceAddress aResSump  = { 0x28, 0xFF, 0xD5, 0x96, 0x74, 0x16, 0x03, 0xF9 };
DeviceAddress aResIn      = { 0x28, 0xFF, 0x80, 0x95, 0x74, 0x16, 0x03, 0x36 };
DeviceAddress aChillSump = { 0x28, 0xFF, 0x80, 0x95, 0x74, 0x16, 0x03, 0x36 };
DeviceAddress aChillIn = { 0x28, 0xFF, 0xD5, 0x96, 0x74, 0x16, 0x03, 0xF9 };

// DHT22 config
#define DHTTYPE           DHT22
DHT_Unified dht(PIN_DHT22, DHTTYPE);

const char* ssid = "NETGEAR80";
const char* password = "dizzykayak157";

float tResSump = 0;
float tResIn = 0;
float tChillSump = 0;
float tChillIn = 0;
float tAmbient = 0;
float hAmbient = 0;
float dpAmbient = 0;

float tSumpTarget = 15.0;

#define SCREEN_SENSORS 1
#define SCREEN_AMBIENT 2
#define SCREEN_STATE 3

#define CHILLER_OFF 0
#define CHILLER_ON 1

byte screen_num = SCREEN_SENSORS;
byte chiller_state = CHILLER_OFF;

unsigned long nextRead = millis();
unsigned long nextRedraw = millis();

#define ANALOG_READ_FREQ 5000
#define REDRAW_INTERVAL 3000
#define MAX_SCREEN_NUM 3

#define PIN_CIRCULATOR PIN_K1

SoftPWM circulator(PIN_CIRCULATOR, 10000000);



float integral = 0;
float kP = 25;
float kI = 1;
float error = 0;
float INTEGRAL_MAX = 150;
float DUTY_MAX = 255;

uint8_t chiller_duty = 0;

byte ntp_ready = 0;

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  String tmp;
  tmp = String(topic);

  Serial.println(topic);
  
  
  tmp.replace("trogdor/settings/","");

  
  
  char buf[16] = "";
  int i = 0;
  for (i = 0; i < length; i++) {
    buf[i] = (char)payload[i];
  }
  buf[i] = '\0';
  String pl = String(buf);
  Serial.println(pl);
  
  if(tmp == "run"){
    //Serial.println("Run: " + pl);
    //run_duration = pl.toInt();
    if(pl.toInt() > 0){
      chiller_state = CHILLER_ON;
      lcd.backlight();
    }else{
      chiller_state = CHILLER_OFF;
      lcd.noBacklight();
    }
  }
  
  if(tmp == "target"){
    //Serial.println("Run: " + pl);
    tSumpTarget = pl.toFloat();
    integral = 0;
    
    //start_watering();
  }

  /*
  if(tmp == "duration"){
    //Serial.println("Run: " + pl);
    watering_duration = pl.toInt();
    //start_watering();
  }

  if(tmp == "hr_start"){
    hour_start_watering = pl.toInt();
  }

  if(tmp == "hr_stop"){
    hour_stop_watering = pl.toInt();
  }
  */
}

void setup_mqtt(){
  client.setServer("doober.space", 1883);
  client.setCallback(mqtt_callback);
  
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("trogdor")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      //client.publish("garden/greenhouse/status", "online");
      // ... and resubscribe
      client.subscribe("trogdor/settings/#");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


void setup_ntp(){
  NTP.onNTPSyncEvent([](NTPSyncEvent_t error) {
    if (error) {
      Serial.print("Time Sync error: ");
      if (error == noResponse)
        Serial.println("NTP server not reachable");
      else if (error == invalidAddress)
        Serial.println("Invalid NTP server address");
    }
    else {
      Serial.print("Got NTP time: ");
      Serial.println(NTP.getTimeDateString(NTP.getLastNTPSync()));
      ntp_ready = true;
    }
    
  });
  NTP.begin("130.102.128.23", 1, false);
  NTP.setInterval(60*60);
  NTP.setTimeZone(10);
  
}

unsigned long last_read_ambient = 0;

byte readAmbient(){
  if(last_read_ambient > 0 && last_read_ambient < millis() + 5000){
    return NULL;
  }
  last_read_ambient = millis();
  sensors_event_t event;  
  dht.temperature().getEvent(&event);
  yield();
  if (isnan(event.temperature)) {
    Serial.println("Error reading ambient temperature!");
  }else {
    tAmbient = event.temperature;
  }
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  yield();
  if (isnan(event.relative_humidity)) {
    Serial.println("Error reading ambient humidity!");
  }else {
    hAmbient = event.relative_humidity;
  }

  dpAmbient = tAmbient - ((100 - hAmbient)/5);
  
  //Serial.println("Ambient: " + String(tAmbient,1) + "C @ " + String(hAmbient,1) + "%");
}

/*
 * 
 * 
 * float tResSump = 0;
float tResIn = 0;
float tChillSump = 0;
float tChillIn = 0;
 */

void readSensors(){
  sensors.requestTemperatures(); 
  yield();
  tResSump = sensors.getTempC(aResSump);
  yield();
  tResIn   = sensors.getTempC(aResIn);
  yield();
  tChillSump = sensors.getTempC(aChillSump);
  yield();
  tChillIn   = sensors.getTempC(aChillIn);
  yield();
  readAmbient();
  nextRead = millis() + ANALOG_READ_FREQ;
}


void mqtt_publish(char * path, String payload){
  char tmpPath[64];
  char tmpPay[64];
  String tmpS;
  tmpS = "trogdor/" + String(path);
  tmpS.toCharArray(tmpPath, 64);
  payload.toCharArray(tmpPay, 64);
  client.publish(tmpPath, tmpPay); 
}

void logSensors(){

  Serial.println("{\n\t\"tResSump\": " + String(tResSump,1) + "\n\t\"tResIn\": " + String(tResIn,1) + "\n\t\"tChillSump\": " + String(tChillSump,1) + "\n\t\"tChillIn\": " + String(tChillIn,1) +"\n\t\"tAmbient\": " + String(tAmbient,1)+"\n\t\"rhAmbient\": " + String(hAmbient,1) +"\n\t\"dpAmbient\": " + String(dpAmbient,1) + "\n}");
  mqtt_publish("reservoir/tOut",      String(tResSump,1));
  mqtt_publish("reservoir/tIn",       String(tResIn,1));
  mqtt_publish("chiller/tSump",       String(tChillSump,1));
  mqtt_publish("chiller/tIn",         String(tChillIn,1));
  mqtt_publish("ambient/t",           String(tAmbient,1));
  mqtt_publish("ambient/rh",          String(hAmbient,1));
  mqtt_publish("ambient/dp",          String(dpAmbient,1));
}

void displaySensors(){
  lcd.clear();
  lcd.print(String(tResSump,1) + " " + String(tResIn,1));
  lcd.setCursor(0,1);
  lcd.print(String(tChillSump,1) + " " + String(tChillIn,1));
}

void displayAmbient(){
  lcd.clear();
  lcd.print("Ambient");
  lcd.setCursor(0,1);
  lcd.print(String(tAmbient,1) + "C @ " + String(hAmbient) + "%"); 
}

void displayState(){
  lcd.clear();
  lcd.print("Chiller " + String(chiller_state == CHILLER_ON ? "ON" : "OFF"));
  lcd.setCursor(0,1);
  lcd.print("Duty " + String(chiller_duty,DEC));
}

#define SWITCH_DEBOUNCE_MS 500
unsigned long debounce = 0;

void btnRed(){
  if(millis() < debounce){
    return;
  }
  
  debounce = millis() + SWITCH_DEBOUNCE_MS;
  if(chiller_state == CHILLER_ON){
    chiller_state = CHILLER_OFF;
    lcd.noBacklight();
  }else{
    chiller_state = CHILLER_ON;
    lcd.backlight();
  }
  displayState();
  //delay(500);
}

void redraw(){
  switch(screen_num){
    case SCREEN_SENSORS:
      displaySensors();
      break;
    case SCREEN_AMBIENT:
      displayAmbient();
      break;
    case SCREEN_STATE:
      displayState();
      break;
  }
  nextRedraw = millis() + REDRAW_INTERVAL;
}

void nextScreen(){
  screen_num = screen_num + 1;
  if(screen_num > MAX_SCREEN_NUM){
    screen_num = 1;
  }
  redraw();
}

void btnBlack(){
  if(millis() < debounce){
    return;
  }
  debounce = millis() + SWITCH_DEBOUNCE_MS;
  nextScreen();
}



void setup() {

  pinMode(PIN_BTN_RED,   INPUT);
  pinMode(PIN_BTN_BLACK, INPUT);
  pinMode(PIN_K1,        OUTPUT);
  pinMode(PIN_K2,        OUTPUT);

  digitalWrite(PIN_K1, HIGH);
  digitalWrite(PIN_K2, HIGH);
  
  Serial.begin(115200);
  Serial.println("Booting");
  
  sensors.begin();
  sensors.setResolution(aResSump, TEMPERATURE_PRECISION);
  sensors.setResolution(aResIn, TEMPERATURE_PRECISION);
  sensors.setResolution(aChillSump, TEMPERATURE_PRECISION);
  sensors.setResolution(aChillIn, TEMPERATURE_PRECISION);

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
  lcd.begin();                     
  lcd.backlight();
  lcd.print("WiFi Connecting");
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  lcd.noBacklight();



  
  
  lcd.clear();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  
  lcd.print("Connected");
  lcd.setCursor(0,1);
  lcd.print(WiFi.localIP());
  delay(2000);
  setup_mqtt();
  
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


  // interrupts for switches
  attachInterrupt(PIN_BTN_BLACK, btnBlack, RISING);
  attachInterrupt(PIN_BTN_RED,   btnRed,   FALLING);

  circulator.setInverted(1);
  
  setup_ntp();
}

float tmp = 0.0;
void runChiller(){
  // DO PI CONTROLLER SHIT HERE
  error = tResSump - tSumpTarget;
  integral = integral + error * kI;
  
  if(integral > INTEGRAL_MAX){
    integral = INTEGRAL_MAX;  
  }

  if(integral < 0) integral = 0;

  tmp = kP * (error > 0 ? error : 0) + integral;
  
  if(tmp > DUTY_MAX){
    tmp = DUTY_MAX;
  }
  //Serial.println(tmp);
  chiller_duty = (uint8_t)tmp;
  circulator.analogWrite(chiller_duty);
  //Serial.println(chiller_duty);
}

void loop() {
  ArduinoOTA.handle();
  circulator.run();
  if(chiller_state == CHILLER_ON){
    runChiller();
  }else{
    circulator.analogWrite(0);
  }
  
  if(millis() > nextRedraw){
    redraw();  
  }
  
  if(millis() > nextRead){  
    readSensors();
    //displayTemperatures();
    logSensors();
    
  }

  if (!client.connected()) {
    setup_mqtt();
  }
  client.loop();
}
