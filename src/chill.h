#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

//#define LCD_I2C_ADDRESS 0x27
#define LCD_I2C_ADDRESS 0x3F

LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, 16, 2);

// For DS18B20 Sensors
//#include <OneWire.h>
#include <DallasTemperature.h>

// For DHT-22 Sensor
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>


#include <SoftPWM.h>

#include <TimeLib.h>
#include <NtpClientLib.h>


#define S2 9
#define S3 10

// Pin mappings
#define LCD_SDA           0 // 0  // D3
#define LCD_SCL           2 // 2  // D4
#define PIN_BTN_RED       D6// D5  // D1
#define PIN_BTN_BLACK     D5// D6 // A0 ??
#define PIN_DHT22         D1// D8 // D7
#define PIN_TEMPS         D2// D7  // D2
#define PIN_K1            D0// S2 // D5 (purple)
#define PIN_LASER_PUMP    D7 //S3//S3 // D6 (blue)
#define PIN_RUNNING       D8

#define D9 3
#define D10 1



// Data wire is plugged into port 2 on the Arduino
#define TEMPERATURE_PRECISION 12
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(PIN_TEMPS);
// DS18B20 Sensors - Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

#define ADDR1 {0x28, 0xff, 0x36, 0x98, 0x74, 0x16, 0x03, 0x78}
#define ADDR2 {0x28, 0xff, 0x5d, 0x94, 0x74, 0x16, 0x03, 0x0b}
#define ADDR3 {0x28, 0xff, 0xbf, 0x95, 0x74, 0x16, 0x03, 0xdd}
#define ADDR4 {0x28, 0xff, 0xd5, 0x96, 0x74, 0x16, 0x03, 0xf9}
#define ADDR5 {0x28, 0xff, 0x80, 0x95, 0x74, 0x16, 0x03, 0x36}

DeviceAddress aTubeIn = ADDR2;
DeviceAddress aTubeReturn = ADDR3;
DeviceAddress aChillReturn = ADDR1; //don't fuck with this one now.
DeviceAddress aChillSump = ADDR4;
DeviceAddress aChillAir = ADDR5;

// DHT22 config
#define DHTTYPE           DHT22
DHT_Unified dht(PIN_DHT22, DHTTYPE);

#define WIFI_SSID "NETGEAR80"
#define WIFI_PASSWORD "dizzykayak157"

#define MQTT_SERVER "doober.space"
#define MQTT_PORT 31989
#define MQTT_USER "trogdor"
#define MQTT_PASSWORD "pewpewpew"


#define NTP_SERVER "130.102.128.23"

float tTubeIn = 0;
float tTubeReturn = 0;
float tChillReturn = 0;
float tChillSump = 0;
float tChillAir = 0;
float tAmbient = 0;
float hAmbient = 0;
float dpAmbient = 0;

float tTubeTarget = 15.0;

#define SCREEN_SENSORS 1
#define SCREEN_AMBIENT 2
#define SCREEN_STATE 3

#define CHILLER_OFF 0
#define CHILLER_ON 1

byte screen_num = SCREEN_SENSORS;
byte chiller_state = CHILLER_OFF;

unsigned long nextRead = millis();
unsigned long nextRedraw = millis();

#define DEBUG "trogdebug/addresses"

#define ANALOG_READ_FREQ 5000
#define REDRAW_INTERVAL 3000
#define MAX_SCREEN_NUM 3

#define PIN_CIRCULATOR PIN_K1

#define SWITCH_DEBOUNCE_MS 500

unsigned long circulator_period = 120000;
SoftPWM circulator(PIN_CIRCULATOR, circulator_period);



float integral = 0;
float kP = 50;
float kI = 1;
float error = 0;
float INTEGRAL_MAX = 150;
float DUTY_MAX = 255;

uint8_t chiller_duty = 0;

byte ntp_ready = 0;
unsigned long debounce = 0;
