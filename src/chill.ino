#include "chill.h"
#include <IOThing.h>
#include <EEPROM.h>

IOThing iot("trogdor3");


void fmtAddress(DeviceAddress deviceAddress, String &str){
  for (uint8_t i = 0; i < 8; i++){
    if (deviceAddress[i] < 16) str = str + "0";
    str = str + String(deviceAddress[i], HEX);
  }
}

void dump_sensors(){
  // Grab a count of devices on the wire
  int numberOfDevices = 0;
  numberOfDevices = sensors.getDeviceCount();
  DeviceAddress tempDeviceAddress;
  //client.publish(DEBUG, ("Found " + String(numberOfDevices) + " devices.").c_str());

  for(int i=0;i<numberOfDevices; i++){
    // Search the wire for address
    if(sensors.getAddress(tempDeviceAddress, i)){
      String tmp = "";
      fmtAddress(tempDeviceAddress, tmp);
      //client.publish(DEBUG, ("Device " + String(i) + " = " + tmp).c_str());
    }else{
      Serial.print("Found ghost device at ");
      Serial.print(i, DEC);
      Serial.print(" but could not detect address. Check power and cabling");
    }
   }
}

void displaySensors(){
  lcd.clear();
  lcd.print("Trg " + String(tTubeIn,1) + " " + String(tTubeReturn,1));
  lcd.setCursor(0,1);
  lcd.print("Slm " + String(tChillSump,1) + " " + String(tChillReturn,1));
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
  lcd.print("Tgt " + String(tTubeTarget,1) + " DC " + String(chiller_duty,DEC));
}


void set_chiller(byte state){
  chiller_state = state;
  // turn on trogdor circulator (important!!!)
  digitalWrite(PIN_LASER_PUMP, chiller_state > 0 ? false : true);
  // Set the status LED
  digitalWrite(PIN_RUNNING, chiller_state);
  circulator.cycle();
  displayState();
}


unsigned long last_read_ambient = 0;

byte readAmbient(){
  if(last_read_ambient > 0 && millis() < last_read_ambient + 10000){
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

}

void readSensors(){
  float tmp;

  sensors.requestTemperatures();
  yield();
  tmp = sensors.getTempC(aTubeIn);
  if(tmp > -40 && tmp < 60) tTubeIn = tmp;
  yield();
  tmp = sensors.getTempC(aTubeReturn);
  if(tmp > -40 && tmp < 60) tTubeReturn = tmp;
  yield();
  tmp = sensors.getTempC(aChillReturn);
  if(tmp > -40 && tmp < 60) tChillReturn = tmp;
  yield();

  tmp = sensors.getTempC(aChillSump);
  if(tmp > -40 && tmp < 60) tChillSump = tmp;
  yield();

  tmp = sensors.getTempC(aChillAir);
  if(tmp > -40 && tmp < 60) tChillAir = tmp;
  yield();

  readAmbient();
  nextRead = millis() + ANALOG_READ_FREQ;
}

/*
void mqtt_publish(char * path, String payload){
  char tmpPath[64];
  char tmpPay[64];
  String tmpS;
  tmpS = "trogdor/" + String(path);
  tmpS.toCharArray(tmpPath, 64);
  payload.toCharArray(tmpPay, 64);
  client.publish(tmpPath, tmpPay);
}
*/

void logSensors(){
  //Serial.println("Logging sensors to MQTT");

  //Serial.println("{\n\t\"tResSump\": " + String(tResSump,1) + "\n\t\"tResIn\": " + String(tResIn,1) + "\n\t\"tChillSump\": " + String(tChillSump,1) + "\n\t\"tChillIn\": " + String(tChillIn,1) +"\n\t\"tAmbient\": " + String(tAmbient,1)+"\n\t\"rhAmbient\": " + String(hAmbient,1) +"\n\t\"dpAmbient\": " + String(dpAmbient,1) + "\n}");
  iot.publish("tube/tIn",        String(tTubeIn,1));
  iot.publish("tube/tReturn",    String(tTubeReturn,1));
  iot.publish("chiller/tReturn", String(tChillReturn,1));
  iot.publish("chiller/tSump",   String(tChillSump,1));
  iot.publish("chiller/tAir",    String(tChillAir,1));
  iot.publish("ambient/t",       String(tAmbient,1));
  iot.publish("ambient/rh",      String(hAmbient,1));
  iot.publish("ambient/dp",      String(dpAmbient,1));
  iot.publish("chiller/run",     String(chiller_state));
}

void btnRed(){
  if(millis() < debounce){
    return;
  }
  debounce = millis() + SWITCH_DEBOUNCE_MS;
  if(chiller_state == CHILLER_ON){
    set_chiller(CHILLER_OFF);
  }else{
    set_chiller(CHILLER_ON);
  }
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

void read_settings(String topic, String payload){
  if(topic == "run"){
    if(payload.toInt() > 0){
      set_chiller(CHILLER_ON);
    }else{
      set_chiller(CHILLER_OFF);
    }
  }

  if(topic == "target"){
    //Serial.println("Run: " + pl);
    tTubeTarget = payload.toFloat();
    integral = 0;
  }
  write_settings_to_eeprom();
}

void setup() {

  pinMode(PIN_RUNNING, OUTPUT);
  pinMode(PIN_BTN_RED,   INPUT);
  pinMode(PIN_BTN_BLACK, INPUT);
  pinMode(PIN_K1,        OUTPUT);
  pinMode(PIN_LASER_PUMP,        OUTPUT);

  digitalWrite(PIN_K1, HIGH);
  digitalWrite(PIN_LASER_PUMP, HIGH);


  Serial.begin(115200);
  Serial.println("Booting");

  sensors.begin();
  sensors.setResolution(aTubeIn, TEMPERATURE_PRECISION);
  sensors.setResolution(aTubeReturn, TEMPERATURE_PRECISION);
  sensors.setResolution(aChillReturn, TEMPERATURE_PRECISION);

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

  iot.useWiFi(WIFI_SSID, WIFI_PASSWORD);
  iot.useOTA();
  // Configure Network Time Protocol server
  iot.useNTP(NTP_SERVER);
  // Configure MQTT Server
  iot.useMQTT(MQTT_SERVER, MQTT_PORT, MQTT_USER, MQTT_PASSWORD,[](String topic, String payload){
    read_settings(topic, payload);
  });

  lcd.clear();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  lcd.print("Connected");
  lcd.setCursor(0,1);
  lcd.print(WiFi.localIP());

  // interrupts for switches
  attachInterrupt(PIN_BTN_BLACK, btnBlack, RISING);
  attachInterrupt(PIN_BTN_RED,   btnRed,   FALLING);

  circulator.setInverted(1);

  //Serial.println("Searching for connected DS18B20 sensors");
  //dump_sensors();


  EEPROM.begin(512);
  read_settings_from_eeprom();
}

void write_settings_to_eeprom(){
  // setpoint
  EEPROM.put(0, tTubeTarget);
  EEPROM.commit();
}

void read_settings_from_eeprom(){
  // setpoint
  EEPROM.get(0, tTubeTarget);
  if(tTubeTarget < 10 || tTubeTarget > 25) tTubeTarget = 18;
}

float tmp = 0.0;
void runChiller(){
  // DO PI CONTROLLER SHIT HERE
  error = tTubeIn - tTubeTarget;
  integral = integral + error * kI;

  if(integral > INTEGRAL_MAX){
    integral = INTEGRAL_MAX;
  }

  if(integral < 0) integral = 0;

  tmp = kP * (error > 0 ? error : 0) + integral;

  if(tmp > DUTY_MAX){
    tmp = DUTY_MAX;
  }
  chiller_duty = (uint8_t)tmp;
  circulator.analogWrite(chiller_duty);
}

void loop() {

  if(chiller_state == CHILLER_ON){
    runChiller();
  }else{
    circulator.analogWrite(0);
  }
  circulator.run();
  if(millis() > nextRedraw){
    redraw();
  }

  if(millis() > nextRead){
    readSensors();
    logSensors();
  }

  iot.loop();
}
