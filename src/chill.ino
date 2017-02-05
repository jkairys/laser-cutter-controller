#include "chill.h"
#include <IOThing.h>

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
  client.publish(DEBUG, ("Found " + String(numberOfDevices) + " devices.").c_str());

  for(int i=0;i<numberOfDevices; i++){
    // Search the wire for address
    if(sensors.getAddress(tempDeviceAddress, i)){
      String tmp = "";
      fmtAddress(tempDeviceAddress, tmp);
      client.publish(DEBUG, ("Device " + String(i) + " = " + tmp).c_str());
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
      set_chiller(CHILLER_ON);
    }else{
      set_chiller(CHILLER_OFF);
    }
  }

  if(tmp == "target"){
    //Serial.println("Run: " + pl);
    tTubeTarget = pl.toFloat();
    integral = 0;
  }

}

void setup_mqtt(){
  client.setServer("doober.space", 31989);
  client.setCallback(mqtt_callback);

  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("trogtroller2", "trogdor", "pewpewpew")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      //client.publish("garden/greenhouse/status", "online");
      // ... and resubscribe
      client.subscribe("trogdor/settings/#");
    } else {
      lcd.clear();
      lcd.print("Jethro MQTT Fail");
      lcd.setCursor(0,1);
      lcd.print("trying again");
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

  //Serial.println("{\n\t\"tResSump\": " + String(tResSump,1) + "\n\t\"tResIn\": " + String(tResIn,1) + "\n\t\"tChillSump\": " + String(tChillSump,1) + "\n\t\"tChillIn\": " + String(tChillIn,1) +"\n\t\"tAmbient\": " + String(tAmbient,1)+"\n\t\"rhAmbient\": " + String(hAmbient,1) +"\n\t\"dpAmbient\": " + String(dpAmbient,1) + "\n}");
  mqtt_publish("tube/tIn",        String(tTubeIn,1));
  mqtt_publish("tube/tReturn",    String(tTubeReturn,1));
  mqtt_publish("chiller/tReturn", String(tChillReturn,1));
  mqtt_publish("chiller/tSump",   String(tChillSump,1));
  mqtt_publish("chiller/tAir",    String(tChillAir,1));
  mqtt_publish("ambient/t",       String(tAmbient,1));
  mqtt_publish("ambient/rh",      String(hAmbient,1));
  mqtt_publish("ambient/dp",      String(dpAmbient,1));
  mqtt_publish("chiller/run",     String(chiller_state));
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
}

void setup() {

  iot.useMQTT(MQTT_SERVER, MQTT_PORT, MQTT_USER, MQTT_PASSWORD,[](String topic, String payload){
    read_settings(topic, payload);
  });

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
  lcd.begin(16,2);
  lcd.backlight();
  lcd.print("WiFi Connecting");

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    lcd.clear();
    lcd.print("Cams Shitty Wifi");
    lcd.setCursor(0,1);
    lcd.print("...rebooting");
    Serial.println("Connection Failed! Rebooting...");
    delay(1000);
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
  setup_mqtt();

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname("trogbrain2");

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

  Serial.println("Searching for connected DS18B20 sensors");
  dump_sensors();
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
  ArduinoOTA.handle();

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

  if (!client.connected()) {
    setup_mqtt();
  }
  client.loop();
}
