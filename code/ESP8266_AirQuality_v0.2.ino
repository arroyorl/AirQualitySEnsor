/*
 *  This sketch demonstrates how to set up a simple IoT 
 *  Air Quality MQTT device.
 *  During the setup the device will act as AP mode so that you can set the
 *  Wifi connection, then it will try to connect to the WiFi.
 *  - While serving the webserver and showing the data, it will also post
 *    the data to MQTT broker.
 */
///////////////////////////////////////////////////////////////
//           History                                         //
///////////////////////////////////////////////////////////////
//  0.1 first version using sds011
//      code based on measure.ino from Dirk O. Kaar <dok@dok-net.net>
//
//  0.1a updated sendDataWunderground()
//      eco mode working
//      added ADC_MODE(ADC_VCC) for read internal VCC
//      reviewed code for getBattLevel() (VCCINT)
//
//  0.2 added DHT22 to read Temp & Humid to be used in 
//      humidityCompensation funcion (based on hackAir github)
//
///////////////////////////////////////////////////////////////


#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPClient.h>

#include <PubSubClient.h>

#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include <DHT.h>

#define FVERSION  "v0.2"

/////////////////SENSOR TYPE///////////////////////
// Uncomment whatever type you're using!         //
#define SDS011      
///////////////////////////////////////////////////
// Comment for final release
#define RDEBUG
///////////////////////////////////////////////////

#include <SoftwareSerial.h>
#include <Sds011.h>

WiFiClient espClient;
PubSubClient client(espClient);

#define   NUM_SAMPLES 3
#define   TABLESIZE 20

#include "Rdebug.h"
#include "settings.h"
#include "mainPage.h"
#include "initPage.h"
#include "fingerPage.h"

#define GPIO0 0
#define GPIO1 1
#define GPIO2 2
#define GPIO3 3
#define GPIO4 4
#define GPIO5 5
#define GPIO6 6
#define GPIO7 7
#define GPIO8 8
#define GPIO9 9
#define GPIO10 10
#define GPIO11 11
#define GPIO12 12
#define GPIO13 13
#define GPIO14 14
#define GPIO15 15
#define GPIO16 16

#define LEDON LOW
#define LEDOFF HIGH
#define SETUP_PIN GPIO0
#define WAKEUP_PIN GPIO16

//#define VCCEXT

// if VCC will be readed internally, this is needed
#ifndef VCCEXT
ADC_MODE(ADC_VCC);
#endif

#define SDS_PIN_RX GPIO5  // D1
#define SDS_PIN_TX GPIO4  // D2
#define DHT_PIN    GPIO13 // D7

// Define Red and green LEDs
#ifdef ARDUINO_ESP8266_NODEMCU
  #define RED_LED GPIO12 // D6
  #define GREEN_LED GPIO10 // D12 - SD3
#endif
#ifdef ARDUINO_ESP8266_GENERIC
  #define RED_LED GPIO12 //    antes GPIO9
  #define GREEN_LED GPIO10
#endif

ESP8266WebServer setupserver(80);
ESP8266WebServer server(80);

#ifdef ESP32
  HardwareSerial& serialSDS(Serial2);
  Sds011Async< HardwareSerial > sds011(serialSDS);
#else
  SoftwareSerial serialSDS;
  Sds011Async< SoftwareSerial > sds011(serialSDS);
#endif

//Struct to store setting in EEPROM
Settings    settings;

//Global Variables
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
int         inecomode = false; // is in eco mode?
String      data;
String      httpUpdateResponse;
const char* ap_ssid = "ESP-TEMP";
const char* ap_password = ""; //"12345678";
int         ap_setup_done = 0;
long        start;
long        timer;
long        timer2;
int         pm25_table[TABLESIZE];
int         pm10_table[TABLESIZE];
float       pm25f;
float       pm10f;
bool        is_SDS_running = true;
float       temperature=0;
float       humidity=0;
float       vcc=0;
int         batt=0;
String      base_topic;
rst_info    *myResetInfo;

// DHT22 definition
DHT dht(DHT_PIN, DHT22);

void displayBusy(){
  digitalWrite(RED_LED, LEDON);
  delay(500);
  digitalWrite(RED_LED, LEDOFF);
  delay(500);
}

/**************************************************************************/
/*  Takes an average of readings on a given pin
/*  Returns the average
/**************************************************************************/
int averageAnalogRead(int pinToRead)
{
    byte numberOfReadings = 3;
    unsigned int runningValue = 0;

    for(int x = 0 ; x < numberOfReadings ; x++)
        runningValue += analogRead(pinToRead);
    runningValue /= numberOfReadings;

    return(runningValue);
}

/**************************************************************************/
/*     Read the internal or external Vcc
/**************************************************************************/
int GetBattLevel ()
{
  int battlevel;
  int vccint;
#ifdef VCCEXT
  vcc = averageAnalogRead(A0);
  DebugLn("VccExt: "+String(vcc) +", "+String(vcc/1024*4.2));
  battlevel=map(vcc,780,1024,0,100);
#else
    vccint  = ESP.getVcc();
    DebugLn("VccInt: " + String(vccint/1000.0));
    battlevel=map(vccint,2200,3300,0,100);
#endif
  return battlevel;
}


/**************************************************************************/
/*    humidityCompensation function
/*    based on same name function in hackAir.cpp 
/*    (https://github.com/hackair-project/hackAir-Arduino)
/**************************************************************************/
void  humidityCompensation() 
{
  pm25f = pm25f / (1.0 + 0.48756 * pow((humidity / 100.0), 8.60068));
  pm10f = pm10f / (1.0 + 0.81559 * pow((humidity / 100.0), 5.83411));
}

/**************************************************************************/
/*     sds011 setup function 
/**************************************************************************/
void SDS011setup()
{

#ifdef ESP32
    serialSDS.begin(9600, SERIAL_8N1, SDS_PIN_RX, SDS_PIN_TX);
    delay(100);
#else
    serialSDS.begin(9600, SWSERIAL_8N1, SDS_PIN_RX, SDS_PIN_TX, false, 192);
#endif

    DebugLn("SDS011 start/stop and reporting sample");

    Sds011::Report_mode report_mode;
    if (!sds011.get_data_reporting_mode(report_mode)) {
        DebugLn("Sds011::get_data_reporting_mode() failed");
    }
    if (Sds011::REPORT_ACTIVE != report_mode) {
        DebugLn("Turning on Sds011::REPORT_ACTIVE reporting mode");
        if (!sds011.set_data_reporting_mode(Sds011::REPORT_ACTIVE)) {
            DebugLn("Sds011::set_data_reporting_mode(Sds011::REPORT_ACTIVE) failed");
        }
    }
}

/**************************************************************************/
/*     sds011 send data to MQTT broker
/**************************************************************************/
void sendSDS011data () {
  mqtt_send("data/pm10", String(pm10f), false);
  mqtt_send("data/pm25", String(pm25f), false);
  
}

/**************************************************************************/
/*     registerSDS011capture function
/**************************************************************************/
void registerSDS011capture() {

    start_SDS();
    DebugLn("started SDS011 (is running = " + String(is_SDS_running) + ")");
  
    sds011.on_query_data_auto_completed([](int n) {
        DebugLn("Begin Handling SDS011 query data");
        digitalWrite(GREEN_LED, LEDON);
        int pm25;
        int pm10;
        DebugLn("n = " + String(n));
        if (sds011.filter_data(n, pm25_table, pm10_table, pm25, pm10) &&
            !isnan(pm10) && !isnan(pm25)) {
              pm10f = float(pm10) / 10;
              pm25f = float(pm25) / 10;
              DebugLn("PM10raw: " + String(pm10f));
              DebugLn("PM2.5raw: " + String(pm25f));
              humidityCompensation();
              DebugLn("PM10: " + String(pm10f));
              DebugLn("PM2.5: " + String(pm25f));
              sendSDS011data();
        }
        stop_SDS();
        digitalWrite(GREEN_LED, LEDOFF);
        DebugLn("End Handling SDS011 query data");

        // Send data to Wunderground
        sendDataWunderground();

    });

    if (!sds011.query_data_auto_async(TABLESIZE, pm25_table, pm10_table)) {
        DebugLn("measurement capture start failed");
    }
  
}
/**************************************************************************/
/*     Start SDS
/**************************************************************************/
void start_SDS() {
    DebugLn("Start wakeup SDS011");

    if (sds011.set_sleep(false)) { is_SDS_running = true; }
}

/**************************************************************************/
/*     Stop SDS
/**************************************************************************/
void stop_SDS() {
    DebugLn("Start sleep SDS011");

    if (sds011.set_sleep(true)) { is_SDS_running = false; }
}


void firstSetup(){
  DebugLn("First Setup ->");
  DebugLn("Magic->"+String(settings.data.magic));
  DebugLn("Setting up AP");
  WiFi.mode(WIFI_AP);             //Only Access point
  WiFi.softAP(ap_ssid, NULL, 8);  //Start HOTspot removing password will disable security
  delay(50);
  setupserver.on("/", handleSetup);
  setupserver.on("/initForm", handleInitForm);
  DebugLn("Server Begin");
  setupserver.begin(); 
  delay(100);
  do {
    setupserver.handleClient(); 
    delay(500);
    Debug(".");
  }
  while (!ap_setup_done);
  settings.data.magic[0] = MAGIC[0];
  settings.data.magic[1] = MAGIC[1];
  settings.data.magic[2] = MAGIC[2];
  settings.data.magic[3] = MAGIC[3];
  setupserver.stop();
  WiFi.disconnect();
  settings.Save();
  DebugLn("First Setup Done");
}


void handleSetup() {
  DebugLn("handlesetup");
  String s = FPSTR(INIT_page);
  s.replace("@@SSID@@", settings.data.ssid);
  s.replace("@@PSK@@", settings.data.psk);
  s.replace("@@CLOCKNAME@@", settings.data.name);
  s.replace("@@VERSION@@",FVERSION);
  s.replace("@@UPDATERESPONSE@@", httpUpdateResponse);
  httpUpdateResponse = "";
  setupserver.send(200, "text/html", s);
}

void handleInitForm() {
  DebugLn("handleInitForm");
  DebugLn("Mode ="+String(WiFi.status()));

  String t_ssid = setupserver.arg("ssid");
  String t_psk = setupserver.arg("psk");
  String t_name = setupserver.arg("clockname");
  String(t_name).replace("+", " ");
  t_ssid.toCharArray(settings.data.ssid,SSID_LENGTH);
  t_psk.toCharArray(settings.data.psk,PSK_LENGTH);
  t_name.toCharArray(settings.data.name,NAME_LENGTH);
  httpUpdateResponse = "The configuration was updated.";
  setupserver.sendHeader("Location", "/");
  setupserver.send(302, "text/plain", "Moved");
  settings.Save();
  ap_setup_done = 1;
}

void handleFinger() {
  DebugLn("handleFinger");
  
  String s = FPSTR(FINGER_page);
  s.replace("@@FINGERPTR@@", settings.data.wunderfinger);
  s.replace("@@UPDATERESPONSE@@", httpUpdateResponse);
  server.send(200, "text/html", s);
  httpUpdateResponse = "";

}

void handleFingerForm() {
  DebugLn("handleFingerForm");

  String fingerptr = server.arg("fingerptr");
  fingerptr.toCharArray(settings.data.wunderfinger,FINGERPRINT_LENGTH);
  httpUpdateResponse = "Fingerprint updated.";
  server.sendHeader("Location", "/fingerprint");
  server.send(302, "text/plain", "Moved");
  httpUpdateResponse = "";

  Debug("Updated fingerprint: ");
  DebugLn(settings.data.wunderfinger);
  settings.Save();
}

void handleRoot() {
  DebugLn("handleRoot");
 
  // calculate Battery level
  batt = GetBattLevel();
  String s = FPSTR(MAIN_page);
  s.replace("@@SSID@@", settings.data.ssid);
  s.replace("@@PSK@@", settings.data.psk);
  s.replace("@@CLOCKNAME@@", settings.data.name);
  s.replace("@@VERSION@@",FVERSION);
  s.replace("@@PM10@@",String(pm10f));
  s.replace("@@PM25@@",String(pm25f));
  s.replace("@@TEMPERATURE@@",String(temperature));
  s.replace("@@HUMIDITY@@",String(humidity));
  s.replace("@@BATT@@", String(batt));
  s.replace("@@UPDATERESPONSE@@", httpUpdateResponse);
  s.replace("@@MQTTBROKER@@",settings.data.mqttbroker);
  s.replace("@@MQTTPORT@@",String(settings.data.mqttport));
  s.replace("@@MQTTUSER@@",settings.data.mqttuser);
  s.replace("@@MQTTPASSWD@@",settings.data.mqttpassword);
  s.replace("@@MQTTTOPIC@@",settings.data.mqtttopic);
  s.replace("@@POOLINT@@",String(settings.data.poolinterval));
  s.replace("@@STATIONID@@",settings.data.stationID);
  s.replace("@@STATIONKEY@@",settings.data.stationKey);
  if (settings.data.ecomode) {
    s.replace("@@ECOMODE@@",String(settings.data.ecomode)+String(" Checked"));
    DebugLn("ECO " + String(settings.data.ecomode) + " ECO checked");
  }
  else {
    s.replace("@@ECOMODE@@",String(settings.data.ecomode));
    DebugLn("ECO =" + String(settings.data.ecomode));
  }
  httpUpdateResponse = "";
  server.send(200, "text/html", s);
}

void handleRowData() {
  DebugLn("handleRowData");

  batt = GetBattLevel();

  httpUpdateResponse = "";
  server.send(200, "text/html", 
                   String("{\"Type\":\"AIRQUAL\",\"data\":") +
                        "{ \"battery\":" + String(batt) + 
                        ", \"pm10\":" + String(pm10f) + 
                        ", \"pm25\":" + String(pm25f) + 
                        ", \"temperature\":" + String(temperature) +
                        ", \"humidity\":" + String(humidity) +
                        ", \"running\":" + String(is_SDS_running) + 
                        "}}\r\n"
                   );
}

void sendHTTP (String httpPost){
  HTTPClient http;
  DebugLn(httpPost);
  http.begin(httpPost);
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  int httpCode = http.POST("");
  DebugLn(httpCode);    
  http.end();
}

void sendHTTPsGet (String httpGet, String host_fingerptr){
  HTTPClient http;
  DebugLn("HTTPsGet: " + httpGet);
  http.begin(httpGet, host_fingerptr);
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  int httpCode = http.GET();
  String payload = http.getString();

  DebugLn("HTTPsGet return code: " + String(httpCode));    // this return 200 when success
  DebugLn(payload);     // this will get the response
  http.end();

}

void handleIO() {
  DebugLn("handleIO");

  // Report Data
  batt = GetBattLevel();
  DebugLn("Batt: "+String(batt)+"%");
  mqtt_send("data/battery", String(batt), false);

  // Read DHT22 data (temp, humid)
  humidity = dht.readHumidity();
  temperature = dht.readTemperature();
  DebugLn("Temp: "+String(temperature)+" ºC, Humid: "+String(humidity)+"%");
  mqtt_send("data/temperature", String(temperature), false);
  mqtt_send("data/humidity", String(humidity), false);

  DebugLn("ECO Mode="+String(settings.data.ecomode));
}


void handleForm() {
  String aux;

  DebugLn("handleForm");
  DebugLn("Mode ="+String(WiFi.status()));
  String update_wifi = server.arg("update_wifi");
  String eco_mode = server.arg("eco_mode");
  DebugLn("update_wifi: " + update_wifi +", eco_mode: "+eco_mode);
  String t_ssid = server.arg("ssid");
  String t_psk = server.arg("psk");
  String t_name = server.arg("clockname");
  String(t_name).replace("+", " ");
  if (update_wifi == "1") {
    t_ssid.toCharArray(settings.data.ssid,SSID_LENGTH);
    t_psk.toCharArray(settings.data.psk,PSK_LENGTH);
    t_name.toCharArray(settings.data.name,NAME_LENGTH);
  }

  String brokerprev = String(settings.data.mqttbroker);
  int portprev = settings.data.mqttport;
  
  aux = server.arg("mqttbroker");
  aux.toCharArray(settings.data.mqttbroker,128);
  aux = server.arg("mqttuser");
  aux.toCharArray(settings.data.mqttuser,30);
  aux = server.arg("mqttpasswd");
  aux.toCharArray(settings.data.mqttpassword,30);
  aux = server.arg("mqtttopic");
  aux.toCharArray(settings.data.mqtttopic,30);
  aux = server.arg("mqttport");
  if (aux.length()) {
    settings.data.mqttport=aux.toInt();
  }
  String pint = server.arg("poolint");
  if (pint.length()) {
    settings.data.poolinterval=constrain(pint.toInt(),10,3600);
  }
//  mqtt_send("setup/poolint", String(settings.data.poolinterval), false);

  aux = server.arg("stationid");
  aux.toCharArray(settings.data.stationID,128);
  aux = server.arg("stationkey");
  aux.toCharArray(settings.data.stationKey,128);

  if (eco_mode != "") {
    settings.data.ecomode=1;
    start=millis();  // wait 5 minutes to enter in eco mode
  }
  else {
    settings.data.ecomode=0;
  }

  httpUpdateResponse = "The configuration was updated.";
  ap_setup_done = 1;
  server.sendHeader("Location", "/");
  server.send(302, "text/plain", "Moved");
  settings.Save();

  if (String(settings.data.mqttbroker) != brokerprev || settings.data.mqttport != portprev) {
    // MQTT broker or port has changed. restart
    client.disconnect();
    mqtt_init();
  }
  
  if (update_wifi != "") {
    delay(500);
    setupSTA();             // connect to Wifi
  }
}

/**************************************************
 * Callback function for MQTT
 * ***********************************************/
void callback(char* topic, byte* payload, unsigned int length) {
char* buff;
const char* nomsg="";

  Debug("MQTT message received [");
  Debug(String(topic));
  Debug("] :");

  buff = (char*)malloc(length+1);
  memcpy(buff, payload, length);
  buff[length] = '\0';
  DebugLn(buff);

  // is payload no NULL?
  if (length > 0) { 
    if (String(topic) == (base_topic + "/setup/poolint") ) {
      int poolintprev = settings.data.poolinterval;
      settings.data.poolinterval = atoi(buff);
      // save settings only if parameter has changed
      if (poolintprev != settings.data.poolinterval) settings.Save();  
      // remove retained message
      client.publish((base_topic + "/setup/poolint").c_str(), nomsg, true);
    }
    else if (String(topic) == (base_topic + "/setup/ecomode") ) {
      int ecomodeprev = settings.data.ecomode;
      settings.data.ecomode = atoi(buff);
      // save settings only if parameter has changed
      if (ecomodeprev != settings.data.ecomode) settings.Save();  
      // remove retained message
      client.publish((base_topic + "/setup/ecomode").c_str(), nomsg, true);
    }
  }
  
  free(buff);
}

/*************************************************
 * MQTT init function
 * **********************************************/
void mqtt_init() {

  client.setServer(settings.data.mqttbroker, settings.data.mqttport);
  DebugLn("setServer: " + String(settings.data.mqttbroker) + ", port: " +String(settings.data.mqttport));
  client.setCallback(callback);
  mqtt_reconnect();

}

/*************************************************
 * MQTT reconnect function
 * **********************************************/
bool mqtt_reconnect() {
int res;

  if (!client.connected() ) {
    DebugLn("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = String(settings.data.name) + String(random(0xffff), HEX);
    // Attempt to connect

    if(strlen(settings.data.mqttuser)) {
      res = client.connect(clientId.c_str(), settings.data.mqttuser, settings.data.mqttpassword);
    }
    else {
      res = client.connect(clientId.c_str());
    }
    if (res) {
      DebugLn("MQTT connected");
      // once connect ....resubscribe
      base_topic = String(settings.data.mqtttopic) + "/" + String(settings.data.name);
      DebugLn("base topic: " + base_topic);
      client.subscribe((base_topic + "/setup/#").c_str());
      return true;
    } else {
      Debug ("MQTT reconnecxion failed, rc=");
      DebugLn (client.state());
      return false;
    }
  }
}

/*************************************************
 * MQTT send function
 * **********************************************/
void mqtt_send(String subtopic, String message, bool retain){

String topic = base_topic + "/" + subtopic;

  if(mqtt_reconnect() ) {
    // send data to topic
    client.publish(topic.c_str(), message.c_str(), retain);
    Debug("mqtt send [" );
    Debug(topic);
    Debug("]: ");
    DebugLn(message);
  }
}

int setupSTA()
{
  int timeOut=0;

  for (int retry=0; retry<=3; retry++) {
    WiFi.disconnect();
    WiFi.hostname("ESP_" + String(settings.data.name)) ;
    WiFi.mode(WIFI_STA);
    DebugLn("Connecting to "+String(settings.data.ssid)+" Retry:"+String(retry));
    DebugLn("Connecting to "+String(settings.data.psk));
    
    if (String(settings.data.psk).length()) {
      WiFi.begin(String(settings.data.ssid), String(settings.data.psk));
    } else {
      WiFi.begin(String(settings.data.ssid));
    }
    
    timeOut=0;
    while (WiFi.status() != WL_CONNECTED) {
      if ((timeOut < 10) && WiFi.status() != WL_CONNECT_FAILED){ // if not timeout or failure, keep trying
        //delay(100);
        Debug(String(WiFi.status()));
        if (!inecomode){
          displayBusy();
        }
        else {
          delay(1000);
        }
        timeOut ++;
      } 
      else{
        timeOut = 0;
        DebugLn("-Wifi connection timeout");
        displayBusy();
        if (retry == 2) 
          return 0;
        break;
      }
    }
    if (WiFi.status() == WL_CONNECTED)
      break;
  }  
  DebugLn(" Connected");
  // Print the IP address
  DebugLn(WiFi.localIP()); 
  DebugLn(WiFi.hostname().c_str());
  return 1;
}

void sendDataWunderground(){

  if(strlen(settings.data.stationID) > 0 && strlen(settings.data.stationKey) > 0 && strlen(settings.data.wunderfinger) >0 ){
    // Send data to Wunderground
    DebugLn("--- Sending data to Wunderground ---");
    String  weatherData =  "https://rtupdate.wunderground.com/weatherstation/updateweatherstation.php?";
    weatherData += "ID=" + String(settings.data.stationID);
    weatherData += "&PASSWORD=" + String(settings.data.stationKey);
    weatherData += "&dateutc=now";
    weatherData += "&action=updateraw";
  
  #ifdef SDS011
    // solar radiance
  //  weatherData += "&solarradiation=" + String (radiance);
  #endif
  
    // send to Wunderground
    sendHTTPsGet(weatherData,settings.data.wunderfinger);
  }
                
}

void setup() {

  DebugStart();
  DebugLn("Setup ->");
  myResetInfo = ESP.getResetInfoPtr();
  DebugLn("myResetInfo->reason "+String(myResetInfo->reason)); // reason is uint32
                                                                 // 0 = power down
                                                                 // 6 = reset button
                                                                 // 5 = restart from deepsleep
                                                                 
  //*************** all modes, check if comming from ECO mode or not *****************
  settings.Load();
  inecomode = false;
  if ((myResetInfo->reason==5) && (settings.data.inecomode == true) && (settings.data.ecomode) ) {
    inecomode = true;
  }
  else {
    settings.data.inecomode = false;
    settings.Save();
  }
  DebugLn("Inecomode? "+String(inecomode));
  // ******** NO ECO mode, initiallize LEDs and wait for setup pin *****************
  if (!inecomode) {
    pinMode(RED_LED, OUTPUT);
    pinMode(GREEN_LED, OUTPUT);
#ifdef _DHT
    pinMode(DHT_PIN, OUTPUT);
#endif
    DebugLn("-> Check if SETUP_PIN is low");
    digitalWrite(RED_LED, LEDON);
    digitalWrite(GREEN_LED, LEDON);
    // Wait up to 5s for SETUP_PIN to go low to enter AP/setup mode.
    pinMode(SETUP_PIN, INPUT);      //Configure setup pin as input
    digitalWrite(SETUP_PIN, HIGH);  //Enable internal pooling
    delay(5000);  
    DebugLn("Magic ->"+String(settings.data.magic));

    // if NO MAGIC Or SETUP PIN enter hard config...
    if ((String(settings.data.magic) != MAGIC)   || !digitalRead(SETUP_PIN)){
      digitalWrite(GREEN_LED, LEDOFF);
      digitalWrite(RED_LED, LEDON);
      firstSetup();
    }
  }
  // NO SETUP, switch off both LEDs
  digitalWrite(GREEN_LED, LEDOFF);
  digitalWrite(RED_LED, LEDOFF);

  //********* all modes - initialize sensors *****************
#ifdef SDS011
  SDS011setup();
#endif
  dht.begin();              // Start DHT
  DebugLn("-> DHT begin");

  // *********** all modes, setup STA mode and connect to WiFi ************
  if (setupSTA() == 0) { // SetupSTA mode
    DebugLn("Wifi no connected");
    if (inecomode) {
      // if ECO mode go to deep sleep for poolinterval secs.
      DebugLn("wait in sleep mode...");
      ESP.deepSleep(1e6*settings.data.poolinterval);
    }
    else{
      // if not ECO mode, wait poolinterval and restart
      DebugLn("wait " + String(settings.data. poolinterval) + " sec. and restart");
      displayBusy();
      delay(1000*settings.data.poolinterval);
      ESP.restart();
    }
  }

  // ********** initialize OTA *******************
  ArduinoOTA.begin();

  // ********* initialize MQTT ******************
  mqtt_init();
  
  //********** NO ECO mode, switch ON GREEN LED, initialize web servers and switch OFF LEDs
  if (!inecomode) {
    digitalWrite(GREEN_LED, LEDON);
    digitalWrite(RED_LED, LEDOFF);
    DebugLn("-> Initiate WebServer");
    server.on("/", handleRoot);
    server.on("/setup", handleRoot);
    server.on("/form", handleForm);
    server.on("/data",handleRowData);
    server.on("/fingerprint",handleFinger);
    server.on("/fingerForm",handleFingerForm);
    //server.on("/io", handleIO);
    delay(100);
    server.begin(); 
     delay(3000);
    digitalWrite(GREEN_LED, LEDOFF);
    digitalWrite(RED_LED, LEDOFF);  
  }
  
  start = millis();
  DebugLn("Started at "+String(start));

  // initiate SDS011 capture
  registerSDS011capture();
  
  // Send data to Jeedom
  handleIO();
  
  // check MQTT received messages
  for (int i=0; i < 10; i++) {
    client.loop();
    delay(10);
  }

  
  timer = 0;
  timer2 = millis();
  DebugLn("-> End Setup");
}

void loop() {

  if (timer > settings.data.poolinterval*9) {
      timer = 0;
      // initiate SDS011 capture
      registerSDS011capture();
      handleIO();
      // sendDataWunderground();
    }
    else {                  // handle http client while waiting
      timer++;
      server.handleClient();    
      delay(100);
    }
 
  // Perform DSD011 read cycle
  if (is_SDS_running) {
    sds011.perform_work();
  }
  else {
    // if SDS is not running, check eco mode
      if (inecomode) {
        //********** ECO mode, go to deep sleep state ******************
        DebugLn("Go to Sleep mode...");
        delay(500);
        ESP.deepSleep(1e6*settings.data.poolinterval);
      }
  }

  // handle OTA
  ArduinoOTA.handle();

  // handle MQTT
  client.loop();
  
  if ((millis() > start + 300000) && (settings.data.ecomode)) {; // First 5 minutes
    digitalWrite(GREEN_LED, LEDON);
    digitalWrite(RED_LED, LEDON);
    DebugLn("Entering Sleep mode...");
    delay(2000);    
    digitalWrite(GREEN_LED, LEDOFF);
    digitalWrite(RED_LED, LEDOFF);
    //wifi_set_sleep_type(LIGHT_SLEEP_T);
    settings.data.inecomode = true;
    settings.Save(); 
    ESP.deepSleep(1e6*settings.data.poolinterval);
  }    
}
