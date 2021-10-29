#include <LiquidCrystal_PCF8574.h>
#include <Wire.h>

LiquidCrystal_PCF8574 lcd(0x27); // set the LCD address to 0x27 for a 16 chars and 2 line display

int show = -1;

#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif



BluetoothSerial SerialBT;


#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <ArduinoOTA.h>
#include <Update.h>
#include <ArduinoJson.h>

#include <EEPROM.h>

#include <ModbusMaster.h>
#include "REG_CONFIG.h"
#include <HardwareSerial.h>
#include "HardwareSerial_NB_BC95.h"

#define trigWDTPin    32
#define ledHeartPIN   0

#include <TaskScheduler.h>

#define _TASK_TIMECRITICAL
HardwareSerial_NB_BC95 AISnb;
HardwareSerial modbus(2);



/**********************************************  WIFI Client 注意编译时要设置此值 *********************************
   wifi client
*/
const char* ssid = "greenio"; //replace "xxxxxx" with your WIFI's ssid
const char* password = "green7650"; //replace "xxxxxx" with your WIFI's password

//WiFi&OTA 参数
String HOSTNAME = "EkaratSingle";
#define PASSWORD "green7650" //the password for OTA upgrade, can set it in any char you want

/************************************************  注意编译时要设置此值 *********************************
   是否使用静态IP
*/
#define USE_STATIC_IP false
#if USE_STATIC_IP
IPAddress staticIP(192, 168, 1, 22);
IPAddress gateway(192, 168, 1, 9);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns1(8, 8, 8, 8);
IPAddress dns2(114, 114, 114, 114);
#endif




String deviceToken = "8966031940014308369";
String serverIP = "147.50.151.130"; // Your Server IP;
String serverPort = "19956"; // Your Server Port;

String json = "";

ModbusMaster node;
void t1CallgetMeter();
void t2CallsendViaNBIOT();
void t3CallLCDUpdate();
//void t4Restart();
//TASK
Task t1(30000, TASK_FOREVER, &t1CallgetMeter);
Task t2(60000, TASK_FOREVER, &t2CallsendViaNBIOT);
Task t3(5000, TASK_FOREVER, &t3CallLCDUpdate);
//Task t4(3600000, TASK_FOREVER, &t4Restart);

Scheduler runner;
String _config = "";
unsigned long _epoch = 0;
String _IP = "";
String dataJson = "";
boolean validEpoc = false;

StaticJsonDocument<400> doc;

struct Meter
{

  String moisture;
  String temp;
  String EC;
  String Ph;
  String Ni;
  String Pho;
  String Pot;


};
Meter meter;
signal meta;


//********************************************************************//
//*********************** HeartBeat Function **************************//
//********************************************************************//
void HeartBeat() {
  //   Sink current to drain charge from watchdog circuit
  pinMode(trigWDTPin, OUTPUT);
  digitalWrite(trigWDTPin, LOW);

  // Led monitor for Heartbeat
  digitalWrite(ledHeartPIN, LOW);
  delay(300);
  digitalWrite(ledHeartPIN, HIGH);

  // Return to high-Z
  pinMode(trigWDTPin, INPUT);

  Serial.println("Heartbeat sent");
  //    SerialBT.println("Heartbeat sent");
}


String getMacAddress() {
  uint8_t baseMac[6];
  // Get MAC address for WiFi station
  esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
  char baseMacChr[18] = {0};
  sprintf(baseMacChr, "%02X%02X%02X%02X%02X%02X", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
  return String(baseMacChr);
}

char  char_to_byte(char c)
{
  if ((c >= '0') && (c <= '9'))
  {
    return (c - 0x30);
  }
  if ((c >= 'A') && (c <= 'F'))
  {
    return (c - 55);
  }
}


void _init() {

  AISnb.setupDevice(serverPort);
  HeartBeat();

  do {
    UDPSend udp = AISnb.sendUDPmsgStr(serverIP, serverPort, _config);
    dataJson = "";
    deviceToken = AISnb.getNCCID();
    Serial.print("nccid:");
    Serial.println(deviceToken);
    _config = "{\"_type\":\"retrattr\",\"Tn\":\"";
    _config.concat(deviceToken);
    _config.concat("\",\"keys\":[\"epoch\",\"ip\"]}");
    Serial.println(_config);
    HeartBeat();
    UDPReceive resp = AISnb.waitResponse();
    AISnb.receive_UDP(resp);
    Serial.print("waitData:");
    Serial.println(resp.data);


    for (int x = 0; x < resp.data.length(); x += 2)
    {
      char c =  char_to_byte(resp.data[x]) << 4 | char_to_byte(resp.data[x + 1]);

      dataJson += c;
    }
    Serial.println(dataJson);
    DeserializationError error = deserializeJson(doc, dataJson);

    // Test if parsing succeeds.
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      validEpoc = true;
      delay(4000);
    } else {
      validEpoc = false;
      unsigned long epoch = doc["epoch"];
      _epoch = epoch;
      String ip = doc["ip"];
      _IP = ip;
      Serial.println(dataJson);
      Serial.print("epoch:");  Serial.println(_epoch);
      _writeEEPROM(_IP);
      Serial.println(_IP);

    }
    delay(5000);
    HeartBeat();
  } while (validEpoc);


}


void writeString(char add, String data)
{
  EEPROM.begin(512);
  int _size = data.length();
  int i;
  for (i = 0; i < _size; i++)
  {
    EEPROM.write(add + i, data[i]);
  }
  EEPROM.write(add + _size, '\0'); //Add termination null character for String Data
  EEPROM.commit();
}


String read_String(char add)
{
  int i;
  char data[100]; //Max 100 Bytes
  int len = 0;
  unsigned char k;
  k = EEPROM.read(add);
  while (k != '\0' && len < 500) //Read until null character
  {
    k = EEPROM.read(add + len);
    data[len] = k;
    len++;
  }
  data[len] = '\0';
  Serial.print("Debug:");
  Serial.println(String(data));
  return String(data);
}

void _initLCD() {
  int error;


  Serial.println("LCD...");



  Serial.println("Dose: check for LCD");

  // See http://playground.arduino.cc/Main/I2cScanner how to test for a I2C device.
  Wire.begin();
  Wire.beginTransmission(0x27);
  error = Wire.endTransmission();
  Serial.print("Error: ");
  Serial.print(error);

  if (error == 0) {
    Serial.println(": LCD found.");
    show = 0;
    lcd.begin(16, 2); // initialize the lcd

  } else {
    Serial.println(": LCD not found.");
  } // if
  lcd.setBacklight(255);
  lcd.home();
  lcd.clear();
  lcd.print("Hello SoilSensor");

  delay(4000);
}
void setupOTA()
{
  //Port defaults to 8266
  //ArduinoOTA.setPort(8266);

  //Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(HOSTNAME.c_str());

  //No authentication by default
  ArduinoOTA.setPassword(PASSWORD);

  //Password can be set with it's md5 value as well
  //MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  //ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]()
  {
    Serial.println("Start Updating....");

    Serial.printf("Start Updating....Type:%s\n", (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem");
  });

  ArduinoOTA.onEnd([]()
  {

    Serial.println("Update Complete!");
    SerialBT.println("Update Complete!");
    ESP.restart();
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
  {
    String pro = String(progress / (total / 100)) + "%";
    int progressbar = (progress / (total / 100));
    //int progressbar = (progress / 5) % 100;
    //int pro = progress / (total / 100);


    SerialBT.printf("Progress: %u%%\n", (progress / (total / 100)));
    Serial.printf("Progress: %u%%\n", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error)
  {
    Serial.printf("Error[%u]: ", error);
    String info = "Error Info:";
    switch (error)
    {
      case OTA_AUTH_ERROR:
        info += "Auth Failed";
        Serial.println("Auth Failed");
        break;

      case OTA_BEGIN_ERROR:
        info += "Begin Failed";
        Serial.println("Begin Failed");
        break;

      case OTA_CONNECT_ERROR:
        info += "Connect Failed";
        Serial.println("Connect Failed");
        break;

      case OTA_RECEIVE_ERROR:
        info += "Receive Failed";
        Serial.println("Receive Failed");
        break;

      case OTA_END_ERROR:
        info += "End Failed";
        Serial.println("End Failed");
        break;
    }


    Serial.println(info);
    ESP.restart();
  });

  ArduinoOTA.begin();
}

void setupWIFI()
{

  Serial.println("Connecting...");
  Serial.println(String(ssid));


  //连接WiFi，删除旧的配置，关闭WIFI，准备重新配置
  WiFi.disconnect(true);
  delay(1000);

  WiFi.mode(WIFI_STA);
  //WiFi.onEvent(WiFiEvent);
  WiFi.setAutoConnect(true);
  WiFi.setAutoReconnect(true);    //断开WiFi后自动重新连接,ESP32不可用
  WiFi.setHostname(HOSTNAME.c_str());
  WiFi.begin(ssid, password);
#if USE_STATIC_IP
  WiFi.config(staticIP, gateway, subnet);
#endif

  //等待5000ms，如果没有连接上，就继续往下
  //不然基本功能不可用
  byte count = 0;
  while (WiFi.status() != WL_CONNECTED && count < 10)
  {
    count ++;
    delay(500);
    Serial.print(".");
  }


  if (WiFi.status() == WL_CONNECTED)
    Serial.println("Connecting...OK.");
  else
    Serial.println("Connecting...Failed");

}

void setup()
{
  pinMode(ledHeartPIN, OUTPUT);

  // Sink current to drain charge from watchdog circuit
  pinMode(trigWDTPin, OUTPUT);
  digitalWrite(trigWDTPin, LOW);

  Serial.begin(115200);
  HOSTNAME.concat(getMacAddress());
  SerialBT.begin(HOSTNAME); //Bluetooth
  

  Serial.println();
  Serial.println(F("***********************************"));
  Serial.println("Initialize...");
  SerialBT.println(F("***********************************"));
  SerialBT.println("Initialize...");

  _initLCD();
  HeartBeat();
  runner.init();
  Serial.println("Initialized scheduler");

  runner.addTask(t1);
  Serial.println("added t1");
  runner.addTask(t2);
  Serial.println("added t2");
  runner.addTask(t3);
  Serial.println("added t3");
  //  runner.addTask(t4);
  //  Serial.println("added t4");
  HeartBeat();


  t1.enable();  Serial.println("Enabled t1");
  t2.enable();  Serial.println("Enabled t2");
  t3.enable();  Serial.println("Enabled t3");
  //  t4.enable();  Serial.println("Enabled t4");

  HeartBeat();


  //  _init();
  HeartBeat();

  //  _loadConfig();
  HeartBeat();



  modbus.begin(4800, SERIAL_8N1, 16, 17);
  HeartBeat();
  setupWIFI();
  setupOTA();
}

void t2CallsendViaNBIOT ()
{
  Serial.println("#####################t2CallsendViaNBIOT###################");
  /*
    meta = AISnb.getSignal();
    HeartBeat();
    Serial.print("RSSI:"); Serial.println(meta.rssi);

    json = "";
    json.concat("{\"Tn\":\"");
    json.concat(deviceToken);
    json.concat("\"");


    json.concat(",\"sdmVolt\":");
    json.concat(meter.sdmVolt);
    json.concat(",\"sdmCur\":");
    json.concat(meter.sdmCurrent);
    json.concat(",\"sdmATP\":");
    json.concat(meter.sdmActivePower);
    json.concat(",\"sdmPF\":");
    json.concat(meter.sdmPowerFactor);
    json.concat(",\"sdmF\":");
    json.concat(meter.sdmFrequency);
    json.concat(",\"sdmTAE\":");
    json.concat(meter.sdmTotalActiveEnergy);

    json.concat(",\"rssi\":");
    json.concat(meta.rssi);
    json.concat(",\"csq\":");
    json.concat(meta.csq);
    json.concat("}");

    Serial.println(json);
    SerialBT.println(json);

    //

    UDPSend udp = AISnb.sendUDPmsgStr(serverIP, serverPort, json);
    Serial.print("udp.strsend:"); Serial.println(udp.strsend);
    Serial.print("udp.status:"); Serial.println(udp.status);

  */
  HeartBeat();
}


int read_Modbus_1Byte(char addr, uint16_t  REG)
{
  static uint32_t i;
  int8_t j, result;
  int16_t data[2];
  int32_t value = 0;
  float val = 0.0;

  // communicate with Modbus slave ID 1 over Serial (port 2)
  node.begin(addr, modbus);

  // slave: read (6) 16-bit registers starting at register 2 to RX buffer
  result = node.readHoldingRegisters(REG, 1);

  // do something with data if read is successful
  if (result == node.ku8MBSuccess)
  {
    for (j = 0; j < 1; j++)
    {
      data[j] = node.getResponseBuffer(j);
    }
    value = data[0];

    return value;
  } else {
    Serial.print("Connec modbus fail. REG >>> "); Serial.println(REG, HEX); // Debug
    //    delay(1000);
    return NULL;
  }
}

void _writeEEPROM(String data) {
  Serial.print("Writing Data:");
  Serial.println(data);

  writeString(10, data);  //Address 10 and String type data
  delay(10);
}

void _loadConfig() {
  serverIP = read_String(10);
  serverIP.trim();
  Serial.print("IP:");
  Serial.println(serverIP);
}

void readMeter()
{


  meter.moisture = readModbus(ID_SENSOR, Address[0]);//แสกนหลายตัวตามค่า ID_METER_ALL=X
  meter.temp = readModbus(ID_SENSOR, Address[1]); //แสกนหลายตัวตามค่า ID_METER_ALL=X

  meter.EC = readModbus(ID_SENSOR, Address[2]); //แสกนหลายตัวตามค่า ID_METER_ALL=X
  meter.Ph = readModbus(ID_SENSOR, Address[3]); //แสกนหลายตัวตามค่า ID_METER_ALL=X
  meter.Ni = readModbus(ID_SENSOR, Address[4]); //แสกนหลายตัวตามค่า ID_METER_ALL=X
  meter.Pho = readModbus(ID_SENSOR, Address[5]); //แสกนหลายตัวตามค่า ID_METER_ALL=X
  meter.Pot = readModbus(ID_SENSOR, Address[6]); //แสกนหลายตัวตามค่า ID_METER_ALL=X

  Serial.print("moisture: ");  Serial.println(meter.moisture);
  Serial.print("temp: ");  Serial.println(meter.temp);
  Serial.print("EC: ");  Serial.println(meter.EC);
  Serial.print("Ph: ");  Serial.println(meter.Ph);
  Serial.print("Ni: ");  Serial.println(meter.Ni);
  Serial.print("Pho: ");  Serial.println(meter.Pho);
  Serial.print("Pot: ");  Serial.println(meter.Pot);


  Serial.println("");
  Serial.println("");
  HeartBeat();
}

void t1CallgetMeter() {     // Update read all data

  readMeter();


}
void t3CallLCDUpdate() {

  lcd.clear();

  lcd.print("Mo:" + meter.moisture);
  lcd.print(" T:" + meter.temp);
  lcd.print(" EC:" + meter.EC);
  lcd.setCursor(0,1);
  lcd.print("Ph:"+ meter.Ph);
  lcd.print(" NPK:"+meter.Ni);
  lcd.print(" "+meter.Pho);
  lcd.print(" "+meter.Pot);
  

}

float HexTofloat(uint32_t x)
{
  return (*(float*)&x);
}



uint32_t FloatTohex(float x) {
  return (*(uint32_t*)&x);
}

uint16_t readModbus(char addr , uint16_t  REG) {
  float i = 0;
  uint8_t j, result;
  uint16_t data[1];
  uint32_t value = 0;
  for (int i = 0; i < 1; i++) {
    node.begin(addr, modbus);
    result = node.readHoldingRegisters (REG, 1); ///< Modbus function 0x04 Read Input Registers
    delay(500);

    if (result == node.ku8MBSuccess) {
      for (j = 0; j < 1; j++)
      {
        data[j] = node.getResponseBuffer(j);
      }
      value = data[0];
      Serial.println(value);
      //      value = value << 16;
      //      value = value + data[1];
      //      i = HexTofloat(value);
      //Serial.println("Connec modbus Ok.");
      return value;
    }

  }

  Serial.print("Connec modbus fail. REG >>> "); Serial.println(REG, HEX); // Debug
  return 0;
}



void loop()
{
  runner.execute();

  ArduinoOTA.handle();
  unsigned long ms = millis();
  if (ms % 60000 == 0)
  {
    Serial.println("hello，OTA now");
    SerialBT.println("hello，OTA now");
  }
}


void WiFiEvent(WiFiEvent_t event)
{
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch (event)
  {
    case SYSTEM_EVENT_WIFI_READY:               /**< ESP32 WiFi ready */
      break;
    case SYSTEM_EVENT_SCAN_DONE:                /**< ESP32 finish scanning AP */
      break;

    case SYSTEM_EVENT_STA_START:                /**< ESP32 station start */
      break;
    case SYSTEM_EVENT_STA_STOP:                 /**< ESP32 station stop */
      break;

    case SYSTEM_EVENT_STA_CONNECTED:            /**< ESP32 station connected to AP */
      break;

    case SYSTEM_EVENT_STA_DISCONNECTED:         /**< ESP32 station disconnected from AP */
      break;

    case SYSTEM_EVENT_STA_AUTHMODE_CHANGE:      /**< the auth mode of AP connected by ESP32 station changed */
      break;

    case SYSTEM_EVENT_STA_GOT_IP:               /**< ESP32 station got IP from connected AP */
    case SYSTEM_EVENT_STA_LOST_IP:              /**< ESP32 station lost IP and the IP is reset to 0 */
      break;

    case SYSTEM_EVENT_STA_WPS_ER_SUCCESS:       /**< ESP32 station wps succeeds in enrollee mode */
    case SYSTEM_EVENT_STA_WPS_ER_FAILED:        /**< ESP32 station wps fails in enrollee mode */
    case SYSTEM_EVENT_STA_WPS_ER_TIMEOUT:       /**< ESP32 station wps timeout in enrollee mode */
    case SYSTEM_EVENT_STA_WPS_ER_PIN:           /**< ESP32 station wps pin code in enrollee mode */
      break;

    case SYSTEM_EVENT_AP_START:                 /**< ESP32 soft-AP start */
    case SYSTEM_EVENT_AP_STOP:                  /**< ESP32 soft-AP stop */
    case SYSTEM_EVENT_AP_STACONNECTED:          /**< a station connected to ESP32 soft-AP */
    case SYSTEM_EVENT_AP_STADISCONNECTED:       /**< a station disconnected from ESP32 soft-AP */
    case SYSTEM_EVENT_AP_PROBEREQRECVED:        /**< Receive probe request packet in soft-AP interface */
    case SYSTEM_EVENT_AP_STA_GOT_IP6:           /**< ESP32 station or ap interface v6IP addr is preferred */
      break;

    case SYSTEM_EVENT_ETH_START:                /**< ESP32 ethernet start */
    case SYSTEM_EVENT_ETH_STOP:                 /**< ESP32 ethernet stop */
    case SYSTEM_EVENT_ETH_CONNECTED:            /**< ESP32 ethernet phy link up */
    case SYSTEM_EVENT_ETH_DISCONNECTED:         /**< ESP32 ethernet phy link down */
    case SYSTEM_EVENT_ETH_GOT_IP:               /**< ESP32 ethernet got IP from connected AP */
    case SYSTEM_EVENT_MAX:
      break;
  }
}
