#include <Arduino.h>
#include <config.h>
#include <EEPROM.h>
#include <hardware.h>
#include <WiFi.h>
#include <Adafruit_NeoPixel.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <WiFiUdp.h>
#include "sACN.h"
#include <ArduinoOTA.h>
#include <esp_wifi.h>
#include <esp_sleep.h>
#include "ADS1X15.h"

#define SYSLED 2

#define SECRET_SACN_UUID "CAC0C2005-DEAD-BEEF-CAFE-69CC04C6C816"
#define SECRET_SACN_RECV "239.255.0.15" // your sACM receiver's IP address

Adafruit_NeoPixel strip = Adafruit_NeoPixel(LED_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

ADS1115 ADS(0x48);
volatile bool RDY = false;
uint8_t channel = 0;
int16_t val[4] = {0, 0, 0, 0};

int SPS = 0;
uint32_t lastTime = 0;

uint8_t uuidBytes[16] = {
    0xCB, 0xC0, 0xC2, 0x71,
    0xDE, 0xAE, 0xBE, 0xEF,
    0xCA, 0xFE, 0x69, 0xF6,
    0x14, 0xC6, 0xC0, 0xC0};

TaskHandle_t task_led = NULL;

WiFiUDP sacn; // instance of UDP library
Source sender(sacn, 14, 100, uuidBytes, "joystick1", true);
Receiver recv(sacn, 15);

int myUniverse = 14;           // DMX universe
char myDevice[] = "joystick1"; // sender name

const int ssidAddress = 1;      // EEPROM address to store SSID (starting from address 1)
const int passwordAddress = 34; // EEPROM address to store password (starting from address 34)

unsigned long lastScnSend = 0;

unsigned long lastScnForcedOn = 0;
unsigned long scnForcedOnLimit = 5000;
bool scnForceSend = false;
bool calibrationEnable = false;

// DEFINE HERE THE KNOWN NETWORKS
const char *KNOWN_SSID[] = {"P4IT", "szarvas", "clueQuest IoT", "Chirp-IoT"};
const char *KNOWN_PASSWORD[] = {"pappka22", "pappka22", "CQIOT8147QQQ", "stardenburdenhardenbart"};

const int KNOWN_SSID_COUNT = sizeof(KNOWN_SSID) / sizeof(KNOWN_SSID[0]); // number of known networks

unsigned int x_min = 12000;
unsigned int x_max = 12000;
unsigned int y_min = 12000;
unsigned int y_max = 12000;

unsigned int last_reported_x_min = 65535;
unsigned int last_reported_x_max = 65535;
unsigned int last_reported_y_min = 65535;
unsigned int last_reported_y_max = 65535;

unsigned int map_x[SAMPLES] = {0};
unsigned int map_y[SAMPLES] = {0};

unsigned short current_sample_x = 0;
unsigned short current_sample_y = 0;

unsigned int centerX = 0;
unsigned int centerY = 0;

uint16_t ringbuffer[4][SAMPLES];
uint8_t ringbuffer_pos[4] = {0};

/* MQTT Vars */
IPAddress mqtt_server(192, 168, 200, 80);
WiFiClient ethClient;
PubSubClient client(ethClient);
long lastReconnectAttempt = 0;
long lastKeepalive = 0;
unsigned long msgcount = 0;
char clientID[20];

boolean reconnect()
{
  char willtopic[50];
  sprintf(willtopic, "joystick/%s/status", clientID);
  if (client.connect(clientID, "admin", "pappka22", willtopic, 1, true, "disconnected"))
  {
    for (int i = 0; i < 10; i++)
    {
      digitalWrite(SYSLED, LOW);
      delay(10);
      digitalWrite(SYSLED, HIGH);
    }
    Serial.println("MQTT connected");
    // client.subscribe("shellies/#");
    char out[50];
    sprintf(out, "joystick/%s/status", clientID);
    client.publish(out, "reconnected");

    char value[50];
    sprintf(out, "joystick/%s/status/ip", clientID);
    sprintf(value, "%d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3]);
    client.publish(out, value, true);

    sprintf(out, "joystick/%s/axis/x/low", clientID);
    sprintf(value, "%d", x_min);
    client.publish(out, value, true);

    sprintf(out, "joystick/%s/axis/x/high", clientID);
    sprintf(value, "%d", x_max);
    client.publish(out, value, true);

    sprintf(out, "joystick/%s/mode", clientID);
    client.publish(out, scnForceSend ? "active" : "sleep", true);

    sprintf(out, "joystick/%s/calibrate", clientID);
    client.publish(out, "", true);
    client.subscribe(out);

    lastKeepalive = millis();
  }
  return client.connected();
}

void callback(char *topic, byte *payload, unsigned int length)
{
  // handle message arrived
  digitalWrite(SYSLED, LOW);
  payload[length] = '\0';
  msgcount++;
  Serial.print("MQTT message cnt: ");
  Serial.print(msgcount);
  Serial.print(" ");
  Serial.print(topic);
  Serial.print(" ");
  Serial.println((char *)payload);
  digitalWrite(SYSLED, LOW);

  char checktopic[50];

  sprintf(checktopic, "joystick/%s/calibrate", clientID);

  if (strcmp(topic, checktopic) == 0)
  {
    if (payload[0] == '1')
    {
      calibrationEnable = true;
      client.publish(checktopic, "CALIBRATION MODE", true);
    }
    if (payload[0] == '0')
    {
      calibrationEnable = false;
      client.publish(checktopic, "", true);
    }
    if (payload[0] == 's')
    {
      Serial.print("saving values as x_low:");
      Serial.print(x_min);
      Serial.print(" x_max:");
      Serial.print(x_max);
      Serial.print(" y_min:");
      Serial.print(y_min);
      Serial.print(" y_max:");
      Serial.println(y_max);

      EEPROM.put(CALIBRATION_X_LOW_ADDRESS, x_min);
      EEPROM.commit();
      EEPROM.put(CALIBRATION_X_HIGH_ADDRESS, x_max);
      EEPROM.commit();
      EEPROM.put(CALIBRATION_Y_LOW_ADDRESS, y_min);
      EEPROM.commit();
      EEPROM.put(CALIBRATION_Y_HIGH_ADDRESS, y_max);
      EEPROM.commit();
      delay(10);
      calibrationEnable = false;
      client.publish(checktopic, "SAVED", true);
    }
    if (payload[0] == 'r')
    {
      EEPROM.put(CALIBRATION_X_LOW_ADDRESS, 65535);
      EEPROM.commit();
      EEPROM.put(CALIBRATION_X_HIGH_ADDRESS, 0);
      EEPROM.commit();
      EEPROM.put(CALIBRATION_Y_LOW_ADDRESS, 65535);
      EEPROM.commit();
      EEPROM.put(CALIBRATION_Y_HIGH_ADDRESS, 0);
      EEPROM.commit();
      x_min = 65535;
      x_max = 0;
      y_min = 65535;
      y_max = 0;
      calibrationEnable = true;
      client.publish(checktopic, "CALIBRATION MODE", true);
    }
    digitalWrite(SYSLED, HIGH);
    return;
  }
  digitalWrite(SYSLED, HIGH);
}

void fn_task_led(void *parameter)
{
  pinMode(LED_SYSTEM, OUTPUT);
  while (1)
  {
    while (WiFi.isConnected() == true)
    {
      if (scnForceSend)
      {
        digitalWrite(LED_SYSTEM, HIGH);
        delay(250);
        digitalWrite(LED_SYSTEM, LOW);
        delay(250);
      }
      else
      {
        digitalWrite(LED_SYSTEM, HIGH);
        delay(250);
      }
    }
    while (WiFi.isConnected() == false)
    {
      digitalWrite(LED_SYSTEM, HIGH);
      delay(50);
      digitalWrite(LED_SYSTEM, LOW);
      delay(50);
    }
  }

  vTaskDelete(NULL);
}

void WiFiEvent(WiFiEvent_t event)
{
  Serial.printf("[WiFi-event] event: %d\n", event);

  switch (event)
  {
  case ARDUINO_EVENT_WIFI_READY:
    Serial.println("WiFi interface ready");
    break;
  case ARDUINO_EVENT_WIFI_SCAN_DONE:
    Serial.println("Completed scan for access points");
    break;
  case ARDUINO_EVENT_WIFI_STA_START:
    Serial.println("WiFi client started");
    break;
  case ARDUINO_EVENT_WIFI_STA_STOP:
    Serial.println("WiFi clients stopped");
    break;
  case ARDUINO_EVENT_WIFI_STA_CONNECTED:
    Serial.println("Connected to access point");
    break;
  case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
    Serial.println("Disconnected from WiFi access point");
    break;
  case ARDUINO_EVENT_WIFI_STA_AUTHMODE_CHANGE:
    Serial.println("Authentication mode of access point has changed");
    break;
  case ARDUINO_EVENT_WIFI_STA_GOT_IP:
    Serial.print("Obtained IP address: ");
    Serial.println(WiFi.localIP());
    break;
  case ARDUINO_EVENT_WIFI_STA_LOST_IP:
    Serial.println("Lost IP address and IP address is reset to 0");
    break;
  case ARDUINO_EVENT_WPS_ER_SUCCESS:
    Serial.println("WiFi Protected Setup (WPS): succeeded in enrollee mode");
    break;
  case ARDUINO_EVENT_WPS_ER_FAILED:
    Serial.println("WiFi Protected Setup (WPS): failed in enrollee mode");
    break;
  case ARDUINO_EVENT_WPS_ER_TIMEOUT:
    Serial.println("WiFi Protected Setup (WPS): timeout in enrollee mode");
    break;
  case ARDUINO_EVENT_WPS_ER_PIN:
    Serial.println("WiFi Protected Setup (WPS): pin code in enrollee mode");
    break;
  case ARDUINO_EVENT_WIFI_AP_START:
    Serial.println("WiFi access point started");
    break;
  case ARDUINO_EVENT_WIFI_AP_STOP:
    Serial.println("WiFi access point  stopped");
    break;
  case ARDUINO_EVENT_WIFI_AP_STACONNECTED:
    Serial.println("Client connected");
    break;
  case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED:
    Serial.println("Client disconnected");
    break;
  case ARDUINO_EVENT_WIFI_AP_STAIPASSIGNED:
    Serial.println("Assigned IP address to client");
    break;
  case ARDUINO_EVENT_WIFI_AP_PROBEREQRECVED:
    Serial.println("Received probe request");
    break;
  case ARDUINO_EVENT_WIFI_AP_GOT_IP6:
    Serial.println("AP IPv6 is preferred");
    break;
  case ARDUINO_EVENT_WIFI_STA_GOT_IP6:
    Serial.println("STA IPv6 is preferred");
    break;
  case ARDUINO_EVENT_ETH_GOT_IP6:
    Serial.println("Ethernet IPv6 is preferred");
    break;
  case ARDUINO_EVENT_ETH_START:
    Serial.println("Ethernet started");
    break;
  case ARDUINO_EVENT_ETH_STOP:
    Serial.println("Ethernet stopped");
    break;
  case ARDUINO_EVENT_ETH_CONNECTED:
    Serial.println("Ethernet connected");
    break;
  case ARDUINO_EVENT_ETH_DISCONNECTED:
    Serial.println("Ethernet disconnected");
    break;
  case ARDUINO_EVENT_ETH_GOT_IP:
    Serial.println("Obtained IP address");
    Serial.println(WiFi.localIP());
    break;
  default:
    break;
  }
}

void dmxReceived()
{
  int led = 0;
  for (int i = 1; i < LED_COUNT * 3; i = i + 3)
  {
    strip.setPixelColor(led, strip.Color(recv.dmx(i), recv.dmx(i + 1), recv.dmx(i + 2)));
    led = led + 1;
  }
  strip.show();
}

void newSource()
{
  Serial.print("new soure name: ");
  Serial.println(recv.name());
}

void framerate()
{
  Serial.print("Framerate fps: ");
  Serial.println(recv.framerate());
}

void timeOut()
{
  Serial.println("Timeout!");
}

int i2c_scan(void)
{

  byte count = 0;

  Wire.begin();
  for (byte i = 1; i < 120; i++)
  {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0)
    {
      Serial.print(F("Found address: "));
      Serial.print(i, DEC);
      Serial.print(F(" (0x"));
      Serial.print(i, HEX);
      Serial.println(F(")"));
      count++;
    }         // end of good response
    delay(5); // give devices time to recover
  }           // end of for loop

  Serial.println(F("I2C scan done.."));
  Serial.print(F("Found "));
  Serial.print(count, DEC);
  Serial.println(F(" device(s)."));
  return count;
}

void wifiConnect()
{
  boolean wifiFound = false;
  int i, n;
  Serial.println("configuring wifi");
  // ----------------------------------------------------------------
  // Set WiFi to station mode and disconnect from an AP if it was previously connected
  // ----------------------------------------------------------------
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  Serial.println("Setup done");

  // ----------------------------------------------------------------
  // WiFi.scanNetworks will return the number of networks found
  // ----------------------------------------------------------------
  Serial.println(F("scan start"));
  int nbVisibleNetworks = WiFi.scanNetworks(true);
  while (WiFi.scanComplete() < 1)
  {
    digitalWrite(SYSLED, LOW);
    delay(250);
    digitalWrite(SYSLED, HIGH);
    delay(250);
    if (WiFi.scanComplete() == -2)
    {
      nbVisibleNetworks = WiFi.scanNetworks(true);
    }
  }
  Serial.println(F("scan done"));
  if (WiFi.scanComplete() == 0)
  {
    Serial.println(F("no networks found. Soon re-scan"));
    delay(5000);
    return;
  }

  // ----------------------------------------------------------------
  // if you arrive here at least some networks are visible
  // ----------------------------------------------------------------
  Serial.print(WiFi.scanComplete());
  Serial.println(" network(s) found");

  // ----------------------------------------------------------------
  // check if we recognize one by comparing the visible networks
  // one by one with our list of known networks
  // ----------------------------------------------------------------
  for (i = 0; i < WiFi.scanComplete(); ++i)
  {
    // Serial.println(WiFi.SSID(i)); // Print current SSID
    for (n = 0; n < KNOWN_SSID_COUNT; n++)
    { // walk through the list of known SSID and check for a match
      if (strcmp(KNOWN_SSID[n], WiFi.SSID(i).c_str()))
      {
        // Serial.print(F("\tNot matching "));
        // Serial.println(KNOWN_SSID[n]);
      }
      else
      { // we got a match
        wifiFound = true;
        break; // n is the network index we found
      }
    } // end for each known wifi SSID
    if (wifiFound)
      break; // break from the "for each visible network" loop
  }          // end for each visible network

  if (!wifiFound)
  {
    Serial.println(F("no Known network identified. Reset to try again"));
    while (true)
      ; // no need to go further, hang in there, will auto launch the Soft WDT reset
  }

  // ----------------------------------------------------------------
  // if you arrive here you found 1 known SSID
  // ----------------------------------------------------------------
  Serial.print(F("\nConnecting to "));
  Serial.println(KNOWN_SSID[n]);

  // ----------------------------------------------------------------
  // We try to connect to the WiFi network we found
  // ----------------------------------------------------------------
  WiFi.begin(KNOWN_SSID[n], KNOWN_PASSWORD[n]);

  int timeout = 150;
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(50);
    Serial.print(".");
    timeout--;
    digitalWrite(SYSLED, !digitalRead(SYSLED));
    if (timeout == 0)
    {
      Serial.println("Timed out");
      digitalWrite(SYSLED, LOW);
      return;
    }
  }
  Serial.println("");

  // ----------------------------------------------------------------
  // SUCCESS, you are connected to the known WiFi network
  // ----------------------------------------------------------------
  digitalWrite(SYSLED, HIGH);
  Serial.println(F("WiFi connected, your IP address is "));
  Serial.println(WiFi.localIP());
  delay(2000);
}

void adsReady()
{
  RDY = true;
}

void setup()
{
  delay(2000);
  Serial.begin(1000000);

  xTaskCreate(
      fn_task_led,
      "led_blink_task",
      1000,
      NULL,
      1,
      &task_led // Task handle
  );

  Serial.println();
  Serial.println("Hellooo");

  char *mychar;
  String s = WiFi.macAddress(); // Use Mac ID as a mychar
  int slen = s.length();
  mychar = &s[0];

  snprintf(clientID, 20, "%s", mychar);

  strip.begin();
  strip.fill(strip.Color(255, 0, 0), 0, LED_COUNT);
  strip.show();
  delay(250);
  strip.clear();
  strip.show();

  delay(500);

  Serial.println("I2C init");
  i2c_scan();

  EEPROM.begin(EEPROM_SIZE);
  delay(100);
  EEPROM.get(CALIBRATION_X_LOW_ADDRESS, x_min);
  delay(10);
  EEPROM.get(CALIBRATION_X_HIGH_ADDRESS, x_max);
  delay(10);
  EEPROM.get(CALIBRATION_Y_LOW_ADDRESS, y_min);
  delay(10);
  EEPROM.get(CALIBRATION_Y_HIGH_ADDRESS, y_max);
  delay(10);

  Serial.print("Reading initial values x_low:");
  Serial.print(x_min);
  Serial.print(" x_max:");
  Serial.print(x_max);
  Serial.print(" y_min:");
  Serial.print(y_min);
  Serial.print(" y_max:");
  Serial.println(y_max);

  if (x_min == -1)
    x_min = 65535;
  if (x_max == -1)
    x_max = 0;
  if (y_min == -1)
    y_min = 65535;
  if (y_max == -1)
    y_max = 0;

  /* Initialize wifi */
  WiFi.onEvent(WiFiEvent);
  wifiConnect();

  Serial.println("I2C init");
  i2c_scan();

  // Serial.println("MQTT hello");
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  Serial.println("Wire timeout set");
  Wire.setTimeOut(1000);

  sender.begin();

  // set DMX channel values to 0:
  for (int dmxChannel = 1; dmxChannel < 10; dmxChannel++)
  {
    sender.dmx(dmxChannel, 0);
    lastScnSend = 0;
  }
  recv.callbackDMX(dmxReceived);
  recv.callbackSource(newSource);
  recv.callbackFramerate(framerate);
  recv.callbackTimeout(timeOut);
  recv.begin();

  ArduinoOTA.begin();
  ArduinoOTA.onStart([]()
                     { 
                      if(client.connected())
                      {
                        char out[50];
                        char value[20];
                        sprintf(out, "joystick/%s/status", clientID);
                        client.publish(out, "OTAFLASH", true);
                           } 
                           pinMode(LED_SYSTEM, OUTPUT);
                           digitalWrite(LED_SYSTEM, HIGH); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                        {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    digitalWrite(LED_SYSTEM,!digitalRead(LED_SYSTEM)); });

  ADS.begin();
  ADS.setGain(1);     //  6.144 volt
  ADS.setDataRate(5); //  0 = slow   4 = medium   7 = fast

  //  SET ALERT RDY PIN
  ADS.setComparatorThresholdHigh(0x8000);
  ADS.setComparatorThresholdLow(0x0000);
  ADS.setComparatorQueConvert(0);
  //  SET INTERRUPT HANDLER TO CATCH CONVERSION READY
  pinMode(25, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(25), adsReady, RISING);

  ADS.setMode(0); //  continuous mode
  channel = 0;
  ADS.requestADC(channel); //  start at 0
}

void ringbuffer_push(uint8_t id, uint16_t value)
{
  ringbuffer[id][ringbuffer_pos[id]] = value;

  ringbuffer_pos[id] = ringbuffer_pos[id] + 1;
  if (ringbuffer_pos[id] >= SAMPLES)
  {
    ringbuffer_pos[id] = 0;
  }
}

long map_limited(long x, long in_min, long in_max, long out_min, long out_max)
{
  if (x < in_min)
    return out_min;
  else if (x > in_max)
    return out_max;
  const long run = in_max - in_min;
  if (run == 0)
  {
    log_e("map(): Invalid input range, min == max");
    return -1; // AVR returns -1, SAM returns 0
  }
  const long rise = out_max - out_min;
  const long delta = x - in_min;
  return (delta * rise) / run + out_min;
}

int get_ring_value(uint8_t id)
{
  unsigned long calc = 0;
  for (int i = 0; i < SAMPLES; i++)
  {
    calc = calc + ringbuffer[id][i];
  }

  calc = calc / SAMPLES;
  return calc;
}

uint8_t calculateAbsoluteDistance(uint8_t value1, uint8_t value2)
{
  if (value1 > value2)
  {
    return value1 - value2;
  }
  else
  {
    return value2 - value1;
  }
}

void loop()
{
  ArduinoOTA.handle();
  if (millis() - lastSysWorkerTick > sysWorkerTickInterval)
  {
    lastSysWorkerTick = millis();
    sender.send();
    if (client.connected())
    {
      if (last_reported_x_max != x_max)
      {
        char out[50];
        char value[20];
        sprintf(out, "joystick/%s/axis/x/high", clientID);
        sprintf(value, "%d", x_max);
        client.publish(out, value, true);

        last_reported_x_max = x_max;
      }
      else if (last_reported_x_min != x_min)
      {
        char out[50];
        char value[20];
        sprintf(out, "joystick/%s/axis/x/low", clientID);
        sprintf(value, "%d", x_min);
        client.publish(out, value, true);
        last_reported_x_min = x_min;
      }
      else if (last_reported_y_max != y_max)
      {
        char out[50];
        char value[20];
        sprintf(out, "joystick/%s/axis/y/high", clientID);
        sprintf(value, "%d", y_max);
        client.publish(out, value, true);
        last_reported_y_max = y_max;
      }
      else if (last_reported_y_min != y_min)
      {
        char out[50];
        char value[20];
        sprintf(out, "joystick/%s/axis/y/low", clientID);
        sprintf(value, "%d", y_min);
        client.publish(out, value, true);
        last_reported_y_min = y_min;
      }
    }
  }

  if (WiFi.isConnected() == false)
  {
    wifiConnect();
  }

  sender.idle();

  /*
  if (millis() - lastScnSend > scnTickInterval && scnForceSend)
   {
     lastScnSend = millis();
     sender.send();
   }*/
  if ((millis() - lastScnForcedOn > scnForcedOnLimit) && scnForceSend)
  {
    scnForceSend = false;
    char out[50];
    sprintf(out, "joystick/%s/mode", clientID);
    client.publish(out, "sleep", true);

    centerX = get_ring_value(3);
    centerY = get_ring_value(2);

    Serial.print("Center values x:");
    Serial.print(centerX);
    Serial.print(" y:");
    Serial.println(centerY);

    sender.dmx(1, 127);
    sender.dmx(2, 127);
  }
  recv.receive();

  if (RDY)
  {
    SPS++;
    val[channel] = ADS.getValue();
    //  request next channel asap

    ADS.requestADC(channel);
    RDY = false;
    if (calibrationEnable)
    {
      if (channel == 2)
      {
        if (val[2] < y_min)
        {
          y_min = val[2];
        }
        if (val[2] > y_max)
        {
          y_max = val[2];
        }
      }
      if (channel == 3)
      {
        if (val[3] < x_min)
        {
          x_min = val[3];
        }
        if (val[3] > x_max)
        {
          x_max = val[3];
        }
      }
    }

    for (int i = 0; i < 4; i++)
    {
      ringbuffer_push(i, val[i]);
    }

    static uint8_t last_x = 255;
    static uint8_t last_y = 255;

    uint8_t x = 0;
    uint8_t y = 0;

    uint16_t ring_x = get_ring_value(3);
    uint16_t ring_y = get_ring_value(2);

    if (ring_x < centerX)
    {
      x = map_limited(ring_x, x_min, (centerX - centerX*JOYSTICK_ERROR_PERCENT/100), 0, 127);
    }
    else
    {
      x = map_limited(ring_x, (centerX + centerX*JOYSTICK_ERROR_PERCENT/100), x_max, 127, 255);
    }

    if (ring_y < centerY)
    {
      y = map_limited(ring_y, y_min, (centerY - centerY*JOYSTICK_ERROR_PERCENT/100), 0, 127);
    }
    else
    {
      y = map_limited(ring_y, (centerY + centerY*JOYSTICK_ERROR_PERCENT/100), y_max, 127, 255);
    }

    if (ringbuffer_pos[1] == 0)
    {

      if (scnForceSend)
      {
        sender.send();
        sender.dmx(1, y);
        sender.dmx(2, x);
      }
    }

    if (calculateAbsoluteDistance(last_x, x) > JOYSTICK_WAKE_THRESHOLD)
    {
      last_x = x;
      lastScnSend = 0;
      lastScnForcedOn = millis();
      if (scnForceSend == false)
      {
        char out[50];
        sprintf(out, "joystick/%s/mode", clientID);
        client.publish(out, "active", true);
      }
      scnForceSend = true;
    }
    else if (calculateAbsoluteDistance(last_y, y) > JOYSTICK_WAKE_THRESHOLD)
    {
      last_y = y;
      lastScnSend = 0;
      lastScnForcedOn = millis();
      if (scnForceSend == false)
      {
        char out[50];
        sprintf(out, "joystick/%s/mode", clientID);
        client.publish(out, "active", true);
      }
      scnForceSend = true;
    }

    channel++;
    if (channel > 3)
      channel = 0;
  }

  //  print the SPS
  if (millis() - lastTime >= 3500)
  {
    lastTime = millis();

    char out[50];
    sprintf(out, "joystick/%s/status", clientID);
    client.publish(out, "online");

    char value[20];
    sprintf(out, "merleg/status/uptime");
    sprintf(value, "%l", millis());
    client.publish(out, value, true);

    sprintf(out, "joystick/%s/status/rssi", clientID);
    sprintf(value, "%d", WiFi.RSSI());
    client.publish(out, value, true);
    sprintf(out, "joystick/%s/status/bssid", clientID);
    sprintf(value, "%s", WiFi.BSSIDstr().c_str());
    client.publish(out, value, true);
  }

  if (!client.connected())
  {
    long now = millis();
    if (now - lastReconnectAttempt > 5000)
    {
      lastReconnectAttempt = now;
      Serial.println("MQTT reconnecting");
      // Attempt to reconnect
      if (reconnect())
      {
        lastReconnectAttempt = 0;
      }
    }
  }
  else
  {
    // Client connected
    client.loop();
  }
}
