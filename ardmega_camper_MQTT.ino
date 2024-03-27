//#define ETH
#define WIFI
#include <dhtnew.h>
#include <SPI.h>
#if defined(ETH)
#include <Ethernet.h>
#elif defined(WIFI)
#include <WiFi101.h>
#include "arduino_secrets.h" 
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = WSSID;        // your network SSID (name)
char pass[] = WPSWD;    // your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS;
#endif
#include <PubSubClient.h>
#include <Adafruit_MCP23X17.h>

#define RELAY_HEAT 49
#define RELAY_FAN 48
#define RELAY_HVAC_HEAT 47
#define RELAY_HVAC_FAN 46
#define RELAY_HVAC_COOL 45

#define ZONE_COUNT 3
#define RELAY_ON LOW
#define RELAY_OFF HIGH

DHTNEW mySensor(2);
DHTNEW roomsensor(3);

Adafruit_MCP23X17 mcp;

// Update these with values suitable for your hardware/network.
byte mac[]    = {  0xDE, 0xED, 0xBA, 0xFE, 0xFE, 0xBE };
IPAddress server(192, 168, 1, 100);
IPAddress ip(192, 168, 0, 29);
IPAddress myDns(192, 168, 0, 1);

#if defined(ETH)
EthernetClient eClient;
PubSubClient client(eClient);
#elif defined(WIFI)
WiFiClient wClient;
PubSubClient client(wClient);
#endif
long lastReconnectAttempt = 0;

int setLowTemp = 0;
int setHighTemp = 0;
int setTemp = 0;
String setEquipMode = "";
String setMode = "";
float tempEquipF = 0.00;
float tempF = 0.00;
float humidityEquipRH = 0.00;
float humidityRH = 0.00;
float thermostatHysteresis = 3.00;
unsigned long lastTimer = 0UL;
bool stateEquipHeating = false;
bool stateEquipCooling = false;
bool stateEquipCool = false;
bool stateHeating = false;
bool stateCooling = false;
bool stateFan = false;
bool stateCool = false;



bool state[4] = {false,false,false,false};
bool stateAct[4] = {false,false,false,false};
String setVal[4] = {"","","",""};
int relayPins[4] = {41,42,43,44};
int relayPinsB[4] = {37,38,39,40};
int switchPins[4] = {33,34,35,36};
String pubStr[4] = {"shop/switch/light","shop/switch/dust","shop/switch/compressor","shop/switch/attic"};
String subStr[4] = {"shop/switch/light/set","shop/switch/dust/set","shop/switch/compressor/set","shop/switch/attic/set"};

int buttonState[4];
int lastButtonState[4] = {HIGH,HIGH,HIGH,HIGH};
unsigned long lastDebounceTime[4] = {0UL,0UL,0UL,0UL};

unsigned long coolTimer = 0UL;

boolean reconnect()
{
  if (client.connect("ArduinoShopMQTT"))
  {
    client.subscribe("equip/hvac/mode/set");
    client.subscribe("equip/hvac/temperature/lowset");
    client.subscribe("equip/hvac/temperature/highset");
    client.subscribe("shop/hvac/mode/set");
    client.subscribe("shop/hvac/temperature/set");

    char tmpChar[40];
    for (int i=0; i<=ZONE_COUNT; i++) {
      subStr[i].toCharArray(tmpChar, 40);
      client.subscribe(tmpChar);
    }
  }
  return client.connected();
}

void callback(char* topic, byte* payload, unsigned int length)
{
  String tmpTopic = topic;
  char tmpStr[length+1];
  for (int x=0; x<length; x++)
  {
      tmpStr[x] = (char)payload[x]; // payload is a stream, so we need to chop it by length.
  }
  tmpStr[length] = 0x00; // terminate the char string with a null

  if (tmpTopic == "shop/hvac/mode/set") setMode = tmpStr;
  else if (tmpTopic == "shop/hvac/temperature/lowset") setTemp = atoi(tmpStr);
  else if (tmpTopic == "equip/hvac/mode/set") setEquipMode = tmpStr;
  else if (tmpTopic == "equip/hvac/temperature/lowset") setLowTemp = atoi(tmpStr);
  else if (tmpTopic == "equip/hvac/temperature/highset") setHighTemp = atoi(tmpStr);
  else {
    for (int i=0; i<=ZONE_COUNT; i++) {
      if (tmpTopic == subStr[i]) setVal[i] = tmpStr;
    }
  }
}

void setup()
{
  mcp.begin_I2C();
  client.setServer(server, 1883);
  client.setCallback(callback);
#if defined(ETH)
  if (Ethernet.begin(mac) == 0)
  {
    if (Ethernet.hardwareStatus() == EthernetNoHardware)
    {
    }
    if (Ethernet.linkStatus() == LinkOFF)
    {
    }
    // try to congifure using IP address instead of DHCP:
    Ethernet.begin(mac, ip, myDns);
  }
  else
  {
    //Serial.print("  DHCP assigned IP ");
    //Serial.println(Ethernet.localIP());
  }
#elif defined(WIFI)
  if (WiFi.status() == WL_NO_SHIELD) {
    while (true);
  }
  while (status != WL_CONNECTED) {
    status = WiFi.begin(ssid, pass);
    delay(10000);
  }
#endif
  delay(1500);
  lastReconnectAttempt = 0;

  for (int i = RELAY_HEAT; i <= RELAY_HVAC_COOL; i++)
  {
    pinMode(i, OUTPUT);
    digitalWrite(i,RELAY_OFF);
  }
  for (int i = 0; i <= ZONE_COUNT; i++)
  {
    pinMode(relayPins[i], OUTPUT);
    digitalWrite(relayPins[i],RELAY_OFF);
    pinMode(relayPinsB[i],OUTPUT);
    digitalWrite(relayPinsB[i],RELAY_ON);
  }
  for (int i = 0; i <= ZONE_COUNT; i++)
  {
    pinMode(switchPins[i], INPUT);
  }
  
}

void loop()
{
    if (!client.connected())
    {
      long now = millis();
      if (now - lastReconnectAttempt > 5000)
      {
        lastReconnectAttempt = now;
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

  if (millis() - lastTimer >= 1000)
  {
    /** \brief setup current values
      *
      *
      */
    lastTimer = millis();

    int chk = mySensor.read();
    humidityEquipRH = mySensor.getHumidity();
    tempEquipF = ((mySensor.getTemperature() * 1.80) + 32.00);
    int chkB = roomsensor.read();
    humidityRH = roomsensor.getHumidity();
    tempF = ((roomsensor.getTemperature() * 1.80) + 32.00);

    /** \brief handle switches
      *
      *
      */
    for (int i=0; i<=ZONE_COUNT; i++) {
      if (setVal[i] == "ON")
      {
        state[i] = true;
      }
      else if (setVal[i] == "OFF")
      {
        state[i] = false;
      }
      
      if (state[i] == true)
      {
        digitalWrite(relayPins[i],RELAY_ON);
        digitalWrite(relayPinsB[i],RELAY_OFF);
      }
      else
      {
        digitalWrite(relayPins[i],RELAY_OFF);
        digitalWrite(relayPinsB[i],RELAY_ON);
      }
      stateAct[i] = digitalRead(switchPins[i]);
    }
    char tmpChar[40];
    for (int i=0; i<=ZONE_COUNT; i++) {
      pubStr[i].toCharArray(tmpChar, 40);
      client.publish(tmpChar,stateAct[i]? "ON":"OFF");
    }
    
    // handle voltage sense for Equipment
    // 20k/1k vd
    // ((Vin * Rlow) / (Rhi + Rlow)) = Vout
    //

    float operatingVoltage = analogRead(0); // analog A0 tied to 3.3v voltage reference. could use a better reference than the 3.3v
    float rawVoltage = analogRead(1); // A1 tied to voltage sense through the 20K/1K voltage divider. 100Vdc max input!
    operatingVoltage = 3.30 / operatingVoltage; // get multiplyer, if better reference, change 3.30 to actual reference volts.
    rawVoltage = operatingVoltage * rawVoltage; // calc raw voltage from 3.3v reference.
    float equipCalcVolts = ((rawVoltage * 1000) / (20000 + 1000));

    char tmpV[32];
    dtostrf(equipCalcVolts, 7, 2, tmpV);
    client.publish("equip/volts",tmpV);
    
    // handle load current sense with a Tamura L01Z200S05
    float BrawVoltage = analogRead(5); // Tied to Load current sensor output.
    BrawVoltage = operatingVoltage * BrawVoltage; // calc raw voltage from 3.3v reference.
    float amps = map(BrawVoltage,0.00,5.00,-200.00,200.00);
    char tmpAmps[32];
    dtostrf(amps, 7, 3, tmpAmps);
    client.publish("equip/amps",tmpAmps);

    /** \brief handle Equipment thermostat
      *
      *
      */
    if (setEquipMode == "off")
    {
      stateEquipHeating = false;
      stateEquipCooling = false;
    }
    else if (setEquipMode == "heat")
    {
      stateEquipCooling = false;
      if (tempEquipF >= setLowTemp)
      {
        stateEquipHeating = false;
      }
      else if (tempEquipF <= setLowTemp - thermostatHysteresis)
      {
        stateEquipHeating = true;
      }
    }
    else if (setEquipMode == "cool")
    {
      stateEquipHeating = false;
      if (tempEquipF <= setHighTemp)
      {
        stateEquipCooling = false;
      }
      else if (tempEquipF >= setHighTemp + thermostatHysteresis)
      {
        stateEquipCooling = true;
      }
    }
    else if (setEquipMode == "auto")
    {
      if (tempEquipF >= setLowTemp + thermostatHysteresis && stateEquipCooling == false)
      {
        stateEquipCooling = true;
        stateEquipHeating = false;
      }
      else if (tempEquipF <= setHighTemp - thermostatHysteresis && stateEquipHeating == false)
      {
        stateEquipCooling = false;
        stateEquipHeating = true;
      }
      else if ((tempEquipF >= setLowTemp && stateEquipHeating == true) || (tempEquipF <= setHighTemp && stateEquipCooling == true))
      {
        stateEquipCooling = false;
        stateEquipHeating = false;
      }

    }
    if (stateEquipHeating == true)
    {
      digitalWrite(RELAY_HEAT, LOW);
    }
    else
    {
      digitalWrite(RELAY_HEAT, HIGH);
    }
    if (stateEquipCooling == true && stateEquipCool == false)
    {
      digitalWrite(RELAY_FAN, LOW);
      stateEquipCool = true;
    }
    else if(stateEquipCooling == false)
    {
      digitalWrite(RELAY_FAN, HIGH);
      stateEquipCool = false;
    }

    /** \brief setup and send values and states
      *
      *
      */
    char sz[32];
    String strAction = "";
    if (stateEquipHeating == true)
    {
      strAction = "heating";
    }
    else if (stateEquipCooling == true)
    {
      strAction = "cooling";
    }
    else
    {
      strAction = "off";
    }
    strAction.toCharArray(sz, 32);
    client.publish("equip/hvac/action",sz);
    dtostrf(tempEquipF, 4, 2, sz);
    client.publish("equip/hvac/temperature/current",sz);
    dtostrf(humidityEquipRH, 4, 2, sz);
    client.publish("equip/hvac/humidity/current",sz);
  }

/** \brief handle HVAC thermostat
    *
    *
    */
  if (setMode == "off")
  {
    stateHeating = false;
    stateCooling = false;
  }
  else if (setMode == "heat")
  {
    stateCooling = false;
    if (tempF >= setTemp)
    {
      stateHeating = false;
    }
    else if (tempF <= setTemp - thermostatHysteresis)
    {
      stateHeating = true;
    }
  }
  else if (setMode == "cool")
  {
    stateHeating = false;
    if (tempF <= setTemp)
    {
      stateCooling = false;
    }
    else if (tempF >= setTemp + thermostatHysteresis)
    {
      stateCooling = true;
    }
  }
  else if (setMode == "auto")
  {
    if (tempF >= setTemp + thermostatHysteresis && stateCooling == false)
    {
      stateCooling = true;
      stateHeating = false;
    }
    else if (tempF <= setTemp - thermostatHysteresis && stateHeating == false)
    {
      stateCooling = false;
      stateHeating = true;
    }
    else if ((tempF >= setTemp && stateHeating == true) || (tempF <= setTemp && stateCooling == true))
    {
      stateCooling = false;
      stateHeating = false;
    }

  }
  if (stateHeating == true)
  {
    digitalWrite(RELAY_HVAC_HEAT, LOW);
  }
  else
  {
    digitalWrite(RELAY_HVAC_HEAT, HIGH);
  }
  if (stateCooling == true && stateFan == false && stateCool == false)
  {
    digitalWrite(RELAY_HVAC_FAN, LOW);
    stateFan = true;
  }
  else if (stateCooling == true && millis() - coolTimer >= 30000 && stateFan == true && stateCool == false)
  {
    stateCool = true;
    coolTimer = millis();
    digitalWrite(RELAY_HVAC_COOL, LOW);
  }
  else if(stateCooling == false)
  {
    digitalWrite(RELAY_HVAC_COOL, HIGH);
    digitalWrite(RELAY_HVAC_FAN, HIGH);
    stateFan = false;
    stateCool = false;
  }

  /** \brief setup and send values and states
    *
    *
    */
  char sz[32];
  String strActionB = "";
  if (stateHeating == true)
  {
    strActionB = "heating";
  }
  else if (stateCooling == true)
  {
    strActionB = "cooling";
  }
  else
  {
    strActionB = "off";
  }
  strActionB.toCharArray(sz, 32);
  client.publish("shop/hvac/action",sz);
  dtostrf(tempF, 4, 2, sz);
  client.publish("shop/hvac/temperature/current",sz);
  dtostrf(humidityRH, 4, 2, sz);
  client.publish("shop/hvac/humidity/current",sz);
}
