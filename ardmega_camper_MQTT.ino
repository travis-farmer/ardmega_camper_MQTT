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
#define RELAY_COOL 48
#define RELAY_FAN 47


#define ZONE_COUNT 2
#define RELAY_ON LOW
#define RELAY_OFF HIGH

DHTNEW mySensor(2);
Adafruit_MCP23X17 mcp;

// Update these with values suitable for your hardware/network.
byte mac[]    = {  0xDE, 0xED, 0xBA, 0xFE, 0xFE, 0xBE };
IPAddress server(192, 168, 1, 84);
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
String setMode = "";
float tempF = 0.00;
float humidityRH = 0.00;
float thermostatHysteresis = 3.00;
unsigned long lastTimer = 0UL;
bool stateHeating = false;
bool stateCooling = false;
bool stateFan = false;
bool stateCool = false;



bool state[8] = {false,false,false,false,false,false,false,false};
bool stateAct[8] = {false,false,false,false,false,false,false,false};
String setVal[8] = {"","","","","","","",""};
int relayPins[8] = {44,43,42,41,40,39,38,37};
int switchPins[8] = {24,25,26,27,28,29,30,31};
String pubStr[8] = {"shop/switch/light","shop/switch/dust","shop/switch/compressor","","","","",""};
String subStr[8] = {"shop/switch/light/set","shop/switch/dust/set","shop/switch/compressor/set","","","","",""};

int buttonState[8];
int lastButtonState[8] = {HIGH,HIGH,HIGH,HIGH,HIGH,HIGH,HIGH,HIGH};
unsigned long lastDebounceTime[8] = {0UL,0UL,0UL,0UL,0UL,0UL,0UL,0UL};

unsigned long coolTimer = 0UL;

boolean reconnect()
{
  if (client.connect("ArduinoShop", MQTT_USER, MQTT_PASS))
  {
    client.subscribe("equip/hvac/mode/set");
    client.subscribe("equip/hvac/temperature/lowset");
    client.subscribe("equip/hvac/temperature/highset");

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

  if (tmpTopic == "equip/hvac/mode/set") setMode = tmpStr;
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

  for (int i = RELAY_HEAT; i <= RELAY_FAN; i++)
  {
    pinMode(i, OUTPUT);
    digitalWrite(i,RELAY_OFF);
  }
  for (int i = 0; i <= ZONE_COUNT; i++)
  {
    pinMode(relayPins[i], OUTPUT);
    digitalWrite(relayPins[i],RELAY_OFF);
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

  if (millis() - lastTimer >= 500)
  {
    /** \brief setup current values
      *
      *
      */
    lastTimer = millis();
    int chk = mySensor.read();
    humidityRH = mySensor.getHumidity();
    tempF = ((mySensor.getTemperature() * 1.80) + 32.00);


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
      }
      else
      {
        digitalWrite(relayPins[i],RELAY_OFF);
      }
      stateAct[i] = digitalRead(switchPins[i]);
    }
    char tmpChar[40];
    for (int i=0; i<=ZONE_COUNT; i++) {
      pubStr[i].toCharArray(tmpChar, 40);
      client.publish(tmpChar,stateAct[i]? "ON":"OFF");
    }
    
    // handle voltage sense for dump load
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
    if (setMode == "off")
    {
      stateHeating = false;
      stateCooling = false;
    }
    else if (setMode == "heat")
    {
      stateCooling = false;
      if (tempF >= setLowTemp)
      {
        stateHeating = false;
      }
      else if (tempF <= setLowTemp - thermostatHysteresis)
      {
        stateHeating = true;
      }
    }
    else if (setMode == "cool")
    {
      stateHeating = false;
      if (tempF <= setHighTemp)
      {
        stateCooling = false;
      }
      else if (tempF >= setHighTemp + thermostatHysteresis)
      {
        stateCooling = true;
      }
    }
    else if (setMode == "auto")
    {
      if (tempF >= setLowTemp + thermostatHysteresis && stateCooling == false)
      {
        stateCooling = true;
        stateHeating = false;
      }
      else if (tempF <= setHighTemp - thermostatHysteresis && stateHeating == false)
      {
        stateCooling = false;
        stateHeating = true;
      }
      else if ((tempF >= setLowTemp && stateHeating == true) || (tempF <= setHighTemp && stateCooling == true))
      {
        stateCooling = false;
        stateHeating = false;
      }

    }
    if (stateHeating == true)
    {
      mcp.digitalWrite(RELAY_HEAT, LOW);
    }
    else
    {
      mcp.digitalWrite(RELAY_HEAT, HIGH);
    }
    if (stateCooling == true && stateFan == false && stateCool == false)
    {
      mcp.digitalWrite(RELAY_FAN, LOW);
      stateFan = true;
    }
    else if (stateCooling == true && millis() - coolTimer >= 30000 && stateFan == true && stateCool == false)
    {
      stateCool = true;
      coolTimer = millis();
      mcp.digitalWrite(RELAY_COOL, LOW);
    }
    else if(stateCooling == false)
    {
      mcp.digitalWrite(RELAY_COOL, HIGH);
      mcp.digitalWrite(RELAY_FAN, HIGH);
      stateFan = false;
      stateCool = false;
    }

    /** \brief setup and send values and states
      *
      *
      */
    char sz[32];
    String strAction = "";
    if (stateHeating == true)
    {
      strAction = "heating";
    }
    else if (stateCooling == true)
    {
      strAction = "cooling";
    }
    else
    {
      strAction = "off";
    }
    strAction.toCharArray(sz, 32);
    client.publish("equip/hvac/action",sz);
    dtostrf(tempF, 4, 2, sz);
    client.publish("equip/hvac/temperature/current",sz);
    dtostrf(humidityRH, 4, 2, sz);
    client.publish("equip/hvac/humidity/current",sz);
  }
}
