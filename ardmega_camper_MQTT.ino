#include <dhtnew.h>
#include <SPI.h>
#include <WiFi.h>
#include "arduino_secrets.h" 
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = WSSID;        // your network SSID (name)
char pass[] = WPSWD;    // your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS;
#include <PubSubClient.h>
#include <Adafruit_MCP23X17.h>
#include <Adafruit_ADS1X15.h>

#define RELAY_HEAT 11
#define RELAY_FAN 12
#define RELAY_HVAC_HEAT 13
#define RELAY_HVAC_FAN 14
#define RELAY_HVAC_COOL 15

#define RELAY_ON HIGH
#define RELAY_OFF LOW

DHTNEW mySensor(2);
DHTNEW roomsensor(3);

Adafruit_MCP23X17 mcp;
Adafruit_MCP23X17 mcpB;

Adafruit_ADS1115 ads1115;	// Construct an ads1115 
//ads1115.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV (default)
// ads1115.setGain(GAIN_ONE);     // 1x gain   +/- 4.096V  1 bit = 2mV
// ads1115.setGain(GAIN_TWO);     // 2x gain   +/- 2.048V  1 bit = 1mV
// ads1115.setGain(GAIN_FOUR);    // 4x gain   +/- 1.024V  1 bit = 0.5mV
// ads1115.setGain(GAIN_EIGHT);   // 8x gain   +/- 0.512V  1 bit = 0.25mV
// ads1115.setGain(GAIN_SIXTEEN); // 16x gain  +/- 0.256V  1 bit = 0.125mV

// Update these with values suitable for your hardware/network.
byte mac[]    = {  0xDE, 0xED, 0xBA, 0xFE, 0xFE, 0xBE };
IPAddress server(192, 168, 1, 100);
IPAddress ip(192, 168, 0, 29);
IPAddress myDns(192, 168, 0, 1);

WiFiClient wClient;
PubSubClient client(wClient);
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



bool state[11] = {false,false,false,false,false,false,false,false,false,false,false};
bool stateAct[11] = {false,false,false,false,false,false,false,false,false,false,false};
String setVal[11] = {"OFF","OFF","OFF","OFF","OFF","OFF","OFF","OFF","OFF","OFF","OFF"};
int relayPins[11] = {0,1,2,3,4,5,6,7,8,9,10};
int switchPins[11] = {0,1,2,3,4,5,6,7,8,9,10};
String pubStr[11] = {"shop/switch/light","shop/switch/dust","shop/switch/compressor","shop/switch/attic","","","","","","",""};
String subStr[11] = {"shop/switch/light/set","shop/switch/dust/set","shop/switch/compressor/set","shop/switch/attic/set","","","","","","",""};

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
    for (int i=0; i<=10; i++) {
      if (subStr[i] != "") {
        subStr[i].toCharArray(tmpChar, 40);
        client.subscribe(tmpChar);
      }
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
    for (int i=0; i<=10; i++) {
      if (tmpTopic == subStr[i] && subStr[i] != "") setVal[i] = tmpStr;
    }
  }
}

void setup()
{
  mcp.begin_I2C(0x27);
  mcpB.begin_I2C(0x20);
  //ads1115.begin(0x49);  // Initialize ads1115 at address 0x49
  client.setServer(server, 1883);
  client.setCallback(callback);
  if (WiFi.status() == WL_NO_SHIELD) {
    while (true);
  }
  while (status != WL_CONNECTED) {
    status = WiFi.begin(ssid, pass);
    delay(10000);
  }

  delay(1500);
  lastReconnectAttempt = 0;

  for (int i = 0; i <= 15; i++)
  {
    mcp.pinMode(i, OUTPUT);
    mcp.digitalWrite(i,RELAY_OFF);
  }
  for (int i = 0; i <= 10; i++)
  {
    mcpB.pinMode(switchPins[i], INPUT);
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

  if (millis() - lastTimer >= 2000)
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
    for (int i=0; i<=10; i++) {
      if (pubStr[i] != "") {
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
          mcp.digitalWrite(relayPins[i],RELAY_ON);
        }
        else
        {
          mcp.digitalWrite(relayPins[i],RELAY_OFF);
        }
        stateAct[i] = mcp.digitalRead(switchPins[i]);
      }
    }
    char tmpChar[40];
    for (int i=0; i<=10; i++) {
      if (pubStr[i] != "") {
        pubStr[i].toCharArray(tmpChar, 40);
        client.publish(tmpChar,stateAct[i]? "ON":"OFF");
      }
    }
    
    // handle voltage sense for Equipment
    // 20k/1k vd
    // ((Vin * Rlow) / (Rhi + Rlow)) = Vout
    //

    //float operatingVoltage = ads1115.readADC_SingleEnded(0); // analog A0 tied to 3.3v voltage reference. could use a better reference than the 3.3v
    float operatingVoltage = analogRead(0); // analog A0 tied to 3.3v voltage reference. could use a better reference than the 3.3v
    //float rawVoltage = ads1115.readADC_SingleEnded(1); // A1 tied to voltage sense through the 20K/1K voltage divider. 100Vdc max input!
    float rawVoltage = analogRead(1); // A1 tied to voltage sense through the 20K/1K voltage divider. 100Vdc max input!
    operatingVoltage = 2.048 / operatingVoltage; // get multiplyer, if better reference, change 3.30 to actual reference volts.
    rawVoltage = operatingVoltage * rawVoltage; // calc raw voltage from 3.3v reference.
    float equipCalcVolts = ((rawVoltage * 1000) / (20000 + 1000));

    char tmpV[32];
    sprintf(tmpV, "%6.2f", equipCalcVolts);
    client.publish("equip/volts",tmpV);
    
    // handle load current sense with a Tamura L01Z200S05
    //float BrawVoltage = ads1115.readADC_SingleEnded(2); // Tied to Load current sensor output.
    float BrawVoltage = analogRead(2); // Tied to Load current sensor output.
    BrawVoltage = operatingVoltage * BrawVoltage; // calc raw voltage from 3.3v reference.
    float ampsA = map(BrawVoltage,0.00,3.30,-200.00,200.00);
    char tmpAmps[32];
    sprintf(tmpAmps, "%6.2f", ampsA);
    client.publish("equip/loadamps",tmpAmps);
    char tmpWatts[32];
    sprintf(tmpWatts, "%6.2f", (equipCalcVolts * ampsA));
    client.publish("equip/loadwatts",tmpWatts);

    //float CrawVoltage = ads1115.readADC_SingleEnded(3); // Tied to Battery current sensor output.
    float CrawVoltage = analogRead(3); // Tied to Battery current sensor output.
    CrawVoltage = operatingVoltage * CrawVoltage; // calc raw voltage from 3.3v reference.
    float ampsB = map(CrawVoltage,0.00,3.30,-200.00,200.00);
    char tmpAmpsB[32];
    sprintf(tmpAmpsB, "%6.2f", ampsB);
    client.publish("equip/battamps",tmpAmpsB);
    char tmpWattsB[32];
    sprintf(tmpWattsB, "%6.2f", (equipCalcVolts * ampsB));
    client.publish("equip/battwatts",tmpWattsB);
    
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
      mcp.digitalWrite(RELAY_HEAT, RELAY_ON);
    }
    else
    {
      mcp.digitalWrite(RELAY_HEAT, RELAY_OFF);
    }
    if (stateEquipCooling == true && stateEquipCool == false)
    {
      mcp.digitalWrite(RELAY_FAN, RELAY_ON);
      stateEquipCool = true;
    }
    else if(stateEquipCooling == false)
    {
      mcp.digitalWrite(RELAY_FAN, RELAY_OFF);
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
    sprintf(sz, "%6.2f", tempEquipF);
    client.publish("equip/hvac/temperature/current",sz);
    sprintf(sz, "%6.2f", humidityEquipRH);
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
    mcp.digitalWrite(RELAY_HVAC_HEAT, RELAY_ON);
  }
  else
  {
    mcp.digitalWrite(RELAY_HVAC_HEAT, RELAY_OFF);
  }
  if (stateCooling == true && stateFan == false && stateCool == false)
  {
    mcp.digitalWrite(RELAY_HVAC_FAN, RELAY_ON);
    stateFan = true;
  }
  else if (stateCooling == true && millis() - coolTimer >= 30000 && stateFan == true && stateCool == false)
  {
    stateCool = true;
    coolTimer = millis();
    mcp.digitalWrite(RELAY_HVAC_COOL, RELAY_OFF);
  }
  else if(stateCooling == false)
  {
    mcp.digitalWrite(RELAY_HVAC_COOL, RELAY_OFF);
    mcp.digitalWrite(RELAY_HVAC_FAN, RELAY_OFF);
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
  sprintf(sz, "%6.2f", tempF);
  client.publish("shop/hvac/temperature/current",sz);
  sprintf(sz, "%6.2f", humidityRH);
  client.publish("shop/hvac/humidity/current",sz);
}
