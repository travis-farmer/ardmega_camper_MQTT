#include <dhtnew.h>
#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <Adafruit_ADS1X15.h>

#define RELAY_HEAT 30
#define RELAY_COOL 31
#define RELAY_FAN 32
#define RELAY_WCHARGE 33
#define RELAY_WDUMP 34

#define DINETTE 0
#define KITCHEN 1
#define BATH 2
#define BED 3
#define PORCH 4
#define UTILITY 5
#define ZONE_COUNT 5
#define RELAY_ON LOW
#define RELAY_OFF HIGH

DHTNEW mySensor(48);
Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
// ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
// ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
// ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
// ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
// ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
// ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

// Update these with values suitable for your hardware/network.
byte mac[]    = {  0xDE, 0xED, 0xBA, 0xFE, 0xFE, 0xBE };
IPAddress server(192, 168, 1, 84);
IPAddress ip(192, 168, 0, 29);
IPAddress myDns(192, 168, 0, 1);

EthernetClient eClient;
PubSubClient client(eClient);

long lastReconnectAttempt = 0;

int setTemp = 0;
String setMode = "";
float tempF = 0.00;
float humidityRH = 0.00;
float thermostatHysteresis = 3.00;
unsigned long lastTimer = 0UL;
bool stateHeating = false;
bool stateCooling = false;
bool stateFan = false;
bool stateCool = false;



bool state[6] = {false,false,false,false,false,false};
bool stateSW[6] = {false,false,false,false,false,false};
bool stateAct[6] = {false,false,false,false,false,false};
String setVal[6] = {"","","","","",""};
int relayPins[6] = {35,36,37,38,39,40};
int switchPins[6] = {24,25,26,27,28,29};
String pubStr[6] = {"camper/switch/dinette","camper/switch/kitchen","camper/switch/bath","camper/switch/bed","camper/switch/porch","camper/switch/utility"};
String subStr[6] = {"camper/switch/dinette/set","camper/switch/kitchen/set","camper/switch/bath/set","camper/switch/bed/set","camper/switch/porch/set","camper/switch/utility/set"};
String setDump = "";
bool stateDump = false;
bool stateCmdDump = false;
bool stateActDump = false;

unsigned long coolTimer = 0UL;


boolean reconnect()
{
  if (client.connect("ArduinoCamper"))
  {
    client.subscribe("hvac/mode/set");
    client.subscribe("hvac/temperature/set");
    client.subscribe("camper/switch/wdump/set");
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

  if (tmpTopic == "hvac/mode/set") setMode = tmpStr;
  else if (tmpTopic == "hvac/temperature/set") setTemp = atoi(tmpStr);
  else if (tmpTopic == "camper/switch/wdump/set") setDump = tmpStr;
  else {
    for (int i=0; i<=ZONE_COUNT; i++) {
      if (tmpTopic == subStr[i]) setVal[i] = tmpStr;
    }
  }
}

void setup()
{
  ads.begin();
  client.setServer(server, 1883);
  client.setCallback(callback);

//Serial.println("Initialize Ethernet with DHCP:");
  if (Ethernet.begin(mac) == 0)
  {
    //Serial.println("Failed to configure Ethernet using DHCP");
    // Check for Ethernet hardware present
    if (Ethernet.hardwareStatus() == EthernetNoHardware)
    {
        //Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
        //errorProc(1);
    }
    if (Ethernet.linkStatus() == LinkOFF)
    {
        //Serial.println("Ethernet cable is not connected.");
        //errorProc(2);
    }
    // try to congifure using IP address instead of DHCP:
    Ethernet.begin(mac, ip, myDns);
  }
  else
  {
    //Serial.print("  DHCP assigned IP ");
    //Serial.println(Ethernet.localIP());
  }
  delay(1500);
  lastReconnectAttempt = 0;

  /*for (int i=30; i<= 46; i++) {
    pinMode(i, OUTPUT);
    digitalWrite(i, HIGH); // Relays are active with a LOW signal
  }*/

  for (int i = RELAY_HEAT; i <= RELAY_WDUMP; i++)
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
    pinMode(switchPins[i], INPUT_PULLUP);
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
      if (digitalRead(switchPins[i]) == HIGH)
      {
        stateSW[i] = true;
      }
      else
      {
        stateSW[i] = false;
      }
      if (state[i] == stateSW[i])
      {
        digitalWrite(relayPins[i],RELAY_ON);
        stateAct[i] = true;
      }
      else
      {
        digitalWrite(relayPins[i],RELAY_OFF);
        stateAct[i] = false;
      }
    }
      
    float multiplier = 0.1875F; /* ADS1115  @ +/- 6.144V gain (16-bit results) */
    int16_t results = ads.readADC_Differential_0_1();
    float windMiliVolts = (results * multiplier);
    if (windMiliVolts >= 35000 && stateCmdDump == false) stateCmdDump = true;
    else if (windMiliVolts <= 14000 && stateCmdDump == true) stateCmdDump = false;
    char tmpMV[32];
    dtostrf((windMiliVolts/1000.00), 7, 2, tmpMV);
    client.publish("camper/windmv",tmpMV);

    // Tamura L01Z200S05
    int16_t refReading = ads.readADC_SingleEnded(2);
    float refVolts = ads.computeVolts(refReading);
    int16_t curReading = ads.readADC_SingleEnded(3);
    float curVolts = ads.computeVolts(curReading);
    float amps = map(curVolts,0.00,refVolts,-200.00,200.00);
    char tmpAmps[32];
    dtostrf(amps, 7, 2, tmpAmps);
    client.publish("camper/loada",tmpAmps);

    if (setDump == "ON" || stateCmdDump == true) {
      digitalWrite(RELAY_WCHARGE,RELAY_OFF);
      digitalWrite(RELAY_WDUMP,RELAY_ON);
    } else if (setDump == "OFF" && stateCmdDump == false) {
      digitalWrite(RELAY_WCHARGE,RELAY_ON);
      digitalWrite(RELAY_WDUMP,RELAY_OFF);
    }

    /** \brief handle thermostat
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
      digitalWrite(RELAY_HEAT, LOW);
    }
    else
    {
      digitalWrite(RELAY_HEAT, HIGH);
    }
    if (stateCooling == true && stateFan == false && stateCool == false)
    {
      digitalWrite(RELAY_FAN, LOW);
      stateFan = true;
    }
    else if (stateCooling == true && millis() - coolTimer >= 30000 && stateFan == true && stateCool == false)
    {
      stateCool = true;
      coolTimer = millis();
      digitalWrite(RELAY_COOL, LOW);
    }
    else if(stateCooling == false)
    {
      digitalWrite(RELAY_COOL, HIGH);
      digitalWrite(RELAY_FAN, HIGH);
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
    client.publish("hvac/action",sz);
    dtostrf(tempF, 4, 2, sz);
    client.publish("hvac/temperature/current",sz);
    dtostrf(humidityRH, 4, 2, sz);
    client.publish("hvac/humidity/current",sz);
    client.publish("camper/switch/wdump",stateActDump? "ON":"OFF");
    char tmpChar[40];
    for (int i=0; i<=ZONE_COUNT; i++) {
      pubStr[i].toCharArray(tmpChar, 40);
      client.publish(tmpChar,stateAct[i]? "ON":"OFF");
    }
  }
}
