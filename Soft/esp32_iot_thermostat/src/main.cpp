#include <Arduino.h>
#include "Adafruit_Sensor.h"
#include "DHT.h"
#include <WiFi.h>
#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>

#include <HT1621_universal.h>

#include "Strings.h"
#include "Control/OnOffRegulator.h"

const uint8_t dhtPin = 17;

const uint8_t displayCsPin = 14;   //Chip selection output
const uint8_t displayWrPin = 27;   //Read clock output
const uint8_t displayDataPin = 26; //Serial data output
const uint8_t DISPLAY_LED = 13;
const uint16_t displayNightLightThreshold = 300; //0-4096 
const uint16_t displayNightLightMax = 80; //0-4096 

//LED PWM properties
const uint16_t freq = 5000;
const uint8_t ledChannel = 0;
const uint8_t resolution = 10;

const uint8_t photoresistorPin = 35;
const uint8_t relayPin = 16;
const float dsiredTempMax = 25.0;   //[°C]
const float dsiredTempMin = 5.0;   //[°C]
const float hysteresisUpTemp = 0.3;      //[°C]
const float hysteresisDownTemp = 0.3;    //[°C]

struct stateVector{
    float actualTemp = 0.0;         //[°C]
    float desiredTempLow = 10.0;    //[°C]
    float desiredTempHigh = 20.0;   //[°C]
    bool heatHigh = false;  //Choose desired temp: 0->desiredTempLow, 1->desiredTempHigh
    float relHum = 0.0;     //[%]
    bool relayOn = false;
    int16_t illuminance = 0;   //10-bit reading
    int16_t displayBrightness = 0;   //10-bit output
} currentStateVector;

DHT dht(dhtPin, DHT22);
WiFiManager wifiManager;
uint32_t delayMS;
//EEPROM_data desiredTempLowEEPROM(&(currentStateVector.desiredTempLow), sizeof(currentStateVector.desiredTempLow));
//EEPROM_data desiredTempHighEEPROM(&(currentStateVector.desiredTempHigh), sizeof(currentStateVector.desiredTempHigh));
//EEPROM_data heatHighEEPROM(&(currentStateVector.heatHigh), sizeof(currentStateVector.heatHigh));
OnOffRegulator heatRegulator;
HT1621_universal lcd(displayCsPin, displayWrPin, displayDataPin);

template <typename T>
T clamp(T value, T min, T max) {
	if (value < min)
		return min;
	if (value > max)
		return max;
	return value;
}

void setup(){
    lcd.init();

    Serial.begin(115200);
    pinMode(relayPin, OUTPUT);
    pinMode(DISPLAY_LED, OUTPUT);
    digitalWrite(relayPin, 1);
    ledcSetup(ledChannel, freq, resolution);
    ledcAttachPin(DISPLAY_LED, ledChannel);

    wifiManager.autoConnect("IoT Thermostat", "12345678");
    dht.begin();

    //desiredTempLowEEPROM.read();
    //desiredTempHighEEPROM.read();
    //heatHighEEPROM.read();

    heatRegulator.setParameters(hysteresisUpTemp, hysteresisDownTemp, dsiredTempMax, dsiredTempMin);
}

void loop(){
    //Updating sensor data
    float t = dht.readTemperature();
    float h = dht.readHumidity();
    if (isnan(h) || isnan(t)) {
        Serial.println(F("Failed to read from DHT sensor!"));
    }
    else {
        currentStateVector.actualTemp = t;
        currentStateVector.relHum = h;
    }
    currentStateVector.illuminance = analogRead(photoresistorPin);

    Serial.print("Temperature: ");
    Serial.print(currentStateVector.actualTemp);
    Serial.println(" *C");
    Serial.println(currentStateVector.illuminance);

    Serial.print("Temp Low: ");
    Serial.print(currentStateVector.desiredTempLow);
    Serial.print("  Temp High: ");
    Serial.print(currentStateVector.desiredTempHigh);
    Serial.print("  HeatHigh: ");
    Serial.println(currentStateVector.heatHigh);

    lcd.displayCelsius(currentStateVector.actualTemp);

    //Automatic control
    if(currentStateVector.heatHigh)
        currentStateVector.relayOn = heatRegulator.getRegAction(currentStateVector.desiredTempHigh, currentStateVector.actualTemp);
    else
        currentStateVector.relayOn = heatRegulator.getRegAction(currentStateVector.desiredTempLow, currentStateVector.actualTemp);

    currentStateVector.displayBrightness = displayNightLightThreshold - currentStateVector.illuminance;
    currentStateVector.displayBrightness = clamp<int>(currentStateVector.displayBrightness, 0, displayNightLightMax);

    //Updating actuators accoring to current StateVector variables
    digitalWrite(relayPin, currentStateVector.relayOn);
    ledcWrite(ledChannel, currentStateVector.displayBrightness);

    delay(500);
}