#include <Arduino.h>
#include <Preferences.h>
#include "Adafruit_Sensor.h"
#include "DHT.h"

#include <HT1621_universal.h>
#include "OnOffRegulator.h"
#include "encoder.h"

const uint8_t dhtPin = 17;
const uint8_t displayCsPin = 14;   // Chip selection output
const uint8_t displayWrPin = 27;   // Read clock output
const uint8_t displayDataPin = 26; // Serial data output
const uint8_t DISPLAY_LED = 13;
const uint16_t displayNightLightThreshold = 300; // 0-4096 
const uint16_t displayNightLightMax = 80; // 0-1023
const uint16_t displayModDelay = 3000;  // [ms]
const uint8_t defaultLowTemp = 10;  // [°C]
const uint8_t defaultHighTemp = 20;  // [°C]

// LED PWM properties
const uint16_t freq = 5000;
const uint8_t ledChannel = 0;
const uint8_t resolution = 10;

const uint8_t photoresistorPin = 35;
const uint8_t relayPin = 16;
const float desiredTempMax = 30.0;   // [°C]
const float desiredTempMin = 5.0;   // [°C]
const float hysteresisUpTemp = 0.3;      // [°C]
const float hysteresisDownTemp = 0.3;    // [°C]

struct stateVector{
    float actualTemp = 0.0;         // [°C]
    float desiredTempLow = defaultLowTemp;    // [°C]
    float desiredTempHigh = defaultHighTemp;   // [°C]
    bool heatHigh = false;  // Choose desired temp: 0->desiredTempLow, 1->desiredTempHigh
    float relHum = 0.0;     // [%]
    bool relayOn = false;
    int16_t illuminance = 0;   // 12-bit reading
    int16_t displayBrightness = 0;   // 10-bit output
    uint8_t displayState = 0;   // 0-measuredTemp, 1-desiredTemp
} currentStateVector;

DHT dht(dhtPin, DHT22);
Preferences flashPreferences;
OnOffRegulator heatRegulator;
HT1621_universal lcd(displayCsPin, displayWrPin, displayDataPin);
Encoder encoder1(1);

template <typename T>
T clamp(T value, T min, T max) {
	if (value < min)
		return min;
	if (value > max)
		return max;
	return value;
}

void setup() {
    Serial.begin(115200);

    lcd.init();
    digitalWrite(relayPin, 0);
    pinMode(relayPin, OUTPUT);
    pinMode(DISPLAY_LED, OUTPUT);
    ledcSetup(ledChannel, freq, resolution);
    ledcAttachPin(DISPLAY_LED, ledChannel);

    dht.begin();

    flashPreferences.begin("desiredTemps", false);  //  RW-mode (second parameter false).
    currentStateVector.desiredTempLow = flashPreferences.getFloat("desiredTempLow", defaultLowTemp);
    currentStateVector.desiredTempHigh = flashPreferences.getFloat("desiredTempHigh", defaultHighTemp);
    currentStateVector.heatHigh = flashPreferences.getBool("heatHigh", false);
    flashPreferences.end();

    heatRegulator.setParameters(hysteresisUpTemp, hysteresisDownTemp, desiredTempMax, desiredTempMin);
}

void loop() {

    // Updating sensor data
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

    // Automatic control
    if(currentStateVector.heatHigh)
        currentStateVector.relayOn = heatRegulator.getRegAction(currentStateVector.desiredTempHigh, currentStateVector.actualTemp);
    else
        currentStateVector.relayOn = heatRegulator.getRegAction(currentStateVector.desiredTempLow, currentStateVector.actualTemp);

    currentStateVector.displayBrightness = displayNightLightThreshold - currentStateVector.illuminance;
    currentStateVector.displayBrightness = clamp<int>(currentStateVector.displayBrightness, 0, displayNightLightMax);

    static uint32_t lastMovedEnc = 0;
    switch(currentStateVector.displayState) {
        case 0:
            lcd.displayCelsius(currentStateVector.actualTemp);
            if(encoder1.getDiff() != 0) {
                currentStateVector.displayState = 1;
                lastMovedEnc = millis();
            }
            break;
        case 1:
            int16_t diff = encoder1.getDiff();
            if(diff != 0)
                lastMovedEnc = millis();
            if(currentStateVector.heatHigh == 0) {
                currentStateVector.desiredTempLow += diff * 0.05;
                currentStateVector.desiredTempLow = clamp<float>(currentStateVector.desiredTempLow, desiredTempMin, desiredTempMax);
                lcd.displayCelsius(currentStateVector.desiredTempLow);
            }
            else {
                currentStateVector.desiredTempHigh += diff * 0.05;
                currentStateVector.desiredTempHigh = clamp<float>(currentStateVector.desiredTempHigh, desiredTempMin, desiredTempMax);
                lcd.displayCelsius(currentStateVector.desiredTempHigh);
            }
            if(millis() > lastMovedEnc + displayModDelay) {
                currentStateVector.displayState = 0;
            }
            break;
    };

    /*char message[20];
    snprintf (message, 20, "%.1f", currentStateVector.actualTemp);
    mqttClient.publish(tempTopic, message);*/

    // Updating actuators accoring to current StateVector variables
    digitalWrite(relayPin, currentStateVector.relayOn);
    ledcWrite(ledChannel, currentStateVector.displayBrightness);

    delay(20);
}