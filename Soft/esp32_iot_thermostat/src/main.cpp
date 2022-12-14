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
const uint8_t displayLedPin = 13;
const uint8_t buttonUpPin = 5;
const uint8_t buttonDownPin = 18;
const uint8_t buttonOkPin = 19;
const uint16_t buttonPressDelay = 200;           // [ms]
const uint16_t displayNightLightThreshold = 300; // 0-4096 
const uint16_t displayNightLightMax = 80;        // 0-1023
const uint16_t displayReturnDelay = 3000;        // [ms]
const float defaultLowTemp = 10.0;               // [°C]
const float defaultHighTemp = 20.0;              // [°C]
const float desiredTempStep = 0.2;               // [°C]
const uint8_t defaultHeatMode = 0;

// LED PWM properties
const uint16_t freq = 5000;
const uint8_t ledChannel = 0;
const uint8_t resolution = 10;

const uint8_t photoresistorPin = 35;
const uint8_t relayPin = 16;
const float desiredTempMax = 25.0;       // [°C]
const float desiredTempMin = 3.0;        // [°C]
const float hysteresisUpTemp = 0.5;      // [°C]
const float hysteresisDownTemp = 0.5;    // [°C]

struct stateVector{
    float measuredTemp = 0.0;                  // [°C]
    float measuredHum = 0.0;                   // [%]
    int16_t measuredIlluminance = 0;           // 12-bit reading
    float desiredTempLow = defaultLowTemp;     // [°C]
    float desiredTempHigh = defaultHighTemp;   // [°C]
    uint8_t heatMode = 0;  // Select desired temp variable: 0-desiredTempLow, 1-desiredTempHigh
    bool relayOn = false;
    int16_t displayBrightness = 0;   // 10-bit output
    uint8_t displayState = 0;        // 0-measuredTemp, 1-desiredTemp
} sv;

DHT dht(dhtPin, DHT22);
Preferences preferencesCF;
OnOffRegulator heatRegulator;
HT1621_universal lcd(displayCsPin, displayWrPin, displayDataPin);

void setup() {
    Serial.begin(115200);

    lcd.init();
    dht.begin();
    //digitalWrite(relayPin, false);
    pinMode(relayPin, OUTPUT);
    pinMode(buttonUpPin, INPUT_PULLDOWN);
    pinMode(buttonDownPin, INPUT_PULLDOWN);
    pinMode(buttonOkPin, INPUT_PULLDOWN);
    
    ledcSetup(ledChannel, freq, resolution);
    ledcAttachPin(displayLedPin, ledChannel);
    pinMode(displayLedPin, OUTPUT);

    preferencesCF.begin("settings", false);
    sv.desiredTempLow = preferencesCF.getFloat("desiredTempLow", defaultLowTemp);
    sv.desiredTempHigh = preferencesCF.getFloat("desiredTempHigh", defaultHighTemp);
    sv.heatMode = preferencesCF.getBool("heatMode", defaultHeatMode);
    preferencesCF.end();

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
        sv.measuredTemp = t;
        sv.measuredHum = h;
    }
    sv.measuredIlluminance = analogRead(photoresistorPin);

    // Automatic control
    static float desiredTemp = 0.0;
    if(sv.heatMode == 0) {
        // Night mode
        desiredTemp = sv.desiredTempLow;
    }
    else if(sv.heatMode == 1) {
        // Day mode
        desiredTemp = sv.desiredTempHigh;
    }
    sv.relayOn = heatRegulator.getRegAction(desiredTemp, sv.measuredTemp);

    int16_t brightness = displayNightLightThreshold - sv.measuredIlluminance;
    if(brightness < 0) {
        brightness = 0;
    }
    else if(brightness > displayNightLightMax) {
        brightness = displayNightLightMax;
    }
    sv.displayBrightness = brightness;

    // Handle UI
    bool buttonUpPressed = false;
    bool buttonDownPressed = false;
    bool buttonOkPressed = false;

    static uint32_t buttonLastPressed = 0;

    if(millis() > (buttonLastPressed + buttonPressDelay)) {
        buttonUpPressed = digitalRead(buttonUpPin);
        buttonDownPressed = digitalRead(buttonDownPin);
        buttonOkPressed = digitalRead(buttonOkPin);
    }

    if(buttonUpPressed || buttonDownPressed || buttonOkPressed) {
        buttonLastPressed = millis();
    }
    float tempChange = -desiredTempStep*buttonDownPressed + desiredTempStep*buttonUpPressed;

    if(sv.displayState == 0) {
        // Display measured temp
        lcd.displayCelsius(sv.measuredTemp);

        if(buttonUpPressed || buttonDownPressed || buttonOkPressed) {
            sv.displayState = 1;
        }        
    }
    else if(sv.displayState == 1) {
        // Display desired temp of the current heat mode
        if(sv.heatMode == 0) {
            sv.desiredTempLow += tempChange;
            lcd.displayCelsius(sv.desiredTempLow);
        }
        else if(sv.heatMode == 1) {
            sv.desiredTempHigh += tempChange;
            lcd.displayCelsius(sv.desiredTempHigh);
        }

        if(millis() > buttonLastPressed + displayReturnDelay) {
            // save selected temp
            sv.displayState = 0;

            preferencesCF.begin("settings", false);
            preferencesCF.putFloat("desiredTempLow", sv.desiredTempLow);
            preferencesCF.putFloat("desiredTempHigh", sv.desiredTempHigh);
            preferencesCF.end();
        }
    }

    // Updating actuators accoring to current StateVector variables
    digitalWrite(relayPin, !sv.relayOn);
    ledcWrite(ledChannel, sv.displayBrightness);

    delay(100);
}