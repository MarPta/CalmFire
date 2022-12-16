#include <Arduino.h>
#include <Preferences.h>
#include "Adafruit_Sensor.h"
#include "DHT.h"
#include "WiFi.h"
#include "WiFiClientSecure.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

#include <HT1621_universal.h>
#include "OnOffRegulator.h"
#include "config.h"

const uint8_t dhtPin = 17;
const uint8_t displayCsPin = 14;   // Chip selection output
const uint8_t displayWrPin = 27;   // Read clock output
const uint8_t displayDataPin = 26; // Serial data output
const uint8_t displayLedPin = 13;
const uint8_t buttonUpPin = 5;
const uint8_t buttonDownPin = 18;
const uint8_t buttonOkPin = 19;
const uint16_t buttonPressDelay = 300;           // [ms]
const uint16_t buttonOkDelay = 2000;             // [ms]
const uint16_t displayNightLightThreshold = 300; // 0-4096 
const uint16_t displayNightLightMax = 80;        // 0-1023
const uint16_t displayReturnDelay = 3000;        // [ms]
const float defaultLowTemp = 10.0;               // [°C]
const float defaultHighTemp = 20.0;              // [°C]
const float desiredTempStep = 0.5;               // [°C]
const uint8_t defaultHeatMode = 0;               // Night

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
const uint32_t measuredTempUpdatePeriod = 10000;    // 10 s [ms]
const uint32_t updateCloudPeriod = 300000;          // 5 min [ms]


struct stateVector{
    float measuredTemp = 0.0;                  // [°C]
    float measuredHum = 0.0;                   // [%]
    int16_t measuredIlluminance = 0;           // 12-bit reading
    float desiredTempLow = defaultLowTemp;     // [°C]
    float desiredTempHigh = defaultHighTemp;   // [°C]
    float desiredTemp = defaultLowTemp;        // [°C]
    uint8_t heatMode = 0;  // Select desired temp variable: 0-desiredTempLow, 1-desiredTempHigh
    bool heatOn = false;   // false-not heating, true-heating now
    int16_t displayBrightness = 0;   // 10-bit output
    uint8_t displayState = 0;        // 0-measuredTemp, 1-desiredTemp
} sv;

DHT dht(dhtPin, DHT22);
Preferences preferencesCF;
OnOffRegulator heatRegulator;
HT1621_universal lcd(displayCsPin, displayWrPin, displayDataPin);

WiFiClientSecure client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

/*
AdafruitIO_Feed *heatMode = io.feed("heatMode");
AdafruitIO_Feed *measuredTemp = io.feed("measuredTemp");
AdafruitIO_Feed *desiredTemp = io.feed("desiredTemp");
AdafruitIO_Feed *heatOn = io.feed("heatOn");
*/

Adafruit_MQTT_Publish measuredTemp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/measuredTemp");


void loadPreferences();
void savePreferences();
//void handleHeatMode(AdafruitIO_Data *data);
//void handleDesiredTemp(AdafruitIO_Data *data);
void sendMeasuredTemp();
void sendDesiredTemp();
void sendHeatMode();
void sendHeatOn();


void MQTT_connect() {
    int8_t ret;

    // Stop if already connected.
    if (mqtt.connected()) {
        return;
    }

    Serial.print("Connecting to MQTT... ");

    uint8_t retries = 3;
    while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
        Serial.println(mqtt.connectErrorString(ret));
        Serial.println("Retrying MQTT connection in 5 seconds...");
        mqtt.disconnect();
        delay(5000);  // wait 5 seconds
        retries--;
        if (retries == 0) {
            // basically die and wait for WDT to reset me
            while (1);
        }
    }

    Serial.println("MQTT Connected!");
}

void setup() {
    digitalWrite(relayPin, false);
    pinMode(relayPin, OUTPUT);
    pinMode(buttonUpPin, INPUT_PULLDOWN);
    pinMode(buttonDownPin, INPUT_PULLDOWN);
    pinMode(buttonOkPin, INPUT_PULLDOWN);

    Serial.begin(115200);

    lcd.init();
    dht.begin();
    
    ledcSetup(ledChannel, freq, resolution);
    ledcAttachPin(displayLedPin, ledChannel);
    pinMode(displayLedPin, OUTPUT);

    heatRegulator.setParameters(hysteresisUpTemp, hysteresisDownTemp, desiredTempMax, desiredTempMin);

    WiFi.begin(WLAN_SSID, WLAN_PASS);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    client.setCACert(adafruitio_root_ca);

    //heatMode->onMessage(handleHeatMode);
    //desiredTemp->onMessage(handleDesiredTemp);

    loadPreferences();
    sendDesiredTemp();
    sendHeatMode();
}

void loop() {
    // Updating sensor data
    static uint32_t measuredTempLastUpdate = 0;
    if((millis() > measuredTempLastUpdate + measuredTempUpdatePeriod) || (measuredTempLastUpdate == 0)) {
        measuredTempLastUpdate = millis();
        float t = dht.readTemperature();
        float h = dht.readHumidity();
        if (isnan(h) || isnan(t)) {
            Serial.println(F("Failed to read from DHT sensor!"));
        }
        else {
            sv.measuredTemp = t;
            sv.measuredHum = h;
        }
    }
    sv.measuredIlluminance = analogRead(photoresistorPin);

    // Handle UI
    MQTT_connect();
    sendMeasuredTemp();
    sendHeatOn();

    bool buttonUpPressed = false;
    bool buttonDownPressed = false;
    bool buttonOkPressed = false;

    static uint32_t buttonLastPressed = 0;
    static uint32_t buttonOKLastReleased = 0;

    if(millis() > (buttonLastPressed + buttonPressDelay)) {
        buttonUpPressed = digitalRead(buttonUpPin);
        buttonDownPressed = digitalRead(buttonDownPin);
        buttonOkPressed = digitalRead(buttonOkPin);
    }

    if(buttonUpPressed || buttonDownPressed || buttonOkPressed) {
        buttonLastPressed = millis();
    }
    static float tempChange = 0;
    tempChange += desiredTempStep*(buttonUpPressed - buttonDownPressed);

    if(digitalRead(buttonOkPin) == false) {
        buttonOKLastReleased = millis();
    }
    if(millis() > buttonOKLastReleased + buttonOkDelay) {
        buttonOKLastReleased = millis();
        if(sv.heatMode == 0) {
            sv.heatMode = 1;
            sv.desiredTemp = sv.desiredTempHigh;
        }
        else if (sv.heatMode == 1) {
            sv.heatMode = 0;
            sv.desiredTemp = sv.desiredTempLow;
        }
        savePreferences();
        sendHeatMode();
        sendDesiredTemp();
    }

    if(sv.displayState == 0) {
        // Display measured temp
        lcd.displayCelsius(sv.measuredTemp);

        if(buttonUpPressed || buttonDownPressed || buttonOkPressed) {
            sv.displayState = 1;
        }
    }
    else if(sv.displayState == 1) {
        // Display desired temp of the current heat mode
        lcd.displayCelsius(sv.desiredTemp + tempChange);

        if(millis() > buttonLastPressed + displayReturnDelay) {
            // save selected temp
            sv.displayState = 0;
            sv.desiredTemp += tempChange;
            tempChange = 0;

            if(sv.heatMode == 0) {
                sv.desiredTempLow = sv.desiredTemp;
            }
            else if(sv.heatMode == 1) {
                sv.desiredTempHigh = sv.desiredTemp;
            }

            savePreferences();
            sendDesiredTemp();
        }
    }

    // Automatic control
    sv.heatOn = heatRegulator.getRegAction(sv.desiredTemp, sv.measuredTemp);

    int16_t brightness = displayNightLightThreshold - sv.measuredIlluminance;
    if(brightness < 0) {
        brightness = 0;
    }
    else if(brightness > displayNightLightMax) {
        brightness = displayNightLightMax;
    }
    sv.displayBrightness = brightness;

    // Updating actuators according to current StateVector variables
    digitalWrite(relayPin, sv.heatOn);
    ledcWrite(ledChannel, sv.displayBrightness);

    delay(100);
}

void loadPreferences() {
    preferencesCF.begin("settings", true);
    sv.desiredTempLow = preferencesCF.getFloat("desiredTempLow", defaultLowTemp);
    sv.desiredTempHigh = preferencesCF.getFloat("desiredTempHigh", defaultHighTemp);
    sv.desiredTemp = preferencesCF.getFloat("desiredTemp", defaultLowTemp);
    sv.heatMode = preferencesCF.getBool("heatMode", defaultHeatMode);
    preferencesCF.end();
}
void savePreferences() {
    preferencesCF.begin("settings", false);
    preferencesCF.putFloat("desiredTempLow", sv.desiredTempLow);
    preferencesCF.putFloat("desiredTempHigh", sv.desiredTempHigh);
    preferencesCF.putFloat("desiredTemp", sv.desiredTemp);
    preferencesCF.putBool("heatMode", sv.heatMode);
    preferencesCF.end();
}
/*void handleHeatMode(AdafruitIO_Data *data) {
    sv.heatMode = data->toBool();
    if(sv.heatMode == 0) {
        sv.desiredTemp = sv.desiredTempLow;
    }
    else if (sv.heatMode == 1) {
        sv.desiredTemp = sv.desiredTempHigh;
    }

    savePreferences();
    sendDesiredTemp();
}
void handleDesiredTemp(AdafruitIO_Data *data) {
    sv.desiredTemp = data->toFloat();
    if(sv.heatMode == 0) {
        sv.desiredTempLow = sv.desiredTemp;
    }
    if(sv.heatMode == 1) {
        sv.desiredTempHigh = sv.desiredTemp;
    }
    savePreferences();
}*/
void sendMeasuredTemp() {
    static uint32_t lastUpdateTemp = 0;
    if((millis() > (lastUpdateTemp + updateCloudPeriod)) || (lastUpdateTemp == 0)) {
        lastUpdateTemp = millis();
        measuredTemp.publish(sv.measuredTemp);
    printf("measuredTemp\n");
    }
}
void sendDesiredTemp() {
    //desiredTemp->save(sv.desiredTemp);
    printf("desiredTemp\n");
}
void sendHeatMode() {
    //heatMode->save(sv.heatMode);
    printf("heatMode\n");
}
void sendHeatOn() {
    static bool prevHeatOn = true;
    if(sv.heatOn != prevHeatOn) {
        prevHeatOn = sv.heatOn;
        //heatOn->save(sv.heatOn);
        printf("heatOn\n");
    }
}