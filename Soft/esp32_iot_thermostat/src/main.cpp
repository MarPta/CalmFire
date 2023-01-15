#include <Arduino.h>
#include <Preferences.h>
#include "Adafruit_Sensor.h"
#include "DHT.h"
#include "WiFi.h"
#include "WiFiClientSecure.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include "U8g2lib.h"

#include "OnOffRegulator.h"
#include "config.h"

const uint8_t dhtPin = 16;
const uint8_t relayPin = 34;
const uint8_t displayCLK = 23;
const uint8_t displayDIN = 19;
const uint8_t displayDC = 18;
const uint8_t displayCE = 5;
const uint8_t displayRST = 32;
const uint8_t displayBL = 33;
const uint8_t buttonUpPin = 12;
const uint8_t buttonDownPin = 14;
const uint8_t buttonOkPin = 27;
const uint16_t buttonPressDelay = 300;           // [ms]
const uint16_t buttonOkDelay = 2000;             // [ms]
const uint16_t displayReturnDelay = 3000;        // [ms]
const float defaultLowTemp = 10.0;               // [°C]
const float defaultHighTemp = 20.0;              // [°C]
const float desiredTempStep = 0.5;               // [°C]
const uint8_t defaultHeatMode = 0;               // Night

// LED PWM properties
const uint16_t freq = 5000;
const uint8_t ledChannel = 0;
const uint8_t resolution = 10;

const float desiredTempMax = 25.0;       // [°C]
const float desiredTempMin = 3.0;        // [°C]
const float hysteresisUpTemp = 0.5;      // [°C]
const float hysteresisDownTemp = 0.5;    // [°C]
const uint32_t measuredTempPeriod = 10000;    // 10 s [ms]
const uint32_t updateCloudPeriod = 300000;    // 5 min [ms]
const uint32_t cloudConnectPeriod = 10000;    // 10 s [ms]
const uint16_t dispUpdatePeriod = 1000;        // [ms]

enum WifiStatus {
    checkWifi,
    connectWifi,
    checkMqtt,
    connectMqtt,
    disconnected,
    connected,
    waiting
};

struct stateVector {
    float measuredTemp = 0.0;                  // [°C]
    float measuredHum = 0.0;                   // [%]
    float desiredTempLow = defaultLowTemp;     // [°C]
    float desiredTempHigh = defaultHighTemp;   // [°C]
    float desiredTemp = defaultLowTemp;        // [°C]
    uint8_t heatMode = 0;  // Select desired temp variable: 0-desiredTempLow, 1-desiredTempHigh
    bool heatOn = false;   // false-not heating, true-heating now
    int16_t displayBrightness = 0;   // 10-bit output
    uint8_t displayState = 0;        // 0-measuredTemp, 1-desiredTemp
    bool cloudConnected = false;         // false-unconnected, true-connected
} sv;

DHT dht(dhtPin, DHT22);
Preferences preferencesCF;
OnOffRegulator heatRegulator;
U8G2_PCD8544_84X48_F_4W_SW_SPI display = U8G2_PCD8544_84X48_F_4W_SW_SPI(U8G2_R0, displayCLK, displayDIN, displayCE, displayDC, displayRST);

WiFiClientSecure client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

/*
AdafruitIO_Feed *heatMode = io.feed("heatMode");
AdafruitIO_Feed *measuredTemp = io.feed("measuredTemp");
AdafruitIO_Feed *desiredTemp = io.feed("desiredTemp");
AdafruitIO_Feed *heatOn = io.feed("heatOn");
*/

Adafruit_MQTT_Publish measuredTemp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/testMeasuredTemp");
Adafruit_MQTT_Publish desiredTemp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/testDesiredTemp");
Adafruit_MQTT_Publish heatOn = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/testHeatOn");
Adafruit_MQTT_Publish heatMode = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/testHeatMode");

Adafruit_MQTT_Subscribe desiredTempSub = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/testDesiredTemp");
Adafruit_MQTT_Subscribe heatModeSub = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/testHeatMode");


void loadPreferences();
void savePreferences();
void measureTemp();
void sendMeasuredTemp(bool forceSend = false);
void sendDesiredTemp();
void sendHeatMode();
void sendHeatOn();
void cloudConnect(void * parameter);
void MQTT_checkSub();
void handleWiFiConnection();
void displayTemp(float temp);

void setup() {
    digitalWrite(relayPin, false);
    pinMode(relayPin, OUTPUT);
    pinMode(buttonUpPin, INPUT_PULLDOWN);
    pinMode(buttonDownPin, INPUT_PULLDOWN);
    pinMode(buttonOkPin, INPUT_PULLDOWN);

    Serial.begin(115200);

    display.begin();

    dht.begin();
    while(sv.measuredTemp < 0.01) {
        measureTemp();
        delay(100);
    }
    
    ledcSetup(ledChannel, freq, resolution);
    ledcAttachPin(displayBL, ledChannel);
    pinMode(displayBL, OUTPUT);

    heatRegulator.setParameters(hysteresisUpTemp, hysteresisDownTemp, desiredTempMax, desiredTempMin);

    loadPreferences();

    xTaskCreatePinnedToCore(cloudConnect, "cloudConnect", 10000 , NULL, 0, NULL, 1);
}

void loop() {
    // Updating sensor data
    measureTemp();

    // Handle UI
    MQTT_checkSub();
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
        displayTemp(sv.measuredTemp);

        if(buttonUpPressed || buttonDownPressed || buttonOkPressed) {
            sv.displayState = 1;
        }
    }
    else if(sv.displayState == 1) {
        // Display desired temp of the current heat mode
        displayTemp(sv.desiredTemp + tempChange);

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

    // Updating actuators according to current StateVector variables
    digitalWrite(relayPin, sv.heatOn);
    ledcWrite(ledChannel, sv.displayBrightness);

    printf("loop %lu\n", millis());
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

void measureTemp() {
    static uint32_t lastUpdate = 0;
    if((millis() > (lastUpdate + measuredTempPeriod)) || (lastUpdate == 0)) {
        lastUpdate = millis();

        float temp = dht.readTemperature();
        float hum = dht.readHumidity();
        if (isnan(hum) || isnan(temp)) {
            Serial.println(F("Failed to read from DHT sensor!"));
            lastUpdate = 0; // Repeat measurement
        }
        else {
            sv.measuredTemp = temp;
            sv.measuredHum = hum;
        }
    }
}
void sendMeasuredTemp(bool forceSend) {
    if (!sv.cloudConnected) {
        return;
    }

    static uint32_t lastUpdate = 0;
    if((forceSend || (millis() > (lastUpdate + updateCloudPeriod)) || (lastUpdate == 0))) {
        lastUpdate = millis();

        measuredTemp.publish(sv.measuredTemp);
        printf("measuredTemp\n");
    }
}
void sendDesiredTemp() {
    if (!sv.cloudConnected) {
        return;
    }

    desiredTemp.publish(sv.desiredTemp);
    printf("desiredTemp\n");
}
void sendHeatMode() {
    if (!sv.cloudConnected) {
        return;
    }

    heatMode.publish(sv.heatMode);
    printf("heatMode\n");
}
void sendHeatOn() {
    if (!sv.cloudConnected) {
        return;
    }

    static bool prevHeatOn = true;
    if(sv.heatOn != prevHeatOn) {
        prevHeatOn = sv.heatOn;
        heatOn.publish(sv.heatOn);
        printf("heatOn\n");
    }
}
void cloudConnect(void * parameter) {
    WiFi.begin(WLAN_SSID, WLAN_PASS);

    client.setCACert(adafruitio_root_ca);
    mqtt.subscribe(&desiredTempSub);
    mqtt.subscribe(&heatModeSub);
    WifiStatus wifiStatus = checkWifi;

    for(;;) {
        switch(wifiStatus) {
            case checkWifi: {
                if(WiFi.status() == WL_CONNECTED) {
                    wifiStatus = checkMqtt;
                }
                else {
                    wifiStatus = connectWifi;
                }
                break;
            }
            case connectWifi: {
                WiFi.reconnect();
                uint32_t connectStart = millis();
                while(WiFi.status() != WL_CONNECTED && (millis() < (connectStart + 3000))) {
                    delay(200);
                }
                if(WiFi.status() == WL_CONNECTED) {
                    wifiStatus = checkWifi;
                }
                else {
                    wifiStatus = disconnected;
                    printf("Failed connecting to wifi\n");
                } 
                break;
            }
            case checkMqtt: {
                if(mqtt.connected() == true) {
                    wifiStatus = connected;
                }
                else {
                    wifiStatus = connectMqtt;
                }
                break;
            }
            case connectMqtt: {
                mqtt.connect(); // blocking command (5s)
                if(mqtt.connected() == true) {
                    wifiStatus = checkWifi;
                }
                else {
                    mqtt.disconnect();
                    wifiStatus = disconnected;
                    printf("Failed connecting to MQTT broker\n");
                }
                break;
            }
            case disconnected: {
                WiFi.disconnect();
                mqtt.disconnect();
                sv.cloudConnected = false;
                wifiStatus = waiting;
                break;
            }
            case connected: {
                if(sv.cloudConnected == false) {
                    sv.cloudConnected = true;
                    sendDesiredTemp();
                    sendHeatMode();
                    sendHeatOn();
                }
                wifiStatus = waiting;
                break;
            }
            case waiting: {
                delay(cloudConnectPeriod);
                wifiStatus = checkWifi;
                break;
            }
            default:
                printf("Error in wifiState\n");
        }
    }
}
void MQTT_checkSub() {
    if (!sv.cloudConnected) {
        return;
    }

    Adafruit_MQTT_Subscribe *subscription;
    subscription = mqtt.readSubscription();
    if(subscription == &desiredTempSub) {
        sv.desiredTemp = atof((char *)desiredTempSub.lastread);
        if(sv.heatMode == 0) {
            sv.desiredTempLow = sv.desiredTemp;
        }
        if(sv.heatMode == 1) {
            sv.desiredTempHigh = sv.desiredTemp;
        }
        savePreferences();
        sendHeatOn();
    }
    else if(subscription == &heatModeSub) {
        sv.heatMode = atoi((char *)heatModeSub.lastread);
        if(sv.heatMode == 0) {
            sv.desiredTemp = sv.desiredTempLow;
        }
        else if (sv.heatMode == 1) {
            sv.desiredTemp = sv.desiredTempHigh;
        }

        savePreferences();
        sendDesiredTemp();
        sendHeatOn();
    }
}

void displayTemp(float temp) {
    static uint32_t prevDispUpdate = 0;
    if(millis() > prevDispUpdate + dispUpdatePeriod) {
        prevDispUpdate = millis();
    }
    else {
        return;
    }

    display.clearBuffer();
    char outString[16];

    display.setFont(u8g2_font_inb38_mn);
    sprintf(outString, "%2d", int(temp));
    display.drawStr(0, 37, outString);

    display.drawDisc(61, 35, 2, U8G2_DRAW_ALL);

    display.setFont(u8g2_font_inb24_mn);
    sprintf(outString, "%1d", int(temp*10 + 0.5)%10);
    display.drawStr(64, 37, outString);

    display.setFont(u8g2_font_tenthinnerguys_tf );
    sprintf(outString, "°C");
    display.drawUTF8(66, 10, outString);

    display.sendBuffer();
}