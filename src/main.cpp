#include <Arduino.h>

#ifdef ESP32
    #include <WiFi.h>
    #include <ESPmDNS.h>
#else
    #include <ESP8266WiFi.h>
    #include <ESP8266mDNS.h>
#endif

#include <DallasTemperature.h>
#include <PubSubClient.h>
#include <AutoPID.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <EEPROM.h>
#include <SSD1306Wire.h>
#include <Ticker.h>
#include <MovingAverageFloat.h>

#include <config.h>
#include <font.h>  // Dialog_plain_13

#define EEPROM_SIZE 4  // holds only targetTemp atm

#ifdef ESP32
    const uint8_t buildinLED = 2;
    const uint8_t oledSDSPin = 4;
    const uint8_t oledSCLPin = 15;
    const uint8_t heaterPin = 22;
    const uint8_t tempSensorPin = 26;

    const uint8_t fanTachoPin = 10;
    const uint8_t servoPin= 19;
#else
    const uint8_t buildinLED = 2;
    const uint8_t oledSDSPin = 13;
    const uint8_t oledSCLPin = 12;
    const uint8_t heaterPin = 4;
    const uint8_t tempSensorPin = 5;

    const uint8_t fanTachoPin = 14;
    const uint8_t servoPin = 10;
#endif  

#define SECONDS 1000

const char* clientId = "incubator";

char topicSubTargetTemp[] = "incubator/target_temp";
char topicSubCommand[] = "incubator/cmd";
char topicSubReset[] = "incubator/reset";
char topicSubKp[] = "incubator/kp";
char topicSubKd[] = "incubator/kd";
char topicSubKi[] = "incubator/ki";
char topicSubOn[] = "incubator/on";

char topicPubEchoOnState[] = "incubator/_echo/on_state";
char topicPubEchoTargetTemp[] = "incubator/_echo/target_temp";
char topicPubEchoKp[] = "incubator/_echo/kp";
char topicPubEchoKd[] = "incubator/_echo/kd";
char topicPubEchoKi[] = "incubator/_echo/ki";
char topicPubEchoReset[] = "incubator/_echo/reset";

char topicPubTemp[] = "incubator/temp";
char topicPubRssi[] = "incubator/rssi";
char topicPubRpm[] = "incubator/rpm";
char topicPubHeater[] = "incubator/heater";

const uint8_t tempResolution = 12;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

const int pwmFreq = 20;

#ifdef ESP32
    const int ledChannel = 0;
    const int resolution = 8;
#endif

bool onState = true;

const uint8_t shutdownTemperature = 50;
double lastHeaterVal = 0;
double heaterVal = 0;
double targetTemp = 34.00;  // this will be written to flash when changed over MQTT
double measuredTemp = 0;
double currentTemp = 0;
double lastTemp = 0;
uint8_t lastHeaterPercent = 0;
uint8_t heaterPercent = 0;
uint8_t welcomeCleared = false;

OneWire oneWire(tempSensorPin);
DallasTemperature sensors(&oneWire);

MovingAverageFloat <8> filter;

// PID settings
#define OUTPUT_MIN 0
#ifdef ESP32
    #define OUTPUT_MAX 255
#else
    #define OUTPUT_MAX 1023
#endif

bool wifiIconVisible = false;
static unsigned char wifiIconBits[] = {
   0x7c, 0x00, 0x82, 0x00, 0x39, 0x01, 0x44, 0x00, 0x92, 0x00, 0x38, 0x00,
   0x38, 0x00, 0x10, 0x00 };

float kp = 850.0;
float kd = 250.0;
float ki = 0.1;

void checkWiFi();
void flashWifi();
void checkMqtt();
void measureTemp();
void checkFan();
void drawGraph();
void reportStatus();
void updateHeater();

uint32_t checkMqttInterval = 60 * SECONDS;  // after first connect

const uint16_t measureTempInterval = 1000;
Ticker tickerTemp = Ticker(measureTemp, measureTempInterval);
Ticker tickerCheckMqtt = Ticker(checkMqtt, 10 * SECONDS);

uint8_t checkFanInterval = 1;
uint16_t minFanRpm = 2000;
uint16_t fanRpm = 0;

uint8_t tickerCount = 8;
Ticker tickers[] = {
    Ticker(checkWiFi, 5 * SECONDS),
    Ticker(flashWifi, 1 * SECONDS),
    tickerCheckMqtt,
    tickerTemp,
    Ticker(checkFan, checkFanInterval * SECONDS),
    Ticker(drawGraph, 5 * SECONDS),
    Ticker(reportStatus, 10 * SECONDS),
    Ticker(updateHeater, 1 * SECONDS)
};

AutoPID heaterPID(&currentTemp, &targetTemp, &heaterVal, OUTPUT_MIN, OUTPUT_MAX, kp, ki, kd);
SSD1306Wire display(0x3c, oledSDSPin, oledSCLPin);

// I'm using an OLED display with two color regions,
const uint8_t displayWidth = 128;
const uint8_t displayHeight = 64;
const uint8_t displayUpperHeight = 16;
const uint8_t graphWidth = 128;
const uint8_t graphHeight = displayHeight - displayUpperHeight;
const uint8_t graphX = 0;
const uint8_t graphY = displayUpperHeight;

uint8_t displayLowerTemp = 15;
uint8_t displayUpperTemp = 40;
const uint8_t tempHistorySize = graphWidth - 2; // without 2 pixels for the frame

int tempHistory[tempHistorySize];
char tmpBuffer[32];
long counter = 0;
int reconnectCount = 0;
bool measureTempMode = true;
bool validMeasurement = false;

volatile uint16_t halfRevolutions = 0;

ICACHE_RAM_ATTR void onTacho() {
    halfRevolutions++;
}

template <class T>
void publish(char* topic, T value, bool retained=true) {
    // Serial.print("publish: ");
    // Serial.println(value);

    String tempStr = String(value);
    tempStr.toCharArray(tmpBuffer, tempStr.length() + 1);

    mqttClient.publish(topic, tmpBuffer, retained);
    mqttClient.loop();
}

void setOnState(bool state) {
    if(state) {
        heaterPID.run();
        onState = true;
    } else {
        heaterPID.stop();
        heaterVal = 0;
        onState = false;
    }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    Serial.print("receiving: ");
    Serial.print(topic);
    Serial.print(' ');
    Serial.println((char*)payload);

    // publish((char*)"incubator/_echo/rcv", (char*)payload);
    // mqttClient.loop();

    if(strcmp(topic, topicSubTargetTemp) == 0) {
        targetTemp = atof((char*)payload);
        if(targetTemp > 50) {
            return;
        } else if(targetTemp < 1) {
            setOnState(false);
        } else {
            setOnState(true);
        }

        EEPROM.write(0, targetTemp);
        EEPROM.commit();
        publish(topicPubEchoTargetTemp, targetTemp);
    }

    else if(strcmp(topic, topicSubOn) == 0) {
        bool state = atoi((char*)payload) > 0;
        setOnState(state);
        publish(topicPubEchoOnState, onState);
    }

    else if(strcmp(topic, topicSubKp) == 0) {
        kp = atof((char*)payload);
        heaterPID.setGains(kp, ki, kd);
        publish(topicPubEchoKp, kp);
    }

    else if(strcmp(topic, topicSubKd) == 0) {
        kd = atof((char*)payload);
        heaterPID.setGains(kp, ki, kd);
        publish(topicPubEchoKd, kd);
    }

    else if(strcmp(topic, topicSubKi) == 0) {
        ki = atof((char*)payload);
        heaterPID.setGains(kp, ki, kd);
        publish(topicPubEchoKi, ki);
    }

    else if(strcmp(topic, topicSubReset) == 0) {
        publish(topicPubEchoReset, (char*) "reset");
        ESP.restart();
    }

    // else if(strcmp(topic, topicSubCommand) == 0) {
    // 	publish((char *)"incubator/_echo/cmd", (char*)payload);
    // 	if(strcmp((char*)payload, "reset") == 0) {
    //     	ESP.restart();
    //     }
    // }
    // mqttClient.loop();
}

void setupOTA() {
    ArduinoOTA.setHostname("incubator");

    ArduinoOTA.onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH) {
            type = "sketch";
        } else { // U_FS
            type = "filesystem";
        }

        // NOTE: if updating FS this would be the place to unmount FS using FS.end()
        Serial.println("Start updating " + type);
    });
    ArduinoOTA.onEnd([]() {
        Serial.println("\nEnd");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) {
            Serial.println("Auth Failed");
        } else if (error == OTA_BEGIN_ERROR) {
            Serial.println("Begin Failed");
        } else if (error == OTA_CONNECT_ERROR) {
            Serial.println("Connect Failed");
        } else if (error == OTA_RECEIVE_ERROR) {
            Serial.println("Receive Failed");
        } else if (error == OTA_END_ERROR) {
            Serial.println("End Failed");
        }
    });
    ArduinoOTA.begin();
}

void publishTemp() {
    Serial.print("temp: ");
    Serial.println(currentTemp);
    publish(topicPubTemp, currentTemp);
}

void publishHeater() {
    publish(topicPubHeater, heaterPercent);
}

void initOled() {
    pinMode(16,OUTPUT);
    digitalWrite(16, LOW);
    delay(50); 
    digitalWrite(16, HIGH); // while OLED is running, must set GPIO16 in high、    
}

void setHeater(int value) {
    Serial.print("setting heater to: ");
    Serial.println(value);
#ifdef ESP32
    ledcWrite(ledChannel, value);
#else
    analogWrite(heaterPin, value);
#endif
}

void updateHeater() {
    // Serial.println("heater");
    // Serial.print("valid: ");
    // Serial.println(validMeasurement);
    // Serial.print("heaterVal: ");
    // Serial.println(heaterVal);
    // Serial.print("lastHeaterVal: ");
    // Serial.println(lastHeaterVal);

    if(validMeasurement and fanRpm >= minFanRpm) {

        if(heaterVal != lastHeaterVal) {
            lastHeaterVal = heaterVal;

            setHeater(heaterVal);
            heaterPercent = heaterVal * 100 / OUTPUT_MAX;

            if(heaterPercent != lastHeaterPercent) {
                lastHeaterPercent = heaterPercent;
                publishHeater();
            }
        }

    // safety feature
    } else {
        setHeater(0);
    }
}

void checkWiFi() {
    if (WiFi.status() != WL_CONNECTED) {
        tickerCheckMqtt.stop();
        reconnectCount++;
        digitalWrite(buildinLED, HIGH);
        Serial.print("WiFi reconnect (");
        Serial.print(reconnectCount);
        Serial.println(")");
        WiFi.begin();
    } else {
        tickerCheckMqtt.start();
        if(reconnectCount > 0) {
            Serial.print("WiFi connected as ");
            Serial.println(WiFi.localIP());
            digitalWrite(buildinLED, LOW);
        }
        reconnectCount = 0;
    }
}

void checkMqtt() {
    if(mqttClient.connected()) return;

    Serial.println("MQTT reconnect");
    if (mqttClient.connect(clientId, mosquittoUser, mosquittoPassword)) {
        // increase interval after first connect
        Serial.println("MQTT connected");
        tickerCheckMqtt.interval(checkMqttInterval);
    } else {
        Serial.print("failed, rc=");
        Serial.print(mqttClient.state());
    }
    mqttClient.subscribe(topicSubTargetTemp);
    mqttClient.subscribe(topicSubCommand);
    mqttClient.subscribe(topicSubReset);
    mqttClient.subscribe(topicSubKp);
    mqttClient.subscribe(topicSubKd);
    mqttClient.subscribe(topicSubKi);
    mqttClient.subscribe(topicSubOn);
}

void reportStatus() {
    publish(topicPubRssi, WiFi.RSSI());
    publish(topicPubRpm, fanRpm);
    publishTemp();
    publishHeater();
}

void measureTemp() {
    // start a new measurement and in the next run get the result
    if(measureTempMode) {
        tickerTemp.interval(measureTempInterval);
        sensors.requestTemperaturesByIndex(0);
        measureTempMode = false;

    // read the result
    } else {
        tickerTemp.interval(0);
        measuredTemp = sensors.getTempCByIndex(0);
        if(measuredTemp >= shutdownTemperature) {
            Serial.println(measuredTemp);
            Serial.println("too hot! rebooting!");
            ESP.restart();
        }

        if(measuredTemp > 0) {
            validMeasurement = true;

            if(!welcomeCleared) {
                display.clear();
                display.setColor(WHITE);
                display.drawRect(graphX, graphY, graphWidth, graphHeight);
                display.display();
                welcomeCleared = true;
            }

            currentTemp = measuredTemp;
            filter.add(currentTemp);
            currentTemp = filter.get();

            display.setColor(BLACK);
            display.fillRect(0, 0, display.getWidth() - 9, 16);
            display.setColor(WHITE);

            sprintf(tmpBuffer, "%2.2f°/%d°@%d%%", currentTemp, int(targetTemp), heaterPercent);
            display.drawString(0, 0, tmpBuffer);
            display.display();
        } else {
            validMeasurement = false;
            lastHeaterVal = 0;
        }

        if(currentTemp != lastTemp) {
            publishTemp();
            lastTemp = currentTemp;
        }
        measureTempMode = true;
    }
}
void drawGraph() {
    memcpy(tempHistory, &tempHistory[1], sizeof(tempHistory) - sizeof(int));
    if(validMeasurement) {
        tempHistory[tempHistorySize - 1] = currentTemp;
    } else {
        tempHistory[tempHistorySize - 1] = 0;
    }

    // clear the draw area
    display.setColor(BLACK);
    display.fillRect(graphX + 1, graphY + 1, graphWidth - 2, graphHeight - 2);
    display.setColor(WHITE);

    // draw history graph
    for(uint8_t i = 0; i < tempHistorySize; i++) {
        uint8_t temp = tempHistory[i];

        uint8_t displayTemp = temp;

        if(displayTemp > displayUpperTemp) {
            displayTemp = displayUpperTemp;
        } else if(displayTemp < displayLowerTemp) {
            displayTemp = displayLowerTemp;
        }
        uint8_t y = (displayTemp - displayLowerTemp) * (graphHeight - 2) / (displayUpperTemp - displayLowerTemp);

        // uint8_t y = graphHeight;
        display.setPixel(
            graphX + 1 + i,
            graphY + graphHeight - 1 - y
        );
    }
    display.display();
}

void checkFan() {
    fanRpm = (halfRevolutions / 2) / checkFanInterval * 60;
    halfRevolutions = 0;
    Serial.print("RPM: ");
    Serial.println(fanRpm);
}

// flash wifi icon while connected
void flashWifi() {
    if(! welcomeCleared) return;
    if(WiFi.status() == WL_CONNECTED) {
        display.setColor(BLACK);
        display.fillRect(graphWidth - 9, 0, 9, 8);
        display.setColor(WHITE);
        display.drawXbm(graphWidth - 9, 0, 9, 8, wifiIconBits);
    } else {
        if(wifiIconVisible) {
            display.setColor(WHITE);
            display.drawXbm(graphWidth - 9, 0, 9, 8, wifiIconBits);
        } else {
            display.setColor(BLACK);
            display.fillRect(graphWidth - 9, 0, 9, 8);
        }
    }
    display.display();
    wifiIconVisible = !wifiIconVisible;
}

void setup() {
    delay(3 * SECONDS);
    Serial.println("start");

    pinMode(buildinLED, OUTPUT);
    pinMode(tempSensorPin, INPUT_PULLUP);
#ifdef ESP32
#else
    analogWriteFreq(pwmFreq);
#endif
    pinMode(fanTachoPin, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(fanTachoPin), onTacho, FALLING);

    DeviceAddress tempDeviceAddress;

    setHeater(0);

    sensors.begin();
    sensors.getAddress(tempDeviceAddress, 0);
    sensors.setResolution(tempDeviceAddress, 12);
    sensors.setWaitForConversion(false);  // async
    if(sensors.getDeviceCount() == 0) {
        Serial.println("No temperature sensor found! Rebooting...");
        // delay(5000);
        // ESP.restart();
    }

    // scan();
    // initOled();  // only needed for a special ESP32/OLED Board

    EEPROM.begin(EEPROM_SIZE);
    targetTemp = EEPROM.read(0);

    // TODO
    // pinMode(fanTachoPin, INPUT_PULLUP);
    // attachInterrupt(digitalPinToInterrupt(fanTachoPin), tachoSignal, RISING);

    Serial.begin(115200);

    Serial.println("Booting");
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    checkWiFi();

    setupOTA();

    // prepare the display
    display.init();
    display.clear();
    display.flipScreenVertically();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setColor(WHITE);
    display.setFont(Dialog_plain_13);
    display.drawString(0, 0, "Let's make tempeh!");
    display.display();

    #ifdef ESP32
        ledcSetup(ledChannel, pwmFreq, resolution);
        ledcAttachPin(heaterPin, ledChannel);
    #else
        pinMode(heaterPin, OUTPUT);
    #endif

    heaterPID.setBangBang(5); // run the heater full bang until 5°c below targetTemperature
    heaterPID.setTimeStep(1000); // run pid calculation every n seconds

    mqttClient.setServer(mqttServer, mqttPort);
    mqttClient.setCallback(mqttCallback);

    // start all tickers
    for(uint8_t i = 0; i < tickerCount; i++) {
        tickers[i].start();
    }
}

void loop() {
    //scan();
    ArduinoOTA.handle();
    mqttClient.loop();
    heaterPID.run();

    // checkk all tickers
    for(uint8_t i = 0; i < tickerCount; i++) {
        tickers[i].update();
    }
}
