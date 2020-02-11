#include <Arduino.h>
#include <DallasTemperature.h>
#include <PubSubClient.h>
#include <AutoPID.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <EEPROM.h>
#include <font.h>
#include <SSD1306Wire.h>

#ifdef ESP32
    #include <WiFi.h>
#else
    #include <ESP8266WiFi.h>
#endif

#include <MovingAverageFloat.h>

#define EEPROM_SIZE 4

#ifndef STASSID
#define STASSID "XXXXX"
#define STAPSK  "XXXXX"
#endif

const char* ssid = STASSID;
const char* password = STAPSK;

const char* clientId = "incubator";
const char* mqttServer = "192.168.178.91";
const int mqttPort = 1883;

const char* topicTargetTemp = "incubator/target_temp";
const char* topicCommand = "incubator/cmd";
const char* topicReset = "incubator/reset";
const char* topicKP = "incubator/kp";
const char* topicKD = "incubator/kd";
const char* topicKI = "incubator/ki";
const char* topicOn = "incubator/on";

const IPAddress ip(192, 168, 178, 97);
const IPAddress fritzbox(192, 168, 178, 1);
const IPAddress subnet(255, 255, 255, 0);

WiFiClient espClient;
PubSubClient mqttClient(espClient);

const int oledSDA = 0;
const int oledSCL = 2;
const int heaterPin = 5;
const int tempSensorPin= 4;

#ifdef ESP32
    const int ledChannel = 0;
    const int freq = 2000;
    const int resolution = 8;
#endif

int onState = 1;

double lastHeaterVal = 0;
double heaterVal = 0;
double targetTemp = 34.00;
double measuredTemp = 0;
double currentTemp = 0;
double lastTemp = 0;
int heaterPercent = 0;

const int oneWireBus = tempSensorPin;
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

MovingAverageFloat <8> filter;

// PID settings
#define OUTPUT_MIN 0

#ifdef ESP32
    #define OUTPUT_MAX 255
#else
    #define OUTPUT_MAX 1023
#endif

float kp = 850.0;
float kd = 250.0;
float ki = 0.1;

unsigned long previousMillisHistory = 0;
unsigned long historyInterval = 4000;

unsigned long previousMillisTemp = 0;
unsigned long tempInterval = 760;

unsigned long previousMillisFanCheck = 0;
unsigned long fanCheckInterval = 4000;

AutoPID heaterPID(&currentTemp, &targetTemp, &heaterVal, OUTPUT_MIN, OUTPUT_MAX, kp, ki, kd);
SSD1306Wire display(0x3c, oledSDA, oledSCL);

const uint8_t displayWidth = 128;
const uint8_t displayHeight = 64;
const int tempHistorySize = displayWidth - 2;
const uint8_t graphHeight = 17;

int tempHistory[tempHistorySize];

void publish(char* topic, double value, bool retained=false) {
	Serial.print("publish: ");
	Serial.println(value);

	String tempStr = String(value);
	char buffer[50];
	tempStr.toCharArray(buffer, tempStr.length() + 1);

	mqttClient.publish(topic, buffer, retained);
	mqttClient.loop();
}

void publish(char* topic, int value, bool retained=false) {
	Serial.print("publish: ");
	Serial.println(value);

	String tempStr = String(value);
	char buffer[50];
	tempStr.toCharArray(buffer, tempStr.length() + 1);

	mqttClient.publish(topic, buffer, retained);
	mqttClient.loop();
}

void publish(char* topic, char* value, bool retained=false) {
	Serial.print("publish: ");
	Serial.println(value);

	String tempStr = String(value);
	char buffer[50];
	tempStr.toCharArray(buffer, tempStr.length() + 1);

	mqttClient.publish(topic, buffer, retained);
	mqttClient.loop();
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    Serial.print("receiving: ");
    Serial.print(topic);
    Serial.print(' ');
    Serial.println((char*)payload);

    // publish((char*)"incubator/_echo/rcv", (char*)payload);
    // mqttClient.loop();

	if(strcmp(topic, topicTargetTemp) == 0) {
	    targetTemp = atof((char*)payload);  
        if(targetTemp > 50) {
            return;
        }
        EEPROM.write(0, targetTemp);
        EEPROM.commit();
		publish((char *)"incubator/_echo/target_temp", targetTemp);
	}

	else if(strcmp(topic, topicOn) == 0) {
	    onState = atoi((char*)payload);
		publish((char *)"incubator/_echo/on", onState);

		if(onState == 1) {
			heaterPID.run();
		} else {
			heaterPID.stop();
            #ifdef ESP32
                ledcWrite(ledChannel, 0);
            #else
                analogWrite(heaterPin, 0);
            #endif
		}
	}

	else if(strcmp(topic, topicKP) == 0) {
	    kp = atof((char*)payload);
		publish((char *)"incubator/_echo/kp", kp);
		heaterPID.setGains(kp, ki, kd);
	}

	else if(strcmp(topic, topicKD) == 0) {
	    kd = atof((char*)payload);
		publish((char *)"incubator/_echo/kd", kd);
		heaterPID.setGains(kp, ki, kd);
	}

	else if(strcmp(topic, topicKI) == 0) {
	    ki = atof((char*)payload);
		publish((char *)"incubator/_echo/ki", ki);
		heaterPID.setGains(kp, ki, kd);
	}

	else if(strcmp(topic, topicReset) == 0) {
		publish((char *)"incubator/_echo/reset", (char*) "reset");
	    ESP.restart();
	}

	// else if(strcmp(topic, topicCommand) == 0) {
	// 	publish((char *)"incubator/_echo/cmd", (char*)payload);
	// 	if(strcmp((char*)payload, "reset") == 0) {
	//     	ESP.restart();
    //     }
	// }
    // mqttClient.loop();
}

void reconnect() {
    while (!mqttClient.connected()) {
        Serial.print("Attempting MQTT connection...");
        if (mqttClient.connect(clientId)) {
            Serial.println("connected");
        } else {
            Serial.print("failed, rc=");
            Serial.print(mqttClient.state());
            delay(1000);
        }
    }
    mqttClient.subscribe(topicTargetTemp);
    mqttClient.subscribe(topicCommand);
    mqttClient.subscribe(topicReset);
    mqttClient.subscribe(topicKP);
    mqttClient.subscribe(topicKD);
    mqttClient.subscribe(topicKI);
    mqttClient.subscribe(topicOn);
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

void setup() {
    EEPROM.begin(EEPROM_SIZE);
    targetTemp = EEPROM.read(0);

    Serial.begin(115200);

    Serial.println("Booting");
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.waitForConnectResult() != WL_CONNECTED) {
        Serial.println("Connection Failed! Rebooting...");
        delay(5000);
        ESP.restart();
    }
    setupOTA();
    Serial.println("Ready");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    sensors.begin();

    // prepare the display
    display.init();
    display.clear();
    display.flipScreenVertically();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(Dialog_plain_13);
    display.drawString(0, 0, "Let's make tempeh!");
    display.display();

#ifdef ESP32
    ledcSetup(ledChannel, freq, resolution);
    ledcAttachPin(heaterPin, ledChannel);
#else
    pinMode(heaterPin, OUTPUT);
#endif
    
    heaterPID.setBangBang(5); // run the heater full bang until 5°c below targetTemperature
    heaterPID.setTimeStep(1000); // run pid calculation every n seconds

    Serial.print("Found ");
    Serial.print(sensors.getDeviceCount(), DEC);
    Serial.println(" devices.");

    mqttClient.setServer(mqttServer, mqttPort);
    mqttClient.setCallback(mqttCallback);

}

void measureTemperature() {
    sensors.requestTemperaturesByIndex(0);
    measuredTemp = sensors.getTempCByIndex(0);
    if(measuredTemp >= 50) {
        ESP.restart();
    }

    Serial.println(measuredTemp);

	if(measuredTemp > 0) {
		currentTemp = measuredTemp;
		filter.add(currentTemp);
		currentTemp = filter.get();
        
        display.setColor(BLACK);
        display.fillRect(0, 0, display.getWidth(), 16);
        display.setColor(WHITE);

        char tempStr[7];
        sprintf(tempStr, "%2.2f°/%d°@%d%%", currentTemp, int(targetTemp), heaterPercent);
        display.drawString(0, 0, tempStr);

        display.display();
	}

    if(currentTemp != lastTemp) {
        Serial.println(currentTemp);

		publish((char *)"incubator/temp", currentTemp);
        lastTemp = currentTemp;
    }
}

void checkFan() {

}

void loop() {
    ArduinoOTA.handle();

    if (!mqttClient.connected()) {
        reconnect();
    }
    mqttClient.loop();

    unsigned long currentMillis = millis();
    
    if(currentMillis - previousMillisTemp > tempInterval) {
        previousMillisTemp = currentMillis;
        measureTemperature();
    }

    if(currentMillis - previousMillisFanCheck > fanCheckInterval) {
        previousMillisFanCheck= currentMillis;
        checkFan();
    }

    if(currentMillis - previousMillisHistory > historyInterval) {
        previousMillisHistory = currentMillis;
        memcpy(tempHistory, &tempHistory[1], sizeof(tempHistory) - sizeof(int));
        int lowpass = 15;
        tempHistory[tempHistorySize - 1] = (currentTemp - lowpass) * graphHeight / (targetTemp - lowpass);

        // clear the draw area
        display.setColor(BLACK);
        display.fillRect(0, graphHeight, display.getWidth(), display.getHeight() - graphHeight);
        display.setColor(WHITE);
        display.drawRect(0, graphHeight, display.getWidth(),  display.getHeight() - graphHeight);

        // draw history graph
        for(uint8_t i = 0; i < tempHistorySize; i++) {
            int y = tempHistory[i];
            display.setPixel(i + 1, 18 + (64 - y));
        }
        display.display();
    }

    heaterPID.run();

    if(heaterVal != lastHeaterVal) {
		lastHeaterVal = heaterVal;
        heaterPercent = heaterVal * 100 / OUTPUT_MAX;

		publish((char *)"incubator/heater", heaterPercent);

        #ifdef ESP32
            ledcWrite(ledChannel, heaterVal);
        #else
		    analogWrite(heaterPin, heaterVal);
        #endif
    }
}
