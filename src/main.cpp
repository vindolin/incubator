#include <IotBase.h>

// PIN 4/5 would have to be free for I2C
#ifdef USE_HTU21DF
#include <Adafruit_HTU21DF.h>
#endif

#include <DallasTemperature.h>
#include <AutoPID.h>
#include <EEPROM.h>
#include <SSD1306Wire.h>
#include <MovingAverageFloat.h>

#include <font.h>  // Dialog_plain_13

#ifdef ESP32
    const uint8_t buildinLED = 2;
    const uint8_t oledSDAPin = 4;
    const uint8_t oledSCLPin = 15;
    const uint8_t heaterPin = 22;
    const uint8_t tempSensorPin = 26;

    const uint8_t fanTachoPin = 10;
    const uint8_t servoPin= 19;
#else
    const uint8_t buildinLED = 2;
    const uint8_t oledSDAPin = 13;
    const uint8_t oledSCLPin = 12;
    const uint8_t heaterPin = 4;
    const uint8_t tempSensorPin = 5;

    const uint8_t fanTachoPin = 14;
    const uint8_t servoPin = 10;
#endif


#define SECONDS 1000

const char* CLIENT_ID = "incubator";

char topicSubTargetTemp[] = "/set/target_temp";
char topicSubHeatKp[] = "/set/heat_kp";
char topicSubHeatKd[] = "/set/heat_kd";
char topicSubHeatKi[] = "/set/heat_ki";

char topicSubCoolKp[] = "/set/cool_kp";
char topicSubCoolKd[] = "/set/cool_kd";
char topicSubCoolKi[] = "/set/cool_ki";

char topicSubOnState[] = "/set/on_state";

char* subTopics[] = {
    topicSubTargetTemp,

    topicSubHeatKp,
    topicSubHeatKd,
    topicSubHeatKi,

    topicSubCoolKp,
    topicSubCoolKd,
    topicSubCoolKi,

    topicSubOnState,
};
char commands[] = "set/target_temp";

char topicPubOnState[] = "/on_state";
char topicPubTargetTemp[] = "/target_temp";

char topicPubHeatKp[] = "/heat_kp";
char topicPubHeatKd[] = "/heat_kd";
char topicPubHeatKi[] = "/heat_ki";

char topicPubCoolKp[] = "/cool_kp";
char topicPubCoolKd[] = "/cool_kd";
char topicPubCoolKi[] = "/cool_ki";

char topicPubTemp[] = "/temp";
char topicPubRpm[] = "/rpm";

char topicPubHeater[] = "/heater";
char topicPubCooler[] = "/cooler";

const uint8_t tempResolution = 12;

#ifdef USE_HTU21DF
char topicPubHumidity[] = "/humidity";
char topicPubTemp2[] = "/temp2";
Adafruit_HTU21DF htu = Adafruit_HTU21DF();
float humidity;
float temp2;
#endif

const int pwmFreq = 20;

#ifdef ESP32
    const int ledChannel = 0;
    const int resolution = 8;
#endif

const uint8_t shutdownTemperature = 50;

double lastHeaterVal = 0;
double heaterVal = 0;

double lastServoVal = 0;
double servoVal = 0;

uint settingsAddr = 32; // behind IotBase
double measuredTemp = 0;
double currentTemp = 0;
double currentOverTemp = 0;
double lastTemp = 0;
uint8_t heaterPercent = 0;
uint8_t coolerPercent = 0;
uint8_t welcomeCleared = false;

struct {
    double targetTemp = 34.00;
    bool onState = false;
} settings;

OneWire oneWire(tempSensorPin);
DallasTemperature sensors(&oneWire);

MovingAverageFloat<8> filter;

// PID settings
#define OUTPUT_MIN 0
#ifdef ESP32
    #define OUTPUT_MAX 255
#else
    #define OUTPUT_MAX 1023
#endif
#define SERVO_MIN 0
#define SERVO_MAX 180

bool wifiIconVisible = false;
static unsigned char wifiIconBits[] = {
   0x7c, 0x00, 0x82, 0x00, 0x39, 0x01, 0x44, 0x00, 0x92, 0x00, 0x38, 0x00,
   0x38, 0x00, 0x10, 0x00
};

float heat_kp = 850.0;
float heat_kd = 250.0;
float heat_ki = 0.1;

float cool_kp = -850.0;
float cool_kd = -250.0;
float cool_ki = -0.1;

void flashWifi();
void measureTemp();
void measureHumidity();
void checkFan();
void drawGraph();
void reportStatus();
void updateHeater();
void updateCooler();

const uint16_t measureTempInterval = 1 * TASK_SECOND;
Task taskMeasureTemp(measureTempInterval, TASK_FOREVER, measureTemp);

#ifdef USE_HTU21DF
Task taskMeasureHumidity(30 * TASK_SECOND, TASK_FOREVER, measureHumidity);
#endif

uint8_t checkFanInterval = 1;
uint16_t minFanRpm = 2000;
uint16_t fanRpm = 0;

Task taskFlashWifi(1 * TASK_SECOND, TASK_FOREVER, flashWifi);
Task taskCheckFan(checkFanInterval * TASK_SECOND, TASK_FOREVER, checkFan);
Task taskDrawGraph(5 * TASK_SECOND, TASK_FOREVER, drawGraph);
Task taskReportStatus(10 * TASK_SECOND, TASK_FOREVER, reportStatus);
Task taskUpdateHeater(1 * TASK_SECOND, TASK_FOREVER, updateHeater);
Task taskUpdateCooler(1 * TASK_SECOND, TASK_FOREVER, updateCooler);

AutoPID heaterPID(&currentTemp, &settings.targetTemp, &heaterVal, OUTPUT_MIN, OUTPUT_MAX, heat_kp, heat_ki, heat_kd);
AutoPID coolerPID(&currentTemp, &settings.targetTemp, &servoVal, SERVO_MIN, SERVO_MAX, cool_kp, cool_ki, cool_kd);

SSD1306Wire display(0x3c, oledSDAPin, oledSCLPin);

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
bool measureTempMode = true;
bool validMeasurement = false;

volatile uint16_t halfRevolutions = 0;

IRAM_ATTR void onTacho() {
    halfRevolutions++;
}

void setOnState(bool state) {
    if(state) {
        heaterPID.run();
        coolerPID.run();
    } else {
        heaterPID.stop();
        coolerPID.stop();
    }
    settings.onState = state;
    publish(topicPubOnState, settings.onState ? "ON" : "OFF", true);
}

void publishTemp() {
    Serial.print("temp: ");
    Serial.println(currentTemp);
    publish(topicPubTemp, currentTemp, true);
}

void publishHeater() {
    publish(topicPubHeater, heaterPercent, true);
}

void publishCooler() {
    publish(topicPubCooler, coolerPercent, true);
}

void initOled() {
    pinMode(16,OUTPUT);
    digitalWrite(16, LOW);
    delay(50);
    digitalWrite(16, HIGH); // while OLED is running, must set GPIO16 in high、
}

void setHeater(int value) {
#ifdef ESP32
    ledcWrite(ledChannel, value);
#else
    analogWrite(heaterPin, value);
#endif
    publishHeater();
}

void setCooler(int value) {
    // todo servo.write(value);
    publishCooler();
}

void updateHeater() {
    if(settings.onState && validMeasurement && fanRpm >= minFanRpm) {
        if(heaterVal != lastHeaterVal) {
            lastHeaterVal = heaterVal;
            publish("/heater_raw", heaterVal, true);
            setHeater(heaterVal);
            heaterPercent = heaterVal * 100 / OUTPUT_MAX;
        }

    // safety feature
    } else {
        heaterVal = 0;
        heaterPercent = 0;
        lastHeaterVal = 0;
        setHeater(0);
    }
}

void updateCooler() {
    publish("/cooler_raw", servoVal);

    if(settings.onState && validMeasurement) {
        if(servoVal != lastServoVal) {
            lastServoVal = servoVal;
            publish("/cooler_raw", servoVal, true);
            setCooler(servoVal);
            coolerPercent = servoVal * 100 / SERVO_MAX;
        }

    // safety feature
    } else {
        servoVal = 0;
        coolerPercent = 0;
        lastServoVal = 0;
        setCooler(0);
    }
}

void reportStatus() {
    publish(topicPubRpm, fanRpm, true);
    publishTemp();
    publishHeater();
}

void measureTemp() {
    // start a new measurement and in the next run get the result
    if(measureTempMode) {
        taskMeasureTemp.setInterval(measureTempInterval);
        sensors.requestTemperaturesByIndex(0);
        measureTempMode = false;

    // read the result
    } else {
        taskMeasureTemp.setInterval(0);

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

            if(currentTemp > settings.targetTemp) {
                currentOverTemp = currentTemp - settings.targetTemp;
            } else {
                currentOverTemp = 0;
            }

            display.setColor(BLACK);
            display.fillRect(0, 0, display.getWidth() - 9, 16);
            display.setColor(WHITE);

            sprintf(tmpBuffer, "%2.2f°/%d°@%d%%", currentTemp, int(settings.targetTemp), heaterPercent);
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

#ifdef USE_HTU21DF
void measureHumidity() {
    humidity = htu.readHumidity();
    if(humidity > 0.1) {
        publish(topicPubHumidity, humidity, true);
    }

    temp2 = htu.readTemperature();
    if(temp2 > 0.1) {
        publish(topicPubTemp2, temp2, true);
    }
}
#endif

void drawGraph() {
    memmove(tempHistory, &tempHistory[1], sizeof(tempHistory) - sizeof(int));
    if(validMeasurement) {
        tempHistory[tempHistorySize - 1] = (int)currentTemp;
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

void firstMqttCallback() {
    taskReportStatus.enable();
    publish(topicPubTargetTemp, settings.targetTemp, true);
    publish(topicPubOnState, settings.onState ? "ON" : "OFF", true);
    publish(topicPubHeatKp, heat_kp);
    publish(topicPubHeatKd, heat_kd);
    publish(topicPubHeatKi, heat_ki);
}

void commitSettings() {
    EEPROM.put(settingsAddr, settings);
    EEPROM.commit();
}

void mqttCallback(char* topic, byte* payload) {
  Serial.print("receiving: ");
    Serial.print(topic);
    Serial.print(' ');
    Serial.println((char*)payload);

    // publish((char*)"incubator/_echo/rcv", (char*)payload);

    if(compareTopic(topic, topicSubTargetTemp)) {
        settings.targetTemp = atof((char*)payload);
        if(settings.targetTemp > 45) {
            return;
        } else if(settings.targetTemp < 1) {
            setOnState(false);
        } else {
            setOnState(true);
        }

        commitSettings();
        publish(topicPubTargetTemp, settings.targetTemp, true);
    }

    else if(compareTopic(topic, topicSubOnState)) {
        bool state = (strcmp((char*)payload, "ON") == 0);
        setOnState(state);
        commitSettings();
        publish(topicPubOnState, settings.onState ? "ON" : "OFF", true);
    }

    else if(compareTopic(topic, topicSubHeatKp)) {
        heat_kp = atof((char*)payload);
        heaterPID.setGains(heat_kp, heat_ki, heat_kd);
        publish(topicPubHeatKp, heat_kp);
    }
    else if(compareTopic(topic, topicSubHeatKd)) {
        heat_kd = atof((char*)payload);
        heaterPID.setGains(heat_kp, heat_ki, heat_kd);
        publish(topicPubHeatKd, heat_kd);
    }
    else if(compareTopic(topic, topicSubHeatKi)) {
        heat_ki = atof((char*)payload);
        heaterPID.setGains(heat_kp, heat_ki, heat_kd);
        publish(topicPubHeatKi, heat_ki);
    }

    else if(compareTopic(topic, topicSubCoolKp)) {
        cool_kp = atof((char*)payload);
        coolerPID.setGains(cool_kp, cool_ki, cool_kd);
        publish(topicPubCoolKp, cool_kp);
    }
    else if(compareTopic(topic, topicSubCoolKd)) {
        cool_kd = atof((char*)payload);
        coolerPID.setGains(cool_kp, cool_ki, cool_kd);
        publish(topicPubCoolKd, cool_kd);
    }
    else if(compareTopic(topic, topicSubCoolKi)) {
        cool_ki = atof((char*)payload);
        coolerPID.setGains(cool_kp, cool_ki, cool_kd);
        publish(topicPubCoolKi, cool_ki);
    }

}

void connectCallback() {}

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
    setCooler(0);

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

    Serial.begin(115200);

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

    coolerPID.setTimeStep(1000); // run pid calculation every n seconds

    IotBase_setup(
        connectCallback,
        subTopics,
        sizeof(subTopics) / sizeof(subTopics[0]),
        mqttCallback,
        firstMqttCallback
    );

    EEPROM.get(settingsAddr, settings);

    runner.addTask(taskMeasureTemp);
    taskMeasureTemp.enable();

    #ifdef USE_HTU21DF
    runner.addTask(taskMeasureHumidity);
    taskMeasureHumidity.enable();
    taskMeasureHumidity.forceNextIteration();
    #endif

    runner.addTask(taskFlashWifi);
    taskFlashWifi.enable();

    runner.addTask(taskCheckFan);
    taskCheckFan.enable();

    runner.addTask(taskDrawGraph);
    taskDrawGraph.enable();

    runner.addTask(taskReportStatus);

    runner.addTask(taskUpdateHeater);
    taskUpdateHeater.enable();

    runner.addTask(taskUpdateCooler);
    taskUpdateCooler.enable();

}

void loop() {
    IotBase_loop();
    heaterPID.run();
}
