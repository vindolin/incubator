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
char topicSubKp[] = "/set/kp";
char topicSubKd[] = "/set/kd";
char topicSubKi[] = "/set/ki";
char topicSubOn[] = "/set/on";

char topicSubCommand[] = "/cmd";

char* subTopics[] = {
    topicSubTargetTemp,
    topicSubKp,
    topicSubKd,
    topicSubKi,
    topicSubOn,

    topicSubCommand,
};
char commands[] = "openValve, closeValve, setTargetMoisture, setOpenDuration, setCheckInterval, measure";

char topicPubEchoOnState[] = "/on_state";
char topicPubEchoTargetTemp[] = "/target_temp";
char topicPubEchoKp[] = "/kp";
char topicPubEchoKd[] = "/kd";
char topicPubEchoKi[] = "/ki";

char topicPubTemp[] = "/temp";
char topicPubRpm[] = "/rpm";
char topicPubHeater[] = "/heater";

char topicPubHumidity[] = "/humidity";

const uint8_t tempResolution = 12;

#ifdef USE_HTU21DF
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

bool onState = true;

const uint8_t shutdownTemperature = 50;
double lastHeaterVal = 0;
double heaterVal = 0;
uint settingsAddr = 32; // behind IotBase
double measuredTemp = 0;
double currentTemp = 0;
double lastTemp = 0;
uint8_t lastHeaterPercent = 0;
uint8_t heaterPercent = 0;
uint8_t welcomeCleared = false;

struct {
    double targetTemp = 34.00;
} settings;

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
   0x38, 0x00, 0x10, 0x00
};

float kp = 850.0;
float kd = 250.0;
float ki = 0.1;

uint reportCounter = 0;

void flashWifi();
void measureTemp();
void measureHumidity();
void checkFan();
void drawGraph();
void reportStatus();
void updateHeater();

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

AutoPID heaterPID(&currentTemp, &settings.targetTemp, &heaterVal, OUTPUT_MIN, OUTPUT_MAX, kp, ki, kd);
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
        onState = true;
    } else {
        heaterPID.stop();
        heaterVal = 0;
        onState = false;
    }
}

void publishTemp() {
    Serial.print("temp: ");
    Serial.println(currentTemp);
    publish(topicPubTemp, currentTemp, true);
}

void publishHeater() {
    publish(topicPubHeater, heaterPercent, true);
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

void reportStatus() {
    publish(topicPubRpm, fanRpm, true);
    publishTemp();
    publishHeater();

    if(reportCounter++ % 100 == 0) {
        publish("target_temp", settings.targetTemp);
    }
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

    publish(topicPubEchoTargetTemp, settings.targetTemp);
    publish(topicPubEchoOnState, onState);
    publish(topicPubEchoKp, kp);
    publish(topicPubEchoKd, kd);
    publish(topicPubEchoKi, ki);
}

void mqttCallback(char* topic, byte* payload) {
  Serial.print("receiving: ");
    Serial.print(topic);
    Serial.print(' ');
    Serial.println((char*)payload);

    // publish((char*)"incubator/_echo/rcv", (char*)payload);

    if(compareTopic(topic, topicSubTargetTemp)) {
        settings.targetTemp = atof((char*)payload);
        if(settings.targetTemp > 50) {
            return;
        } else if(settings.targetTemp < 1) {
            setOnState(false);
        } else {
            setOnState(true);
        }

        EEPROM.put(settingsAddr, settings);
        EEPROM.commit();
        publish(topicPubEchoTargetTemp, settings.targetTemp);
    }

    else if(compareTopic(topic, topicSubOn)) {
        bool state = atoi((char*)payload) > 0;
        setOnState(state);
        publish(topicPubEchoOnState, onState);
    }

    else if(compareTopic(topic, topicSubKp)) {
        kp = atof((char*)payload);
        heaterPID.setGains(kp, ki, kd);
        publish(topicPubEchoKp, kp);
    }

    else if(compareTopic(topic, topicSubKd)) {
        kd = atof((char*)payload);
        heaterPID.setGains(kp, ki, kd);
        publish(topicPubEchoKd, kd);
    }

    else if(compareTopic(topic, topicSubKi)) {
        ki = atof((char*)payload);
        heaterPID.setGains(kp, ki, kd);
        publish(topicPubEchoKi, ki);
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

    // TODO
    // pinMode(fanTachoPin, INPUT_PULLUP);
    // attachInterrupt(digitalPinToInterrupt(fanTachoPin), tachoSignal, RISING);

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

}

void loop() {
    IotBase_loop();
    heaterPID.run();
}
