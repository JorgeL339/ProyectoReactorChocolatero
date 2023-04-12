#include <Arduino.h>
#include <ContinuousStepper.h>
#include <AutoPID.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "HX711.h"
#include <SPI.h>
#include <TFT_eSPI.h>

//Pines Para la Pantalla
#define TFT_CLK 32
#define TFT_MOSI 33
#define TFT_CS 27
#define TFT_DC 26
#define TFT_RST 25

TFT_eSPI  tft = TFT_eSPI();

constexpr uint8_t RPM_PIN = 27;
constexpr uint8_t STEP_PIN = 5;
constexpr uint8_t DIRECTION_PIN = 17;
constexpr uint8_t ENABLE_PIN = 16;
constexpr uint8_t RELAY_PIN = 13;
constexpr uint8_t TEMP_SENSOR_PIN = 14;
constexpr uint8_t LOADCELL_DOUT_PIN = 19;
constexpr uint8_t LOADCELL_SCK_PIN = 18;
const int BAR_WIDTH = 120;
const int BAR_HEIGHT = 10;
const int BAR_X = 10;
const int BAR_Y = 200;
static int dotIndex = 0;
static const char* dots[] = {"   ", ".  ", ".. ", "..."};
constexpr float INTERRUPTS_PER_REVOLUTION = 12;
constexpr double KP = 0.12;
constexpr double KI = 0.0003;
constexpr double KD = 0;
constexpr int PULSE_WIDTH = 10;
double SET_POINT = 40.0;

HX711 scale;
OneWire oneWire(TEMP_SENSOR_PIN);
DallasTemperature tempSensor(&oneWire);
ContinuousStepper stepper;
TaskHandle_t temperatureTaskHandle;

volatile int rpmCount = 0;
unsigned long lastMillis = 0;
unsigned int rpm = 0;
double temperature = 0.0;
bool relayState = false;
int StepperSpeed = 400; // initial speed of the stepper motor

AutoPIDRelay pid(&temperature, &SET_POINT, &relayState, PULSE_WIDTH, KP, KI, KD);

void rpmInterrupt() {
  rpmCount++;
}

void measureTemperatureTask(void * parameter) {
  for (;;) {
    Serial.print("Measuring temperature...");
    detachInterrupt(digitalPinToInterrupt(RPM_PIN));
    rpm = (rpmCount * 60) / INTERRUPTS_PER_REVOLUTION;
    rpmCount = 0;
    lastMillis = millis();
    attachInterrupt(digitalPinToInterrupt(RPM_PIN), rpmInterrupt, RISING);
    
    tempSensor.requestTemperatures();
    temperature = tempSensor.getTempCByIndex(0);
    pid.run();
    digitalWrite(RELAY_PIN, relayState);
    
    int load = -scale.get_units(1);
    Serial.print("RPM: ");
    Serial.print(rpm);
    Serial.print(" | ");
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.print(" | ");
    Serial.print("Relay state: ");
    Serial.print(digitalRead(RELAY_PIN));
    Serial.print(" | ");
    Serial.print("Load: ");
    Serial.println(load);

  
    tft.fillScreen( 0xFFFF);
    tft.setCursor(0, 0);
    tft.print("Temp Actual:  ");
    tft.print(temperature);
    tft.print((char)247);
    tft.print("C");
    tft.println();
    tft.print("Temp Deseada: ");
    tft.print(SET_POINT);
    tft.print((char)247);
    tft.print("C");
    tft.println();
    tft.print("Relay State: ");
    tft.print(relayState ? "On" : "Off");
    tft.println();
    tft.print("Peso: ");
    tft.print(load);
    tft.print("g");
    tft.println();
    tft.print("RPM: ");
    tft.print(rpm);
    tft.println();

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting...");
  tft.begin();
  tft.setRotation(3);
  tft.setTextColor(0x0000);
  tft.setTextSize(2);

  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(903.405);
  scale.tare(10);

  pid.setTimeStep(100);

  stepper.begin(STEP_PIN, DIRECTION_PIN);
  stepper.spin(StepperSpeed); // start with initial speed

  pinMode(RPM_PIN, INPUT_PULLUP);
  pinMode(RELAY_PIN, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(RPM_PIN), rpmInterrupt, RISING);

  tempSensor.begin();
  lastMillis = millis();
  rpmCount = 0;

  xTaskCreatePinnedToCore(measureTemperatureTask, "measureTemperatureTask", 10000, NULL, 1, &temperatureTaskHandle, 0); // create temperature measurement task on core 0
}

void loop() {
  stepper.loop();
}


