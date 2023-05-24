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
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

// Definitions
#define BLYNK_TEMPLATE_ID "TMPL2Eu7zlL6E"
#define BLYNK_TEMPLATE_NAME "Quickstart Template"
#define BLYNK_AUTH_TOKEN "tKhKkJz7osel4Exbe-5KrvIZuD4kRog0"

#define TFT_CS 19  // Chip select control pin
#define TFT_DC 18  // Data Command control pin
#define TFT_RST 5  // Reset pin (could connect to RST pin)
#define TFT_MOSI 17
#define TFT_SCLK 16

// Constants
constexpr uint8_t RPM_PIN = 35;
constexpr uint8_t STEP_PIN = 23;
constexpr uint8_t DIRECTION_PIN = 2;
constexpr uint8_t STEP_PIN_bomba = 22;
constexpr uint8_t DIRECTION_PIN_bomba = 2;
constexpr uint8_t RELAY_PIN = 5;
constexpr uint8_t TEMP_SENSOR_PIN = 21;
constexpr uint8_t LOADCELL_DOUT_PIN = 32;
constexpr uint8_t LOADCELL_SCK_PIN = 33;
constexpr float INTERRUPTS_PER_REVOLUTION = 12;
constexpr double KP = 0.12;
constexpr double KI = 0.0003;
constexpr double KD = 0;
constexpr int PULSE_WIDTH = 10;
#define OUTPUT_MIN 0
#define OUTPUT_MAX 700
const int BAR_WIDTH = 120;
const int BAR_HEIGHT = 10;
const int BAR_X = 10;
const int BAR_Y = 200;
const int S1 = 13; // CLK or A
const int S2 = 12; // DT or B
const int KEY = 14; // Push button

// Global Variables
TFT_eSPI tft = TFT_eSPI();
HX711 scale;
OneWire oneWire(TEMP_SENSOR_PIN);
DallasTemperature tempSensor(&oneWire);
ContinuousStepper stepper;
ContinuousStepper bomba;
TaskHandle_t temperatureTaskHandle;

// Variables with initial values
double SET_POINT = 40.0;
double SET_PESO = 0;
volatile int rpmCount = 0;
unsigned long lastMillis = 0;
unsigned int rpm = 0;
double temperature = 0.0;
double peso, StepperSpeed_bomba;
bool relayState = false;
int SET_MOTOR = 0;
int StepperSpeed =  0;
volatile int encoder_count = 0;
static int encoder_H2OPeso = 0;
static int encoder_RPMmotor = 0;
static int encoder_TEMPh2o = 0;
bool direction; // true: clockwise, false: counterclockwise
bool button_pressed = false;
static const char* dots[] = {"   ", ".  ", ".. ", "..."};
static int dotIndex = 0;
int RPM_Objetivo = 0;
int WaterPeso = 0;

// Create PID objects
AutoPIDRelay pid(&temperature, &SET_POINT, &relayState, PULSE_WIDTH, KP, KI, KD);
AutoPID pid2(&peso, &SET_PESO, &StepperSpeed_bomba, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);

// Blynk Setup
char auth[] = "tKhKkJz7osel4Exbe-5KrvIZuD4kRog0";  // Paste auth token you copied
char ssid[] = "ainh007"; // Enter your WiFi name
char pass[] = "12345678"; // Enter WiFi password

// Interrupts
void rpmInterrupt() { rpmCount++; }

// Blynk functions
BLYNK_WRITE(V6) { RPM_Objetivo = param.asInt(); }
BLYNK_WRITE(V8) { WaterPeso = param.asInt(); }

// Other functions
void measureTemperatureTask(void * parameter) {
  for (;;) {

    //Serial.print("Measuring temperature...");
    detachInterrupt(digitalPinToInterrupt(RPM_PIN));
    rpm = (rpmCount*60*60) / (INTERRUPTS_PER_REVOLUTION*200);
    rpmCount = 0;
    lastMillis = millis();
    attachInterrupt(digitalPinToInterrupt(RPM_PIN), rpmInterrupt, RISING);


    
    tempSensor.requestTemperatures();
    temperature = tempSensor.getTempCByIndex(0);

    //digitalWrite(RELAY_PIN, relayState);
    
    peso = scale.get_units(1);

    tft.fillScreen( 0xFFFF);
    tft.setCursor(0, 0);
    tft.print("Temp Actual:  ");
    tft.print(temperature);
    tft.print((char)247);
    tft.print("C");
    Blynk.virtualWrite(V5, temperature);

    tft.println();
    tft.print("Temp Deseada: ");
    tft.print(SET_POINT);
    tft.print((char)247);
    tft.print("C");
    tft.println();
    tft.print("Relay State: ");
    tft.print(relayState ? "On" : "Off");
    tft.println();
    tft.print("Velocidad Bomba: ");
    tft.print(StepperSpeed_bomba);
    Blynk.virtualWrite(V4, StepperSpeed_bomba);
    tft.println();
    tft.println();
    tft.println();

    tft.print("Peso Objetivo: ");
    tft.print(SET_PESO);
    tft.print("g");
    tft.println();
    tft.print("Peso Actual: ");
    tft.print(peso);
    tft.print("g");
    Blynk.virtualWrite(V7, peso);
    tft.println();
    tft.println();
    tft.print("RPM Objetivo: ");
    tft.print(SET_MOTOR);
    tft.print("RPM");
    tft.println();
    tft.print("RPM Actual: ");
    tft.print(rpm);
    tft.print("RPM");
    Blynk.virtualWrite(V1, rpm);

    SET_MOTOR = RPM_Objetivo;
    SET_PESO = WaterPeso;






    
    Blynk.run();
    vTaskDelay(pdMS_TO_TICKS(780));
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting...");

  // TFT Setup
  tft.begin();
  tft.setRotation(3);
  tft.setTextColor(0x0000);
  tft.setTextSize(2);

  // Pin Modes
  pinMode(S1, INPUT_PULLUP);
  pinMode(S2, INPUT_PULLUP);
  pinMode(KEY, INPUT_PULLUP);
  pinMode(RPM_PIN, INPUT_PULLUP);
  pinMode(RELAY_PIN, OUTPUT);


  // Scale Setup
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(903.405);
  scale.tare(10);

  // PID Setup
  pid.setTimeStep(100);

  // Stepper and bomba setup
  stepper.begin(STEP_PIN, DIRECTION_PIN);
  stepper.spin(round(5.303*(SET_MOTOR) - 3.0303)); // start with initial speed
  bomba.begin(STEP_PIN_bomba, DIRECTION_PIN_bomba);
  bomba.spin(int(StepperSpeed_bomba)); // start with initial speed
  pid2.setBangBang(1);

  // Temperature sensor setup
  tempSensor.begin();
  lastMillis = millis();
  rpmCount = 0;

  // Blynk setup
  Blynk.begin(auth, ssid, pass);

  // Create temperature measurement task on core 0
  xTaskCreatePinnedToCore(measureTemperatureTask, "measureTemperatureTask", 10000, NULL, 1, &temperatureTaskHandle, 0); 
}

void loop() {
  // PID control
  pid2.run();

  // Stepper control
  stepper.loop();
  bomba.loop();
  bomba.spin(int(StepperSpeed_bomba));
  stepper.spin(round(5.303 * (SET_MOTOR) - 3.0303)); // Update stepper speed

  // Blynk run
  Blynk.run();
}
