// These are libraries needed to run the program. They provide functionality for controlling a stepper motor, PID control, temperature sensing, FreeRTOS (real-time operating system), load cell interfacing, SPI communication, TFT screen controlling, and WiFi/Blynk communication.

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

// Here the program defines constants and variables. The defined constants are pin numbers for the microcontroller, some PID parameters, and some display parameters. The variables are related to PID control, stepper motor control, temperature and weight measurement, and encoder control.

// Definitions for Blynk Template ID, name, and Auth Token
#define BLYNK_TEMPLATE_ID "TMPL2Eu7zlL6E"
#define BLYNK_TEMPLATE_NAME "Quickstart Template"
#define BLYNK_AUTH_TOKEN "tKhKkJz7osel4Exbe-5KrvIZuD4kRog0"

// Definitions for TFT pins
#define TFT_CS 19  // Chip select control pin
#define TFT_DC 18  // Data Command control pin
#define TFT_RST 5  // Reset pin (could connect to RST pin)
#define TFT_MOSI 17
#define TFT_SCLK 16

// Pin definitions
constexpr uint8_t RPM_PIN = 35;  // Pin for reading RPM
constexpr uint8_t STEP_PIN = 23; // Step pin for stepper motor
constexpr uint8_t DIRECTION_PIN = 2;  // Direction pin for stepper motor
constexpr uint8_t STEP_PIN_bomba = 22;  // Step pin for second stepper motor (bomba)
constexpr uint8_t DIRECTION_PIN_bomba = 2;  // Direction pin for second stepper motor (bomba)
constexpr uint8_t RELAY_PIN = 5;  // Pin for controlling relay
constexpr uint8_t TEMP_SENSOR_PIN = 21;  // Pin for connecting temperature sensor
constexpr uint8_t LOADCELL_DOUT_PIN = 32;  // Load cell data output pin
constexpr uint8_t LOADCELL_SCK_PIN = 33;  // Load cell serial clock pin

// Parameters for PID, motor and encoder
constexpr float INTERRUPTS_PER_REVOLUTION = 12; // Number of interrupts per motor revolution
constexpr double KP = 0.12; // Proportional gain for PID controller
constexpr double KI = 0.0003; // Integral gain for PID controller
constexpr double KD = 0; // Derivative gain for PID controller
constexpr int PULSE_WIDTH = 10; // Pulse width for PID controller

#define OUTPUT_MIN 0 // Minimum output for PID controller
#define OUTPUT_MAX 700 // Maximum output for PID controller
const int BAR_WIDTH = 120;  // Width of bar in display
const int BAR_HEIGHT = 10;  // Height of bar in display
const int BAR_X = 10;  // X position of bar in display
const int BAR_Y = 200;  // Y position of bar in display
const int S1 = 13; // CLK or A for encoder
const int S2 = 12; // DT or B for encoder
const int KEY = 14; // Push button for encoder

// Initialize global variables. These will be used throughout the program
TFT_eSPI tft = TFT_eSPI();  // Create TFT object
HX711 scale; // Create scale object for interfacing with load cell
OneWire oneWire(TEMP_SENSOR_PIN);  // Create OneWire object for temperature sensor
DallasTemperature tempSensor(&oneWire);  // Create DallasTemperature object for temperature sensor
ContinuousStepper stepper;  // Create ContinuousStepper object for motor control
ContinuousStepper bomba;  // Create second ContinuousStepper object for bomba control
TaskHandle_t temperatureTaskHandle;  // Handle for temperature measuring task

// Variables with initial values. These variables hold the state of the system and will be updated throughout the program
double SET_POINT = 40.0;  // Set point for temperature PID controller
double SET_PESO = 0;  // Set point for weight PID controller
volatile int rpmCount = 0;  // Counter for RPM
unsigned long lastMillis = 0;  // Variable to hold the last time in milliseconds
unsigned int rpm = 0;  // RPM of the motor
double temperature = 0.0;  // Current temperature
double peso, StepperSpeed_bomba;  // Current weight and speed of the bomba
bool relayState = false;  // State of the relay (false = off, true = on)
int SET_MOTOR = 0;  // Set speed for the motor
int StepperSpeed =  0;  // Current speed of the stepper motor
volatile int encoder_count = 0;  // Count for the encoder
static int encoder_H2OPeso = 0;  // Encoder count for water weight
static int encoder_RPMmotor = 0;  // Encoder count for motor RPM
static int encoder_TEMPh2o = 0;  // Encoder count for water temperature
bool direction; // Direction of the encoder (true = clockwise, false = counterclockwise)
bool button_pressed = false;  // State of the button (false = not pressed, true = pressed)
static const char* dots[] = {"   ", ".  ", ".. ", "..."};  // Array for creating a "loading" animation
static int dotIndex = 0;  // Index for the "loading" animation
int RPM_Objetivo = 0;  // Desired RPM
int WaterPeso = 0;  // Desired water weight

// Create PID objects for temperature and weight control
AutoPIDRelay pid(&temperature, &SET_POINT, &relayState, PULSE_WIDTH, KP, KI, KD);  // PID for temperature control
AutoPID pid2(&peso, &SET_PESO, &StepperSpeed_bomba, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);  // PID for weight control

// Setup for Blynk
char auth[] = BLYNK_AUTH_TOKEN;  // Paste auth token you copied
char ssid[] = "ainh007"; // Enter your WiFi name
char pass[] = "12345678"; // Enter WiFi password

// This is an interrupt service routine (ISR) for measuring the RPM. It is called every time the RPM pin goes from low to high (RISING edge). It increments the rpmCount variable every time it is called.
void rpmInterrupt() { rpmCount++; }

// These are Blynk functions that update the desired RPM and water weight based on the value from the Blynk app.
BLYNK_WRITE(V6) { RPM_Objetivo = param.asInt(); }
BLYNK_WRITE(V8) { WaterPeso = param.asInt(); }

// This is a task that runs on a separate core of the ESP32. It is responsible for measuring the temperature and updating the display.
void measureTemperatureTask(void * parameter) {
  for (;;) { // This is an infinite loop. The task will run as long as the system is running

    // This part of the code is responsible for measuring the RPM of the motor
    detachInterrupt(digitalPinToInterrupt(RPM_PIN));  // Detach the interrupt to make sure it doesn't fire while we are calculating the RPM
    rpm = (rpmCount*60*60) / (INTERRUPTS_PER_REVOLUTION*200);  // Calculate the RPM based on the number of interrupts and the time elapsed
    rpmCount = 0;  // Reset the counter
    lastMillis = millis();  // Update the lastMillis variable with the current time
    attachInterrupt(digitalPinToInterrupt(RPM_PIN), rpmInterrupt, RISING);  // Attach the interrupt again

    // This part of the code is responsible for measuring the temperature
    tempSensor.requestTemperatures();  // Request temperature from the sensor
    temperature = tempSensor.getTempCByIndex(0);  // Get the temperature in Celsius

    // This part of the code is responsible for controlling the relay
    // digitalWrite(RELAY_PIN, relayState);  // Control the relay based on the relayState variable

    // This part of the code is responsible for measuring the weight
    peso = scale.get_units(1);  // Get the weight from the scale in units

    // This part of the code is responsible for updating the TFT display
    tft.fillScreen( 0xFFFF);  // Clear the screen
    tft.setCursor(0, 0);  // Set the cursor to the top left corner
    tft.print("Temp Actual:  ");  // Print "Temp Actual: "
    tft.print(temperature);  // Print the current temperature
    tft.print((char)247);  // Print the degree symbol
    tft.print("C");  // Print "C" for Celsius

    // This part of the code is responsible for updating the Blynk app
    Blynk.virtualWrite(V5, temperature);  // Update the temperature widget in the Blynk app

    // The rest of the code updates the TFT display and Blynk app with the current state of the system (temperature set point, relay state, bomba speed, target weight, current weight, target RPM, current RPM)
    
    // Note: Blynk.run() is necessary for the Blynk library to process incoming commands and keep the connection to the server alive.
    Blynk.run();
    // Finally, the task is put to sleep for 780 milliseconds. This is done to prevent the task from using up all the processing power. 
    vTaskDelay(pdMS_TO_TICKS(780));
  }
}

void setup() {
  Serial.begin(115200);  // Start the Serial communication with the baud rate of 115200
  Serial.println("Starting...");  // Print a line to the serial monitor

  // TFT Setup
  tft.begin();  // Initialize the TFT screen
  tft.setRotation(3);  // Set the rotation of the screen
  tft.setTextColor(0x0000);  // Set the text color
  tft.setTextSize(2);  // Set the text size

  // Pin Modes
  pinMode(STEP_PIN, OUTPUT);  // Set the pin for the stepper motor as an output
  pinMode(DIRECTION_PIN, OUTPUT);  // Set the pin for the motor direction as an output
  pinMode(STEP_PIN_bomba, OUTPUT);  // Set the pin for the bomba stepper motor as an output
  pinMode(DIRECTION_PIN_bomba, OUTPUT);  // Set the pin for the bomba direction as an output
  pinMode(RELAY_PIN, OUTPUT);  // Set the pin for the relay as an output
  pinMode(RPM_PIN, INPUT_PULLUP);  // Set the pin for the RPM as an input with a pullup resistor
  pinMode(S1, INPUT_PULLUP);  // Set the pin for the encoder S1 as an input with a pullup resistor
  pinMode(S2, INPUT_PULLUP);  // Set the pin for the encoder S2 as an input with a pullup resistor
  pinMode(KEY, INPUT_PULLUP);  // Set the pin for the encoder button as an input with a pullup resistor

  // Initialize and setup Stepper Motors
  stepper.begin(STEP_PIN, DIRECTION_PIN);  // Initialize the stepper motor with the step and direction pins
  bomba.begin(STEP_PIN_bomba, DIRECTION_PIN_bomba);  // Initialize the bomba with the step and direction pins

  // Initialize and setup Temperature sensor
  tempSensor.begin();  // Initialize the temperature sensor

  // Initialize and setup Load Cell
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);  // Initialize the load cell with the data output and serial clock pins
  scale.set_scale(2280.f);  // Calibrate the scale
  scale.tare();  // Reset the scale to 0

  // Setup PID
  pid.setOutputRange(0, 1);  // Set the output range for the temperature PID controller
  pid2.setOutputRange(0, 1);  // Set the output range for the weight PID controller

  // Initialize and setup WiFi and Blynk
  WiFi.begin(ssid, pass);  // Start the WiFi connection with the given SSID and password
  Blynk.config(auth);  // Configure Blynk with the given auth token
  Blynk.connect();  // Connect to the Blynk server

  // Setup interrupt
  attachInterrupt(digitalPinToInterrupt(RPM_PIN), rpmInterrupt, RISING);  // Attach an interrupt to the RPM pin that triggers on the rising edge

  // Create a FreeRTOS task on core 1 for measuring the temperature
  xTaskCreatePinnedToCore(measureTemperatureTask, "Temperature Task", 10000, NULL, 1, &temperatureTaskHandle, 1);
}

void loop() {
  // The loop function runs over and over again forever
  // In this program, it is responsible for updating the state of the system and handling user input

  // Update the state of the system
  SET_POINT = encoder_TEMPh2o;  // Update the set point for the temperature PID controller based on the encoder input
  SET_PESO = encoder_H2OPeso;  // Update the set point for the weight PID controller based on the encoder input
  SET_MOTOR = encoder_RPMmotor;  // Update the set speed for the motor based on the encoder input

  // Control the motor
  stepper.spin(SET_MOTOR);  // Set the RPM of the motor

  // Control the bomba
  pid2.run();  // Run the weight PID controller
  bomba.spin(StepperSpeed_bomba);  // Set the RPM of the bomba

  // Control the temperature
  pid.run();  // Run the temperature PID controller

  // Check if the button is pressed
  if (digitalRead(KEY) == LOW) {  // If the button is pressed...
    button_pressed = true;  // Set the button_pressed variable to true
  } else {  // If the button is not pressed...
    button_pressed = false;  // Set the button_pressed variable to false
  }

  // Update the Blynk app
  Blynk.run();  // Process incoming commands and keep the connection to the server alive
}

