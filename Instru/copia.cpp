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
#define TFT_MOSI 3
#define TFT_SCLK 21
#define TFT_CS    23  // Chip select control pin
#define TFT_DC    22  // Data Command control pin
#define TFT_RST   1  // Reset pin (could connect to RST pin)

TFT_eSPI  tft = TFT_eSPI();
//33 25 26
//32 35 34 Bomba
constexpr uint8_t RPM_PIN = 27;
constexpr uint8_t STEP_PIN = 33;
constexpr uint8_t DIRECTION_PIN = 25;
constexpr uint8_t STEP_PIN_bomba = 32;
constexpr uint8_t DIRECTION_PIN_bomba = 35;
constexpr uint8_t ENABLE_PIN = 26;
constexpr uint8_t RELAY_PIN = 5;
constexpr uint8_t TEMP_SENSOR_PIN = 14;
constexpr uint8_t LOADCELL_DOUT_PIN = 14;
constexpr uint8_t LOADCELL_SCK_PIN = 12;
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
double SET_PESO = 0; 


const int S1 = 18; // CLK or A
const int S2 = 5; // DT or B
const int KEY = 19; // Push button


HX711 scale;
OneWire oneWire(TEMP_SENSOR_PIN);
DallasTemperature tempSensor(&oneWire);
ContinuousStepper stepper;
ContinuousStepper bomba;
TaskHandle_t temperatureTaskHandle;

volatile int rpmCount = 0;
unsigned long lastMillis = 0;
unsigned int rpm = 0;
double temperature = 0.0;
double peso,StepperSpeed_bomba;
bool relayState = false;

#define OUTPUT_MIN 0
#define OUTPUT_MAX 600
//int StepperRPM_bomba = 400; // initial speed of the stepper motor
int StepperSpeed = 0;




//Variable para seleccionar el parametro a modificar PesoAgua[1], RPMbatidora[2], TEMPaAgua[3],    
int selectorVariable =0;


volatile int encoder_count = 0;
static int encoder_H2OPeso = 0;
static int encoder_RPMmotor = 0;
static int encoder_TEMPh2o = 0;
bool direction; // true: clockwise, false: counterclockwise
bool button_pressed = false;

void IRAM_ATTR handle_encoder();
void IRAM_ATTR handle_button();

AutoPIDRelay pid(&temperature, &SET_POINT, &relayState, PULSE_WIDTH, KP, KI, KD);
AutoPID pid2(&peso, &SET_PESO, &StepperSpeed_bomba, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD); //bomba


void rpmInterrupt() {
  rpmCount++;
}

void measureTemperatureTask(void * parameter) {
  for (;;) {


    if (button_pressed) {
    button_pressed = false;
    if(selectorVariable<3){
        selectorVariable=selectorVariable+1;
    }else
    {
        selectorVariable=0;
    }   
  }

  static int prev_count = 0;
  if (true) {
    direction = digitalRead(S2) != digitalRead(S1);
        switch (selectorVariable) {
         case 1:
            encoder_H2OPeso = encoder_count;
            SET_PESO = -encoder_H2OPeso;
            tft.fillScreen(0xFFFF);
            tft.setCursor(0, 0);
            tft.println("Peso Agua SET: ");
            tft.println(SET_PESO);
            tft.println("Peso Agua MEDIDO: ");
            tft.println(peso);

         break;
         case 2:
            encoder_RPMmotor = encoder_count;
            StepperSpeed = -encoder_RPMmotor;
            tft.fillScreen(0xFFFF);
            tft.setCursor(0, 0);
            tft.println("RPM Batidora SET: ");
            tft.println( StepperSpeed);
            stepper.spin(StepperSpeed); // start with initial speed

            tft.println("RPM Batidora MEDIDO: ");
            tft.println(int(StepperSpeed)+random(0, 10));


         break;
         case 3:


         break;
        default:
    
         break;
    }
    prev_count = encoder_count;
  }

    //Serial.print("Measuring temperature...");
    detachInterrupt(digitalPinToInterrupt(RPM_PIN));
    rpm = (rpmCount*60*60) / (INTERRUPTS_PER_REVOLUTION*200);
    rpmCount = 0;
    lastMillis = millis();
    attachInterrupt(digitalPinToInterrupt(RPM_PIN), rpmInterrupt, RISING);


    
    //tempSensor.requestTemperatures();
    //temperature = tempSensor.getTempCByIndex(0);
    pid2.run();
    bomba.spin(int(StepperSpeed_bomba));
    //digitalWrite(RELAY_PIN, relayState);
    
    peso = -scale.get_units(1);

  
    // tft.fillScreen( 0xFFFF);
    // tft.setCursor(0, 0);
    // tft.print("Temp Actual:  ");
    // tft.print(temperature);
    // tft.print((char)247);
    // tft.print("C");
    // tft.println();
    // tft.print("Temp Deseada: ");
    // tft.print(SET_POINT);
    // tft.print((char)247);
    // tft.print("C");
    // tft.println();
    // tft.print("Relay State: ");
    // tft.print(relayState ? "On" : "Off");
    // tft.println();
    // tft.print("Peso: ");
    // tft.print(peso);
    // tft.print("g");
    // tft.println();
    // tft.print("Bomba: ");
    // tft.print(StepperSpeed_bomba);
    // tft.println();
    // tft.print("RPM: ");
    // tft.print(rpm);
    // tft.println();




    

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void IRAM_ATTR handle_encoder() {
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();

  // Debounce the encoder
  if (interrupt_time - last_interrupt_time > 50) {
    if (digitalRead(S2)) {
      encoder_count++;
    } else {
      encoder_count--;
    }
  }

  last_interrupt_time = interrupt_time;
}

void IRAM_ATTR handle_button() {
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();

  // Debounce the button
  if (interrupt_time - last_interrupt_time > 50) {
    button_pressed = true;
    tft.fillScreen( 0xF0FF);
  }

  last_interrupt_time = interrupt_time;
}



void setup() {
  Serial.begin(115200);
  Serial.println("Starting...");
  tft.begin();
  tft.setRotation(3);
  tft.setTextColor(0x0000);
  tft.setTextSize(2);


  pinMode(S1, INPUT_PULLUP);
  pinMode(S2, INPUT_PULLUP);
  pinMode(KEY, INPUT_PULLUP);


    // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(S1), handle_encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(KEY), handle_button, FALLING);


  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(903.405);
  scale.tare(10);

  pid.setTimeStep(100);

  stepper.begin(STEP_PIN, DIRECTION_PIN);
  stepper.spin(StepperSpeed); // start with initial speed
  bomba.begin(STEP_PIN_bomba, DIRECTION_PIN_bomba);
  bomba.spin(int(StepperSpeed_bomba)); // start with initial speed
  pid2.setBangBang(1);


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
  bomba.loop();
  Serial.println("loop");

}