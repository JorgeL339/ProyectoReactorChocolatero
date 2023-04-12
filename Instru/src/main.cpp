#include <ContinuousStepper.h>
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
const uint8_t stepPin = 33;
const uint8_t dirPin = 25;

ContinuousStepper stepper;

void setup() {
  stepper.begin(stepPin, dirPin);
  stepper.spin(600);
}

void loop() {
  stepper.loop();
}