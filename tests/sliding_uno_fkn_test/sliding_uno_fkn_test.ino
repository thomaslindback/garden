#ifndef __AVR_ATmega328P__
#define __AVR_ATmega328P__
#endif

#include <Arduino.h>
#include <Bounce2.h>
#include <Stepper.h>

enum btn_states : bool { open = true, pressed = false };

const int DEBOUNCE_INTERVAL_MS = 10;
const int STEPS_PER_REVOLUTION = 200;

const int PROG_START_PIN = 12;
const int PROG_NEXT_PIN = 11;
const int PROG_STOP_PIN = 10;
const int RUN_ONCE_PIN = 8;

const int STOP_SWITCH_PIN = A1;
const int MOISTURE_SENSOR_PIN = A3;

const int LED_PIN = 13;

Bounce btn_prog_start = Bounce();
Bounce btn_prog_next = Bounce();
Bounce btn_prog_stop = Bounce();
Bounce btn_run_once = Bounce();
Bounce btn_stop_switch = Bounce();

//Stepper stepper(STEPS_PER_REVOLUTION, 4, 5, 6, 7);


void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(PROG_START_PIN, INPUT_PULLUP);
  btn_prog_start.attach(PROG_START_PIN);
  btn_prog_start.interval(DEBOUNCE_INTERVAL_MS); // interval in ms

  pinMode(PROG_NEXT_PIN, INPUT_PULLUP);
  btn_prog_next.attach(PROG_NEXT_PIN);
  btn_prog_next.interval(DEBOUNCE_INTERVAL_MS);

  pinMode(PROG_STOP_PIN, INPUT_PULLUP);
  btn_prog_stop.attach(PROG_STOP_PIN);
  btn_prog_stop.interval(DEBOUNCE_INTERVAL_MS);

  pinMode(RUN_ONCE_PIN, INPUT_PULLUP);
  btn_run_once.attach(RUN_ONCE_PIN);
  btn_run_once.interval(DEBOUNCE_INTERVAL_MS);

  pinMode(STOP_SWITCH_PIN, INPUT_PULLUP);
  btn_stop_switch.attach(STOP_SWITCH_PIN);
  btn_stop_switch.interval(DEBOUNCE_INTERVAL_MS);

  Serial.begin(9600); // Starts the serial communication
  Serial.println(F("sliding_uno_fkn_test_v1"));

  //stepper.setSpeed(30);

}

void loop() {

  btn_prog_start.update();
  btn_prog_next.update();
  btn_prog_stop.update();
  btn_run_once.update();
  btn_stop_switch.update();

  bool isProgStart = btn_prog_start.read();
  bool isProgNext = btn_prog_next.read();
  bool isProgStop = btn_prog_stop.read();
  bool isRunOnec = btn_run_once.read();
  bool isStopSwitch = btn_stop_switch.read();


  if(isProgStart == pressed) {
    Serial.println(F("start"));
  } else if(isProgNext == pressed) {
    Serial.println(F("next"));
  } else if(isProgStop == pressed) {
    Serial.println(F("stop"));
  } else if(isRunOnec == pressed) {
    Serial.println(F("runonce"));
  } else if(isStopSwitch == pressed) {
    Serial.println(F("stop switch"));
  } else {
    Serial.println(F("no button"));
  }

  delay(250);
  int sensorValue = analogRead(MOISTURE_SENSOR_PIN);
  sensorValue = map(sensorValue, 1024, 300 ,0, 100);
  // print out the value you read:
  Serial.println(sensorValue);
}
