#ifndef __AVR_ATmega328P__
#define __AVR_ATmega328P__
#endif

#include "led_matrix.h"
#include <Arduino.h>
#include <Bounce2.h>
#include <EEPROM.h>
#include <Stepper.h>

const int DEBOUNCE_INTERVAL_MS = 10;
const int STEPS_PER_REVOLUTION = 200;
const int STEPPER_DELAY = 3000;
const int DELAY_BETWEEN_STEP_AND_PUMP = 5000;

volatile int timer_times;

volatile long unsigned eventTime;
volatile long unsigned previousEventTime;
volatile long unsigned timeSinceLastEvent;

enum class State { IDLE, START_PROG, PROG, RUN_ONCE, MEASURE_MOIST };
State state = State::IDLE;
enum btn_states : bool { open = true, pressed = false };
enum stepper_dir : byte { left = 0x1, right = 0x0 };

const int PUMP_SPEED_ANALOG_PIN = A0;
const int STOP_SWITCH_PIN = A1;
const int MOISTURE_SENSOR_PIN = A3;
const int MATRIX_DATA_PIN = A4;
const int MATRIX_CLC_PIN = A5;

const int STEPPER_DIR_PIN = 3;
const int STEPPER_SLEEP_PIN = 4;
const int STEPPER_STEP_PIN =  5;
const int PUMP_DRIVER_PIN = 6;
const int RUN_ONCE_BTN_PIN = 8;
const int PROG_BTN_STOP_PIN = 10;
const int PROG_BTN_NEXT_PIN = 11;
const int PROG_BTN_START_PIN = 12;
const int LED_PIN = 13;

Bounce btn_prog_start = Bounce();
Bounce btn_prog_next = Bounce();
Bounce btn_prog_stop = Bounce();
Bounce btn_run_once = Bounce();
Bounce btn_stop_switch = Bounce();

LedMatrix ledMatrix = LedMatrix();

long start_time;

void EEPROMWriteInt(int address, uint16_t value) {
  EEPROM.write(address, highByte(value));
  EEPROM.write(address + 1, lowByte(value));
}

uint16_t EEPROMReadInt(int address) {
  byte high = EEPROM.read(address);
  byte low = EEPROM.read(address + 1);
  return word(high, low);
}

void take_one_stepper_step(stepper_dir dir) {
    digitalWrite(STEPPER_DIR_PIN, dir); 

    digitalWrite(STEPPER_STEP_PIN, HIGH);
    delayMicroseconds(STEPPER_DELAY);
    digitalWrite(STEPPER_STEP_PIN, LOW);
    delayMicroseconds(STEPPER_DELAY);
}

void return_home() {
  btn_stop_switch.update();
  while(btn_stop_switch.read() == open) {
    take_one_stepper_step(right);
    btn_stop_switch.update();
  } 
}

void setup() {  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(PUMP_DRIVER_PIN, OUTPUT);
  digitalWrite(PUMP_DRIVER_PIN, LOW);

  pinMode(STEPPER_SLEEP_PIN, OUTPUT);
  digitalWrite(STEPPER_SLEEP_PIN, HIGH);

  pinMode(PROG_BTN_START_PIN, INPUT_PULLUP);
  btn_prog_start.attach(PROG_BTN_START_PIN);
  btn_prog_start.interval(DEBOUNCE_INTERVAL_MS); // interval in ms

  pinMode(PROG_BTN_NEXT_PIN, INPUT_PULLUP);
  btn_prog_next.attach(PROG_BTN_NEXT_PIN);
  btn_prog_next.interval(DEBOUNCE_INTERVAL_MS);

  pinMode(PROG_BTN_STOP_PIN, INPUT_PULLUP);
  btn_prog_stop.attach(PROG_BTN_STOP_PIN);
  btn_prog_stop.interval(DEBOUNCE_INTERVAL_MS);

  pinMode(RUN_ONCE_BTN_PIN, INPUT_PULLUP);
  btn_run_once.attach(RUN_ONCE_BTN_PIN);
  btn_run_once.interval(DEBOUNCE_INTERVAL_MS);

  pinMode(STOP_SWITCH_PIN, INPUT_PULLUP);
  btn_stop_switch.attach(STOP_SWITCH_PIN);
  btn_stop_switch.interval(DEBOUNCE_INTERVAL_MS);

  Serial.begin(9600); // Starts the serial communication
  Serial.println(F("sliding_waterfall_v1"));

  ledMatrix.connect(0x70); // pass in the address

  cli();

  TCCR2A = 0; // set entire TCCR2A register to 0
  TCCR2B = 0; // same for TCCR2B
  TCNT2 = 0;  // initialize counter value to 0
  // set compare match register for 8khz increments
  OCR2A = 249; // = (16*10^6) / (8000*8) - 1 (must be <256)
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS21 bit for 8 prescaler
  TCCR2B |= (1 << CS21) | (1 << CS22);
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);

  sei();

  ledMatrix.draw_heads_up_seq(1000);
  return_home();
}

ISR(TIMER2_COMPA_vect) { // timer0 interrupt 500Hz
  timer_times++;
  if (timer_times > 500) {
    //       eventTime = millis();
    //       timeSinceLastEvent = eventTime - previousEventTime;
    // Serial.print(F("timer "));
    // Serial.println(timeSinceLastEvent);
    timer_times = 0;
  }
}

byte prog_len;
bool tmp;
int pump_speed;
bool doStepper;

void loop() {
  ledMatrix.clear();
  ledMatrix.draw_shower();
  delay(500);
  ledMatrix.clear();
  ledMatrix.draw_circle();
  delay(500);
  ledMatrix.clear();
  ledMatrix.draw_one_char_to_disp("Q");
  delay(500);

  
  return;

  btn_prog_start.update();
  btn_prog_next.update();
  btn_prog_stop.update();
  btn_run_once.update();
  btn_stop_switch.update();

  bool isProgStart = btn_prog_start.read();
  bool isProgNext = btn_prog_next.read();
  bool isProgStop = btn_prog_stop.read();
  bool isRunOnce = btn_run_once.read();
  bool isStopSwitch = btn_stop_switch.read();

  if (state != State::PROG) {
    if (isProgStart == pressed) {
      Serial.println(F("start prog is pressed"));
      state = State::START_PROG;
    } else if (isRunOnce == pressed) {
      state = State::RUN_ONCE;
    } else {
      state = State::IDLE;
    }
  }

  switch (state) {
  case State::IDLE:
    break;
  case State::MEASURE_MOIST:
    break;
  case State::RUN_ONCE:
    Serial.println(F("State::RUN_ONCE"));
    if (EEPROM.read(0) != 90) {
      state = State::IDLE;
      break;
    }
    doStepper = true;
    Serial.println(F("memory"));
    prog_len = EEPROM.read(3);
    Serial.println(prog_len);
    for (int mp = 4; mp < prog_len * 2 + 4; mp += 2) {
      int mem = EEPROMReadInt(mp);
      Serial.print(mem);
      Serial.print(F(" at "));
      Serial.println(mp);
      if (doStepper) {
        ledMatrix.draw_circle();
        for(int i = 0; i < mem; i++) {
          take_one_stepper_step(right);
        }
      } else {
        ledMatrix.draw_shower();
        pump_speed = analogRead(PUMP_SPEED_ANALOG_PIN);
        pump_speed = map(pump_speed, 0, 1023, 0, 255);
        analogWrite(PUMP_DRIVER_PIN, pump_speed);
        start_time = millis();
        do {
          delay(50);
        } while (millis() - start_time < mem);
        analogWrite(PUMP_DRIVER_PIN, 0);
      }
      doStepper = !doStepper;
      delay(DELAY_BETWEEN_STEP_AND_PUMP);
    }
    ledMatrix.clear();
    //
    return_home();
    state = State::IDLE;
    break;
  case State::START_PROG:
    Serial.println(F("State::START_PROG"));
    start_time = millis();
    while (millis() - start_time < 5000 && btn_prog_start.read() == pressed) {
      delayMicroseconds(10);
      btn_prog_start.update();
    }
    if (millis() - start_time >= 5000) {
      ledMatrix.draw_heads_up_seq(1000);
      do {
        btn_prog_start.update();
        delay(40);
      } while (btn_prog_start.read() == pressed);

      state = State::PROG;
    } else {
      Serial.println(F("Set State::IDLE"));
      state = State::IDLE;
    }
    break;
  case State::PROG:
    Serial.println(F("State::PROG"));

    // write to EEPROM
    // format 90,79,203,len,no steps,time to pump,no steps,time to pump.....

    if (EEPROM.read(0) != 90) {
      EEPROM.write(0, 90);
      EEPROM.write(1, 79);
      EEPROM.write(2, 203);
    }

    byte no_moves = 0;
    bool pump = false;
    bool cont = true;
    int current_adress = 4;

    // cont = false;
    while (cont) {
      if (pump == false) {
        ledMatrix.draw_circle();
        // count no steps
        uint16_t no_steps = 0;
        do {
          take_one_stepper_step(right);
          no_steps++;

          btn_prog_stop.update();
          if (btn_prog_stop.read() == pressed) {
            cont = false;
          }

          btn_prog_next.update();
        } while (btn_prog_next.read() == open && cont == true);

        if (cont == false) {
          continue;
        }
        Serial.print(F("writing steps: "));
        Serial.print(no_steps);
        Serial.print(F(" at "));
        Serial.println(current_adress);

        EEPROMWriteInt(current_adress, no_steps);
        current_adress += 2;
        no_moves++;
      } else {
        ledMatrix.draw_shower();

        // measure running time of pump
        unsigned long start = millis();

        pump_speed = analogRead(PUMP_SPEED_ANALOG_PIN);
        pump_speed = map(pump_speed, 0, 1023, 0, 255);
        analogWrite(PUMP_DRIVER_PIN, pump_speed);

        do {
          btn_prog_next.update();
          btn_prog_stop.update();
          if (btn_prog_stop.read() == pressed) {
            cont = false;
          }
          delayMicroseconds(50);
        } while (btn_prog_next.read() == open && cont == true);

        analogWrite(PUMP_DRIVER_PIN, 0);

        if (cont == false) {
          continue;
        }

        int elapsed_time = millis() - start;
        Serial.print(F("writing time: "));
        Serial.print(elapsed_time);
        Serial.print(F(" at "));
        Serial.println(current_adress);

        EEPROMWriteInt(current_adress, elapsed_time);
        current_adress += 2;
        no_moves++;
      }

      pump = !pump;

      do {
        btn_prog_next.update();
      } while (btn_prog_next.read() == pressed);

      btn_prog_stop.update();
      if (btn_prog_stop.read() == pressed) {
        cont = false;
      }

      delay(DELAY_BETWEEN_STEP_AND_PUMP);
    }
    EEPROM.write(3, no_moves);
    Serial.println(F("Done PROG"));
    ledMatrix.clear();
    state = State::IDLE;
    break;
  }
}
