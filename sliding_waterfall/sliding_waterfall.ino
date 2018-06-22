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

volatile int timer_times;

volatile long unsigned eventTime;
volatile long unsigned previousEventTime;
volatile long unsigned timeSinceLastEvent;

enum class State { IDLE, START_PROG, PROG, RUN_ONCE };
State state = State::IDLE;
enum btn_states : bool { open = true, pressed = false };

const int PUMP_SPEED_ANALOG_PIN = A0;
const int MATRIX_DATA_PIN = A4;
const int MATRIX_CLC_PIN = A5;

const int PUMP_DRIVER_PIN = 6;
const int START_PROG_PIN = 7;
const int PROG_NEXT_PIN = 8;
const int PROG_STOP_PIN = 9;
const int RUN_ONCE_PIN = 12;
const int LED_PIN = 13;

Bounce btn_start_prog = Bounce();
Bounce btn_next = Bounce();
Bounce btn_stop = Bounce();
Bounce btn_run_once = Bounce();
Stepper stepper(STEPS_PER_REVOLUTION, 3, 4, 5, 6);
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

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(PUMP_DRIVER_PIN, OUTPUT);
  digitalWrite(PUMP_DRIVER_PIN, LOW);

  pinMode(START_PROG_PIN, INPUT_PULLUP);
  btn_start_prog.attach(START_PROG_PIN);
  btn_start_prog.interval(DEBOUNCE_INTERVAL_MS); // interval in ms

  pinMode(PROG_NEXT_PIN, INPUT_PULLUP);
  btn_next.attach(PROG_NEXT_PIN);
  btn_next.interval(DEBOUNCE_INTERVAL_MS);

  pinMode(PROG_STOP_PIN, INPUT_PULLUP);
  btn_stop.attach(PROG_STOP_PIN);
  btn_stop.interval(DEBOUNCE_INTERVAL_MS);

  pinMode(RUN_ONCE_PIN, INPUT_PULLUP);
  btn_run_once.attach(RUN_ONCE_PIN);
  btn_run_once.interval(DEBOUNCE_INTERVAL_MS);

  Serial.begin(9600); // Starts the serial communication
  Serial.println(F("sliding_waterfall_v1"));

  stepper.setSpeed(180);

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

  ledMatrix.draw_heads_up_seq(500);
  return;

  btn_start_prog.update();
  btn_run_once.update();

  bool isStartProg = btn_start_prog.read();
  bool isRunOnce = btn_run_once.read();

  if (state != State::PROG) {
    if (isStartProg == pressed) {
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
        stepper.step(mem);
      } else {
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
      delay(4000);
    }
    state = State::IDLE;
    break;
  case State::START_PROG:
    Serial.println(F("State::START_PROG"));
    start_time = millis();
    while (millis() - start_time < 5000 && btn_start_prog.read() == pressed) {
      delayMicroseconds(10);
      btn_start_prog.update();
    }
    if (millis() - start_time >= 5000) {
      // TODO: Show wait
      do {
        btn_start_prog.update();
        delay(40);
      } while (btn_start_prog.read() == pressed);

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
        // count no steps
        uint16_t no_steps = 0;
        do {
          stepper.step(1);
          no_steps++;

          btn_stop.update();
          if (btn_stop.read() == pressed) {
            cont = false;
          }

          btn_next.update();
        } while (btn_next.read() == open && cont == true);

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
        // measure running time of pump
        unsigned long start = millis();

        pump_speed = analogRead(PUMP_SPEED_ANALOG_PIN);
        pump_speed = map(pump_speed, 0, 1023, 0, 255);
        analogWrite(PUMP_DRIVER_PIN, pump_speed);

        do {
          btn_next.update();
          btn_stop.update();
          if (btn_stop.read() == pressed) {
            cont = false;
          }
          delayMicroseconds(50);
        } while (btn_next.read() == open && cont == true);

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
        btn_next.update();
      } while (btn_next.read() == pressed);

      btn_stop.update();
      if (btn_stop.read() == pressed) {
        cont = false;
      }
    }
    EEPROM.write(3, no_moves);
    Serial.println(F("Done PROG"));
    state = State::IDLE;
    break;
  }
}
