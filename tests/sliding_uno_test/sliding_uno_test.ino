#ifndef __AVR_ATmega328P__
#define __AVR_ATmega328P__
#endif

#include <Arduino.h>
#include <Bounce2.h>
#include <Stepper.h>

const int DEBOUNCE_INTERVAL_MS = 10;
const int STEPS_PER_REVOLUTION = 200;

volatile int timer_times;

volatile long unsigned eventTime;
volatile long unsigned previousEventTime;
volatile long unsigned timeSinceLastEvent;

enum class State { IDLE, MOVE_LEFT, MOVE_RIGHT };
State state = State::IDLE;
enum btn_states : bool { open = true, pressed = false };

const int MOVE_LEFT_PIN = 11;
const int MOVE_RIGHT_PIN = 12;
const int LED_PIN = 13;

Bounce btn_move_left = Bounce();
Bounce btn_move_right = Bounce();
Stepper stepper(STEPS_PER_REVOLUTION, 4, 5, 6, 7);

long start_time;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(MOVE_LEFT_PIN, INPUT_PULLUP);
  btn_move_left.attach(MOVE_LEFT_PIN);
  btn_move_left.interval(DEBOUNCE_INTERVAL_MS); // interval in ms

  pinMode(MOVE_RIGHT_PIN, INPUT_PULLUP);
  btn_move_right.attach(MOVE_RIGHT_PIN);
  btn_move_right.interval(DEBOUNCE_INTERVAL_MS); // interval in ms

  Serial.begin(9600); // Starts the serial communication
  Serial.println(F("sliding_uno_test_v1"));

  stepper.setSpeed(30);

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

void loop() {

  btn_move_left.update();
  btn_move_right.update();

  bool isMoveLeft = btn_move_left.read();
  bool isMoveRight = btn_move_right.read();

  if(isMoveLeft == pressed) {
    state = State::MOVE_LEFT;
  } else if(isMoveRight == pressed) {
    state = State::MOVE_RIGHT;
  } else {
    state = State::IDLE;
  }

  switch (state) {
  case State::IDLE:
    break;
  case State::MOVE_LEFT:
    Serial.println(F("State::MOVE_LEFT"));
    do {
      btn_move_left.update();
      stepper.step(1);
      //delay(40);
    } while (btn_move_left.read() == pressed);

    state = State::IDLE;
    break;
  case State::MOVE_RIGHT:
    Serial.println(F("State::MOVE_RIGHT"));
    do {
      btn_move_right.update();
      stepper.step(-1);
      //delay(40);
    } while (btn_move_right.read() == pressed);

    state = State::IDLE;
    break;
  }
}
