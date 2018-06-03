#ifndef __AVR_ATmega328P__
    #define __AVR_ATmega328P__
#endif

#include <Arduino.h>
#include <Bounce2.h>

const int DEBOUNCE_INTERVAL_MS = 10;

#ifdef __AVR_ATtiny85__
    const int LED = 1;
    const int TRIG_PIN = 9;
    const int ECHO_PIN = 10;
    const int BTN_PUMP_PIN = 7;
    const int PUMP_DRIVER_PIN = 8;
#else
    const int LED = 13;
    const int TRIG_PIN = 9;
    const int ECHO_PIN = 10;
    const int BTN_PUMP_PIN = 7;
    const int PUMP_DRIVER_PIN = 8;
#endif

volatile int timer_times;

volatile long unsigned eventTime;
volatile long unsigned previousEventTime;
volatile long unsigned timeSinceLastEvent;

Bounce btn_pump = Bounce(); 

enum class State
{
    IDLE,
    PUMP,
    MEASURE,
    FILL_UP
};
State state = State::IDLE;

// the setup routine runs once when you press reset:
void setup()
{
    pinMode(LED, OUTPUT);
    pinMode(PUMP_DRIVER_PIN, OUTPUT);
    pinMode(TRIG_PIN, OUTPUT); // Sets the trigPin as an Output
    pinMode(ECHO_PIN, INPUT); // Sets the echoPin as an Input

    pinMode(BTN_PUMP_PIN, INPUT_PULLUP);
    btn_pump.attach(BTN_PUMP_PIN);
    btn_pump.interval(DEBOUNCE_INTERVAL_MS); // interval in ms

    Serial.begin(9600); // Starts the serial communication

    cli();
#ifdef __AVR_ATmega328P__
    TCCR1A = 0;// set entire TCCR1A register to 0
    TCCR1B = 0;// same for TCCR1B
    TCNT1  = 0;//initialize counter value to 0
    // set compare match register for 2hz increments
    OCR1A = 31249; //(16000000) / (0.25*1024) - 1 //15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)
    // turn on CTC mode
    TCCR1B |= (1 << WGM12);
    // Set CS12 and CS10 bits for 1024 prescaler
    TCCR1B |= (1 << CS12) | (1 << CS10);  
    // enable timer compare interrupt
    TIMSK1 |= (1 << OCIE1A);  
#else

#endif
    sei();
}

float measure() {
    // Clears the trigPin
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    long duration = pulseIn(ECHO_PIN, HIGH);
    // Calculating the distance
    float distance = duration*0.034/2;
    // Prints the distance on the Serial Monitor
    Serial.print(F("Distance: "));
    Serial.println(distance);
}

ISR(TIMER1_COMPA_vect){//timer1 interrupt 0.5Hz
  timer_times++;
  if(timer_times >= 5 && state == State::IDLE) {
    eventTime = millis();
    state = State::MEASURE;
    timer_times = 0;
  }
}

// the loop routine runs over and over again forever:
void loop() {
    bool isPump = btn_pump.read();
    if(isPump == false) {
        state = State::PUMP;
    } else if(state == State::MEASURE) {
        state = State::MEASURE;
    } else if(state == State::FILL_UP) {
        state = State::FILL_UP;
    } else {
        state = State::IDLE;
    }

    float water_level = 0.0;
    switch (state) {
        case State::IDLE:
            analogWrite(PUMP_DRIVER_PIN, 0);
            break;
        case State::PUMP:
            analogWrite(PUMP_DRIVER_PIN, 100);
            break;
        case State::MEASURE: 
            water_level = measure();
            if(water_level > 15.0) {
                state = State::FILL_UP;
            } else {
                state = State::IDLE;
            }
            break;
        case State::FILL_UP:
            while(measure() > 10.0) {
                analogWrite(PUMP_DRIVER_PIN, 100);
                delayMicroseconds(50);
            }
            analogWrite(PUMP_DRIVER_PIN, 0);
            state = State::IDLE;
            break;
    }
}