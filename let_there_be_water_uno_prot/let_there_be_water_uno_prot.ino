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
    const int BTN_PUMP_PIN = 12;
    const int PUMP_DRIVER_PIN = 6;    
    const int PUMP_SPEED_ANALOG_PIN = A0;
#endif

volatile int timer_times;

volatile long unsigned eventTime;
volatile long unsigned previousEventTime;
volatile long unsigned timeSinceLastEvent;
int pump_speed = 0;
Bounce btn_pump = Bounce(); 

enum class State
{
    IDLE,
    PUMP,
    MEASURE,
    FILL_UP
};
State state = State::IDLE;
enum btn_states : bool { open = true, pressed = false };

// the setup routine runs once when you press reset:
void setup()
{
    pinMode(LED, OUTPUT);
    pinMode(PUMP_DRIVER_PIN, OUTPUT);
    digitalWrite(PUMP_DRIVER_PIN, LOW);
    pinMode(TRIG_PIN, OUTPUT); // Sets the trigPin as an Output
    pinMode(ECHO_PIN, INPUT); // Sets the echoPin as an Input

    pinMode(BTN_PUMP_PIN, INPUT_PULLUP);
    btn_pump.attach(BTN_PUMP_PIN);
    btn_pump.interval(DEBOUNCE_INTERVAL_MS); // interval in ms

    Serial.begin(9600); // Starts the serial communication
    Serial.println(F("let_there_be_water_uno_prot_v1"));
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

    return distance;
}

ISR(TIMER1_COMPA_vect){//timer1 interrupt 0.5Hz
  timer_times++;
  if(timer_times >= 10 && state == State::IDLE) {
    eventTime = millis();
    state = State::MEASURE;
    timer_times = 0;
  }
}

// the loop routine runs over and over again forever:
void loop() {
    btn_pump.update();
    
    bool isPump = btn_pump.read();
    if(isPump == pressed) {
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
            do {
              btn_pump.update();
              pump_speed = analogRead(PUMP_SPEED_ANALOG_PIN);
              pump_speed = map(pump_speed, 0, 1023, 0, 255);
              analogWrite(PUMP_DRIVER_PIN, pump_speed);
              delayMicroseconds(50);
            } while(btn_pump.read() == pressed);

            analogWrite(PUMP_DRIVER_PIN, 0);
            state = State::IDLE;
            break;
        case State::MEASURE: 
            Serial.print(F("measure "));
            water_level = measure();
            Serial.println(water_level);
            if(water_level > 20.0) {
                state = State::FILL_UP;
                Serial.println(F("-> state :: Fillup"));
            } else {
                state = State::IDLE;
                Serial.println(F("-> state :: idle"));
            }
            break;
        case State::FILL_UP:
            Serial.print(F("Fill_up: "));

            while(measure() > 12.0) {
                pump_speed = analogRead(PUMP_SPEED_ANALOG_PIN);
                pump_speed = map(pump_speed, 0, 1023, 0, 255);
                analogWrite(PUMP_DRIVER_PIN, pump_speed);
                delayMicroseconds(50);
            }
            analogWrite(PUMP_DRIVER_PIN, 0);
            state = State::IDLE;
            break;
    }
}