#ifndef __AVR_ATmega328P__
#define __AVR_ATmega328P__
#endif

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

enum class State
{
    IDLE,
    START_PROG,
    PROG
};
State state = State::IDLE;
enum btn_states : bool
{
    open = true,
    pressed = false
};

const int RED_PIN = 3;
const int GREEN_PIN = 2;
const int START_PROG_PIN = 7;
const int PROG_NEXT_PIN = 8;
const int PROG_STOP_PIN = 9;
const int LED_PIN = 13;

Bounce btn_prog = Bounce();
Bounce btn_next = Bounce();
Bounce btn_stop = Bounce();
Stepper stepper(STEPS_PER_REVOLUTION, 3, 4, 5, 6);

long start_time;

void EEPROMWriteInt(int address, uint16_t value)
{
    byte two = (value & 0xFF);
    byte one = ((value >> 8) & 0xFF);
    EEPROM.write(address, two);
    EEPROM.write(address + 1, one);
}

uint16_t EEPROMReadInt(int address)
{
    //Read the 4 bytes from the eeprom memory.
    int two = EEPROM.read(address);
    int one = EEPROM.read(address + 1);

    //Return the recomposed int by using bitshift.
    return ((two << 0) & 0xFF) + ((one << 8) & 0xFF);
}

void setup()
{
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    pinMode(RED_PIN, OUTPUT);
    digitalWrite(RED_PIN, LOW);
    pinMode(GREEN_PIN, OUTPUT);
    digitalWrite(GREEN_PIN, LOW);

    pinMode(START_PROG_PIN, INPUT_PULLUP);
    btn_prog.attach(START_PROG_PIN);
    btn_prog.interval(DEBOUNCE_INTERVAL_MS); // interval in ms
    
    pinMode(PROG_NEXT_PIN, INPUT_PULLUP);
    btn_next.attach(PROG_NEXT_PIN);
    btn_next.interval(DEBOUNCE_INTERVAL_MS);

    pinMode(PROG_STOP_PIN, INPUT_PULLUP);
    btn_stop.attach(PROG_STOP_PIN);
    btn_stop.interval(DEBOUNCE_INTERVAL_MS);

    Serial.begin(9600); // Starts the serial communication
    Serial.println(F("sliding_waterfall_v1"));

    stepper.setSpeed(180);

    cli();

    TCCR2A = 0; // set entire TCCR2A register to 0
    TCCR2B = 0; // same for TCCR2B
    TCNT2 = 0;  //initialize counter value to 0
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

ISR(TIMER2_COMPA_vect)
{ //timer0 interrupt 500Hz
    timer_times++;
    if (timer_times > 500)
    {
        //       eventTime = millis();
        //       timeSinceLastEvent = eventTime - previousEventTime;
        Serial.print(F("timer "));
        Serial.println(timeSinceLastEvent);
        timer_times = 0;
    }
}

void loop()
{
    btn_prog.update();

    if(state != State::PROG) {
        bool isProg = btn_prog.read();
        if (isProg == pressed)
        {
            state = State::START_PROG;
        }
        else
        {
            state = State::IDLE;
        }
    }

    switch (state)
    {
    case State::IDLE:
        break;
    case State::START_PROG:
        Serial.println(F("Start prog"));
        start_time = millis();
        while (millis() - start_time < 5000 && btn_prog.read() == pressed)
        {
            delayMicroseconds(10);
            btn_prog.update();
        }
        if (millis() - start_time >= 5000)
        {
            digitalWrite(LED_PIN, HIGH);
            state = State::PROG;
        }
        else
        {
            state = State::IDLE;
        }
        break;
    case State::PROG:
        // write to EEPROM
        // format 90,79,203,len,no steps,time to pump,no steps,time to pump.....
        
        bool led_state = false;
        for(int i = 0; i < 10; i++) {
            digitalWrite(RED_PIN, led_state);
            digitalWrite(GREEN_PIN, !led_state);
            led_state = !led_state;
            delay(500);
        }
        digitalWrite(RED_PIN, LOW);
        digitalWrite(GREEN_PIN, LOW);

        if(EEPROM.read(0) != 90) {
            EEPROM.write(0, 90);
            EEPROM.write(1, 79);
            EEPROM.write(2, 203);
        }

        byte no_moves = 0;
        bool pump = false;
        bool cont = true;
        int current_adress = 4;

        while(cont) {
            if(pump == false) {
                digitalWrite(GREEN_PIN, HIGH);

                // count no steps
                uint16_t no_steps = 0;
                do {
                    stepper.step(1);
                    no_steps++;

                    btn_stop.update();
                    if(btn_stop.read() == pressed) {
                        cont = false;
                    }

                    btn_next.update();
                } while(btn_next.read() == open && cont == true);

                EEPROMWriteInt(current_adress, no_steps);
                current_adress += 2;
                no_moves++;
                digitalWrite(GREEN_PIN, LOW);
            } else {
                digitalWrite(GREEN_PIN, HIGH);

                // measure running time of pump
                long start = millis();
                do {
                    btn_next.update();

                    btn_stop.update();
                    if(btn_stop.read() == pressed) {
                        cont = false;
                    }

                } while(btn_next.read() == open && cont == true);

                EEPROMWriteInt(current_adress, millis() - start);
                current_adress += 2;
                no_moves++;
                digitalWrite(GREEN_PIN, LOW);
            }
            
            pump = !pump;

            digitalWrite(RED_PIN, HIGH);
            delay(2000);
            digitalWrite(RED_PIN, LOW);

            btn_stop.update();
            if(btn_stop.read() == pressed) {
                cont = false;
            }
        }

        EEPROM.write(3, no_moves);
        state = State::IDLE;
        break;
    }
}
