#ifndef __AVR_ATmega328P__
    #define __AVR_ATmega328P__
#endif

#include <Arduino.h>

#include <Bounce2.h>
#include <DHT.h>
#include <Stepper.h>

const int STEPS_PER_REVOLUTION = 200;
const int DEBOUNCE_INTERVAL_MS = 10;

#define DHTTYPE DHT22   // DHT 22  (AM2302)

#define BTN_STOP 2
#define BTN_CLOSE 8
#define BTN_OPEN 9
#define DHTPIN 13     // what pin we're connected to

enum class State { IDLE, TAKE_MEASUREMENT, CLOSE, OPEN };
enum btn_states : bool { open = true, pressed = false };

struct MeasurementVal {
  float hum;  // Stores humidity value
  float temp; // Stores temperature value
}; 

volatile unsigned int timer_times = 0;

// 200 steps = 1 turn
// 10 turns = 12.5 cm
// 1 expance = 200 steps * 10 revolutions * 10 times = 20000 steps
int current_step = 0;
byte current_expance = 0;
const byte MAX_EXPANCES = 3;
const int STEPS_PER_EXPANCE = 20000;
const unsigned int TOTAL_MAXIMUM_STEPS = 60000;
unsigned int current_total_step = 0;

volatile long unsigned eventTime;
volatile long unsigned previousEventTime;
volatile long unsigned timeSinceLastEvent;

State state = State::IDLE;
Stepper stepper(STEPS_PER_REVOLUTION, 3, 4, 5, 6);
DHT dht(DHTPIN, DHTTYPE); //// Initialize DHT sensor for normal 16mhz Arduino

Bounce btn_stop = Bounce(); 
Bounce btn_close = Bounce(); 
Bounce btn_open = Bounce(); 

bool init_sesam = true;

void setup() {
  Serial.begin(9600);
  Serial.println(F("OpenSesame v3"));
  cli();

  //set timer1 interrupt at 1Hz
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

  sei();

  pinMode(BTN_STOP, INPUT_PULLUP);
  btn_stop.attach(BTN_STOP);
  btn_stop.interval(DEBOUNCE_INTERVAL_MS); // interval in ms
  
  pinMode(BTN_CLOSE, INPUT_PULLUP);
  btn_close.attach(BTN_CLOSE);
  btn_close.interval(DEBOUNCE_INTERVAL_MS); // interval in ms

  pinMode(BTN_OPEN, INPUT_PULLUP);
  btn_open.attach(BTN_OPEN);
  btn_open.interval(DEBOUNCE_INTERVAL_MS); // interval in ms

  stepper.setSpeed(180);
}

ISR(TIMER1_COMPA_vect){//timer1 interrupt 0.5Hz
  timer_times++;
  Serial.println(timer_times);
  if(timer_times >= 30 && state == State::IDLE) {
    eventTime = millis();
    state = State::TAKE_MEASUREMENT;
    timer_times = 0;
  }
}

void takeMeasurement(MeasurementVal &val) {
  //Read data and store it to variables hum and temp
  val.hum = dht.readHumidity();
  val.temp = dht.readTemperature();
}

bool is_between(float value, float lower, float upper) {
  return value > lower && value <= upper;
}

void serial_state(State state) {
  Serial.print(F("State "));
  switch(state)
  {
        case State::IDLE:
          Serial.println(F("IDLE"));
          break;
        case State::CLOSE:
          Serial.println(F("CLOSE"));
          break;
        case State::OPEN:
          Serial.println(F("OPEN"));
          break;
        case State::TAKE_MEASUREMENT:
          Serial.println(F("TAKE_MEASUREMENT"));
          break;
  }
}

void openOneExpansion() {
  Serial.println(F("openOneExpansion"));

  if(current_expance < MAX_EXPANCES) {
    unsigned int steps_to_open = STEPS_PER_EXPANCE - current_step; 

    Serial.print(F("  opening: "));
    Serial.println(steps_to_open);

    current_total_step += steps_to_open;
    stepper.step(steps_to_open);
    current_expance++;
    current_step = 0;
  }
}

void resetCounters() {
  current_expance = 0;
  current_step = 0;
  current_total_step = 0;
}

void loop() {
  btn_stop.update();
  btn_close.update();
  btn_open.update();

  bool isStop = btn_stop.read();
  bool isClose = btn_close.read();
  bool isOpen = btn_open.read();

  if(init_sesam) {
    state = State::CLOSE;
    init_sesam = false;
  }
  else {
    if(isOpen == pressed) {
      state = State::OPEN;
    } else if(isClose == pressed) {
      state = State::CLOSE;
    } else if(state != State::TAKE_MEASUREMENT) {
      state = State::IDLE;
    }
  }

  switch(state)
  {
        case State::IDLE:
        break;
        case State::TAKE_MEASUREMENT:
          serial_state(state);

          timeSinceLastEvent = eventTime - previousEventTime;
          previousEventTime = eventTime;

          Serial.print(timeSinceLastEvent);
          Serial.println(F(" ms: "));
          //Read data and store it to variables hum and temp
          MeasurementVal mval;
          takeMeasurement(mval);
          //Print temp and humidity values to serial monitor
          Serial.print(F("Humidity: "));
          Serial.print(mval.hum);
          Serial.print(F(" %, Temp: "));
          Serial.print(mval.temp);
          Serial.println(F(" Celsius"));

          if(mval.temp <= 20.0) {
            state = State::CLOSE;            
          } else {
            if(is_between(mval.temp, 20.0, 25.0) && current_expance == 0) {
              openOneExpansion();
            }
            else if(is_between(mval.temp, 25.0, 30.0) && current_expance <= 1)  {
              openOneExpansion();
            } 
            else if(mval.temp > 30.0 && current_expance <= 2)  {
              openOneExpansion();
            }
            state = State::IDLE;
          }
          Serial.print(F("State: "));
          serial_state(state);
          Serial.print(F("Current step: "));
          Serial.println(current_step);
          Serial.print(F("Current expance: "));
          Serial.println(current_expance);
          
          break;
        case State::CLOSE:
          //serial_state(state);

          while(btn_stop.read() == open) {
            current_total_step--;
            stepper.step(-1);
            btn_stop.update();
          } 
          state = State::IDLE;
          resetCounters();

          //Serial.print(current_step);
          //Serial.print(F(" "));
          //Serial.print(current_expance);
          //Serial.print(F(" "));
          //Serial.println(current_total_step);
          
          break;
        case State::OPEN:
          serial_state(state);

          do {
            current_step++;
            if(current_step >= STEPS_PER_EXPANCE) {
              current_step = 0;
              current_expance++;
            }
            current_total_step++;
            stepper.step(1);
            btn_open.update();
          } while(btn_open.read() == pressed);

          state = State::IDLE;

          Serial.print(current_step);
          Serial.print(F(" "));
          Serial.print(current_expance);
          Serial.print(F(" "));
          Serial.println(current_total_step);

          break;
  } 
}
