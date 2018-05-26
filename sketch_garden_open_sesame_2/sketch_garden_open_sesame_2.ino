#include <Bounce2.h>
#include <DHT.h>;
#include <Stepper.h>

const int STEPS_PER_REVOLUTION = 200;
const int DEBOUNCE_INTERVAL_MS = 10;

#define DHTTYPE DHT22   // DHT 22  (AM2302)

#define BTN_ALL_STOP 2
#define BTN_CLOSE 8
#define BTN_OPEN 9
#define DHTPIN 13     // what pin we're connected to

#define cbi(reg, bit) (_SFR_BYTE(reg) &= ~_BV(bit)) // clear bit
#define sbi(reg, bit) (_SFR_BYTE(reg) |= _BV(bit)) // set bit

enum State: byte { DO_NOTHING, TAKE_MEASUREMENT, ALL_STOP, CLOSE, OPEN, AUTO_OPEN };

float hum;  // Stores humidity value
float temp; // Stores temperature value

volatile unsigned int timer_times = 0;

// 200 steps = 1 turn
// 10 turns = 12.5 cm
// 1 expance = 200 steps * 10 revolutions * 10 times = 20000 steps
int current_step = 0;
byte current_expance = 0;
const byte MAX_EXPANCES = 3;

volatile long unsigned eventTime;
volatile long unsigned previousEventTime;
volatile long unsigned timeSinceLastEvent;

long stepsTaken  = 0;
const long maximumRevolutions = STEPS_PER_REVOLUTION * 200;

State state = DO_NOTHING;
Stepper stepper(STEPS_PER_REVOLUTION, 3, 4, 5, 6);
DHT dht(DHTPIN, DHTTYPE); //// Initialize DHT sensor for normal 16mhz Arduino
Bounce btn_all_stop = Bounce(); 
Bounce btn_close = Bounce(); 
Bounce btn_open = Bounce(); 

void setup() {
  Serial.begin(9600);

  pinMode(BTN_ALL_STOP, INPUT_PULLUP);
  btn_all_stop.attach(BTN_ALL_STOP);
  btn_all_stop.interval(DEBOUNCE_INTERVAL_MS); // interval in ms
  
  pinMode(BTN_CLOSE, INPUT_PULLUP);
  btn_close.attach(BTN_CLOSE);
  btn_close.interval(DEBOUNCE_INTERVAL_MS); // interval in ms

  pinMode(BTN_OPEN, INPUT_PULLUP);
  btn_open.attach(BTN_OPEN);
  btn_open.interval(DEBOUNCE_INTERVAL_MS); // interval in ms

  stepper.setSpeed(180);
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
}

ISR(TIMER1_COMPA_vect){//timer1 interrupt 0.5Hz
  timer_times++;
  if(timer_times >= 300 && state == DO_NOTHING) {
    eventTime = millis();
    state = TAKE_MEASUREMENT;
    timer_times = 0;
  }
}

bool is_between(double value, byte lower, byte upper) {
  if(value > lower && value <= upper) {
    return true;
  } else {
    return false;
  }
}

void loop() {
  btn_all_stop.update();
  btn_close.update();
  btn_open.update();

  byte isAllStop = btn_all_stop.read();
  if(isAllStop == LOW) {
    state = ALL_STOP;
  }
  else {
    if(state == CLOSE) {
      state = CLOSE;
    }
    else {
      byte isClose = btn_close.read();
      if(isClose == LOW) {
        state = CLOSE;
      }
      else {
        byte isOpen = btn_open.read();
        if(isOpen == LOW) {
          state = OPEN;
        } else if(state != TAKE_MEASUREMENT || state != AUTO_OPEN) {
          state = DO_NOTHING;
        }
      }      
    }
  }
  switch(state)
    {
        case DO_NOTHING:
          ;    
          break;
        case ALL_STOP:
          stepsTaken = 0;
          current_expance = 0;
          current_step = 0;
          state = DO_NOTHING;
          break;
        case AUTO_OPEN:
          if(current_expance < MAX_EXPANCES) {
            current_step++;
            if(current_step >= 20000) {
              current_step = 0;
              current_expance++;
              state = DO_NOTHING; 
            } else {
              stepper.step(1);
            }
          } else {
            state = DO_NOTHING;
          }
          break;
        case TAKE_MEASUREMENT: 
          timeSinceLastEvent = eventTime - previousEventTime;
          previousEventTime = eventTime;

          Serial.print(timeSinceLastEvent);
          Serial.println(" ms: ");
          //Read data and store it to variables hum and temp
          hum = dht.readHumidity();
          temp = dht.readTemperature();
          //Print temp and humidity values to serial monitor
          Serial.print("Humidity: ");
          Serial.print(hum);
          Serial.print(" %, Temp: ");
          Serial.print(temp);
          Serial.println(" Celsius");

          if(temp <= 20) {
            state = CLOSE;            
          } else {
            if(is_between(temp, 20, 25) && current_expance == 0) {
              state = AUTO_OPEN;
            }
            else if(is_between(temp, 25, 30) && current_expance <= 1)  {
              state = AUTO_OPEN;
            } 
            else if(temp > 30 && current_expance <= 2)  {
              state = AUTO_OPEN;
            } 
            else {
              state = DO_NOTHING;
            }
          }
          Serial.print("State: ");
          Serial.println(state);
          Serial.print("Current step: ");
          Serial.println(current_step);
          Serial.print("Current expance: ");
          Serial.println(current_expance);
          
          break;
        case CLOSE: 
          stepper.step(-1);
          ;  
          break;
        case OPEN: 
          current_step++;
          if(current_step >= 20000) {
            current_step = 0;
            current_expance++;
          }
          stepper.step(1);
          ;   
          break;
        default: 
          ;
    }
}
