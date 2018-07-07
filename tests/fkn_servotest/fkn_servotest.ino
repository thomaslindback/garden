// a4988

const int STEP_PIN =  5;
const int DIR_PIN = 3;

void setup() 
{
  Serial.begin(9600);
  pinMode(STEP_PIN, OUTPUT); // Enable
  pinMode(DIR_PIN, OUTPUT); // Step
}

void loop() 
{
  int del = analogRead(A2);  
  
  delay(2000);
  del = map(del, 0,1023, 500, 10000);
  Serial.println(del);
  digitalWrite(DIR_PIN,HIGH); // Set Dir high
  Serial.println("Loop 200 steps (1 rev)");
  
  for(int i = 0; i < 200; i++) // Loop 200 times
  {
    digitalWrite(STEP_PIN,HIGH); // Output high
    delayMicroseconds(del); // Wait
    digitalWrite(STEP_PIN,LOW); // Output low
    delayMicroseconds(del); // Wait
  }
  
  Serial.println("Pause");
  delay(1000); // pause one second
  digitalWrite(DIR_PIN,LOW); // Set Dir high
  Serial.println("Loop 200 steps (1 rev)");
  
  for(int i = 0; i < 200; i++) // Loop 200 times
  {
    digitalWrite(STEP_PIN,HIGH); // Output high
    delayMicroseconds(del); // Wait
    digitalWrite(STEP_PIN,LOW); // Output low
    delayMicroseconds(del); // Wait
  }
  
  Serial.println("Pause");
  
  delay(1000); // pause one second
}
