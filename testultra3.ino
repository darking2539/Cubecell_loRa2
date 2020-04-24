long t1;
long t2;
long pulse_width;
float cm;
float inches;
const int TRIG_PIN = GPIO1;
const int ECHO_PIN = GPIO2;

// Anything over 400 cm (23200 us pulse) is "out of range"
const unsigned int MAX_DIST = 23200;

void setup() {

  // The Trigger pin will tell the sensor to range find
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);

  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);  //Open Vext
  
  // We'll use the serial monitor to view the sensor output
  Serial.begin(115200);
}

void loop() {
  
    readlength();
    // Print out results
    
    Serial.print("pulse_width: ");
    Serial.println(pulse_width);
    
    Serial.print("t1: ");
    Serial.println(t1);
    
    Serial.print("t2: ");
    Serial.println(t2);
    

  
  // Wait at least 500ms before next measurement
  delay(500);
}

void readlength() {

  // Hold the trigger pin high for at least 10 us
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Wait for pulse on echo pin
  while ( digitalRead(ECHO_PIN) == 0 );

  // Measure how long the echo pin was held high (pulse width)
  // Note: th micros() counter will overflow after ~70 min
  t1 = CySysTickGetValue();
  while ( digitalRead(ECHO_PIN) == 1);
  t2 = CySysTickGetValue();
  
  if (t2<t1){
    t2 = 48000 + t2;
  }
  
  pulse_width = t2 - t1;

  // Calculate distance in centimeters and inches. The constants
  // are found in the datasheet, and calculated from the assumed speed
  //of sound in air at sea level (~340 m/s).
  cm = pulse_width / 58.0;

}
