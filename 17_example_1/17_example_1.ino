#include <Servo.h>

// Arduino pin assignment
#define PIN_LED 9
#define PIN_POTENTIOMETER 3 // Potentiometer at Pin A3
// Add IR Sensor Definition Here !!!
#define PIN_IR 0 // IR at Pin A0
#define PIN_SERVO 10

#define _DUTY_MIN 553  // servo full clock-wise position (0 degree)
#define _DUTY_NEU 1476 // servo neutral position (90 degree)
#define _DUTY_MAX 2399 // servo full counter-clockwise position (180 degree)
#define DIST_MIN 100 // minimum distance to be measured (unit: mm)
#define DIST_MAX 250 //maximum distance to be measured (unit: mm)

#define _EMA_ALPHA 0.4

#define LOOP_INTERVAL 30   // Loop Interval (unit: msec)

Servo myservo;
unsigned long last_loop_time;   // unit: msec
float dist_prev = DIST_MIN;
float dist_ema;

void setup()
{
  pinMode(PIN_LED, OUTPUT);
  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(_DUTY_NEU);
  
  dist_prev = DIST_MIN;

  Serial.begin(1000000);
}

void loop()
{
  float dist_raw;
  unsigned long time_curr = millis();
  int a_value, duty, dist;

  // wait until next event time
  if (time_curr < (last_loop_time + LOOP_INTERVAL))
    return;
  last_loop_time += LOOP_INTERVAL;

  // Read IR Sensor value !!!
  a_value = analogRead(PIN_IR);
  // Convert IR sensor value into distance !!!
  dist = (6762.0/(a_value-9)-4.0)*10.0-60.0;

  // we need distance range filter here !!!
  if (dist < DIST_MIN) {
    dist = dist_prev;    // Set Lower Value
    digitalWrite(PIN_LED, 1);       // LED OFF
  } else if (dist > DIST_MAX) {
    dist = dist_prev;    // Set Higher Value
    digitalWrite(PIN_LED, 1);       // LED OFF
  } else {    // In desired Range
    digitalWrite(PIN_LED, 0);       // LED ON  
    dist_prev = dist;
  }

  // we need EMA filter here !!!
  dist_ema = _EMA_ALPHA * dist + (1-_EMA_ALPHA) * dist_ema;

  // map distance into duty
  //duty = map(a_value, 0, 1023, _DUTY_MIN, _DUTY_MAX);
  duty = (dist_ema - 100)*(_DUTY_MAX - _DUTY_MIN)/150 + _DUTY_MIN;
  myservo.writeMicroseconds(duty);

  // print IR sensor value, distnace, duty !!!
  Serial.print("Min:");    Serial.print(DIST_MIN);
  Serial.print(",IR:");  Serial.print(a_value);
  Serial.print(",dist:");  Serial.print(dist);
  Serial.print(",ema:");  Serial.print(dist_ema);
  Serial.print(",servo:"); Serial.print(duty);  
  Serial.print(",Max:");   Serial.print(DIST_MAX);
  Serial.println("");
}
