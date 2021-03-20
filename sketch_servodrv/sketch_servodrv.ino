#include "Arduino.h"
#include <Wire.h>
#include <adafruit_pwmservodriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN 600 // official 0 degress position (in us)
#define SERVOMAX 2400 // official 180 degress position (in us)
#define FREQ 60 // pulses per second
#define BIT_RANGE 4096 // bit range of pulse
#define USEC 1000000 // usec per second

const byte GRIPPER = 5;
const byte WRIST = 4;
const byte NECK = 3;
const byte WAIST = 2;
const byte LEGS = 1;
const byte BASE = 0;

const byte SERVO_COUNT = 6;

// Servos position by default
byte servoDefault[SERVO_COUNT] = {90, 90, 90, 90, 0, 90};
byte servoPos[SERVO_COUNT];
boolean firstStroke = true;

// Range of each particular servo in us
int servoFloor[SERVO_COUNT] = {560, 700, 700, 500, 550, 570};
int servoCeiling[SERVO_COUNT] = {2570, 2500, 2850, 2500, 2400, 2750};

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(1000);
  Serial.println("ServoDriver");
  Serial.println("Setting up hardware .....");
  pwm.begin();
  pwm.setPWMFreq(60);
  Serial.println("............... completed");
}

// There is nothing to do in loop
void loop() {
  ;
}

int roundDouble(double x) {
  x = x + 0.5;
  int y = (int) x;
  return y;
}

// Get pulse length in bits out of usecs
int bitPulse(double usec) {
  double result;
  
  result = FREQ;
  result *= BIT_RANGE;
  result /= USEC;
  result *= usec;
  return roundDouble(result);
}

// Get pulse length in usec out of bits
double usPulse(int bits) {
  double result;
  
  result = USEC; // 1000000 usec per second
  result /= FREQ; // usec per pulse
  result /= BIT_RANGE; // usec per pulse per bit
  result *= bits;
  return result;
}

//Sets all servos on their default positions
void moveAllDefault() {
  Serial.println("Setting servos by default...");
  for (int i = 0; i < SERVO_COUNT; i++) {
    int pulseLength = map(servoDefault[i], 0, 180, 
                          servoFloor[i], servoCeiling[i]);
    pulseLength = bitPulse((double) pulseLength);
    pwm.setPWM((SERVO_COUNT-1) - i, 0, pulseLength);
    servoPos[i] = servoDefault[i];
    //delay(1000);
  }
  Serial.println("Done...");
}

// Moves appropriate servo by given rotational angle (0 <= alpha <= 180)
byte moveServo(byte servo, byte alpha) {
  if (alpha == 254)
    return 0;
  if (alpha == 255)
    alpha = servoDefault[servo];
  
  int angle = servoPos[servo];
  if (angle == alpha) return 0;
  int i = alpha - angle > 0 ? 1 : -1;
  do {
    delay(10);
    angle += i;
    int pulseLength = map(angle, 0, 180, servoFloor[servo], 
                        servoCeiling[servo]); // In scale of us
    pulseLength = bitPulse((double) pulseLength); // In scale of bits
    pwm.setPWM((SERVO_COUNT-1) - servo, 0, pulseLength);
  } while (angle != alpha);
  servoPos[servo] = alpha;
  return 1;
}

void serialEvent() {
  while (Serial.available() > 0) {
    char bytes[SERVO_COUNT];
    int b = Serial.readBytes(bytes, SERVO_COUNT);
    if (b < SERVO_COUNT) {
      Serial.println("Insufficient number of received bytes");
      return;
    }
    if (firstStroke) {
      moveAllDefault();
      firstStroke = false;
      return;
    }
    
    for (int i = 0; i < SERVO_COUNT; i++) {
      if ( moveServo(i, byte(bytes[i])) == 1 )
        ;
        //delay(1000);
    }
    Serial.print("Done");
  }
}
