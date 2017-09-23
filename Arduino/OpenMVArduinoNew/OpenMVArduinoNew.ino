
/* Miminimum sketch to translate incoming serial from an OpenMV board to Ardunio commands for motor and servo
 *  Written by Chris Anderson, DIYRobocars, 2017
 */

#include <PWMServo.h> //servo library
#include <AltSoftSerial.h>

#include <PinChangeInterrupt.h>

AltSoftSerial altSerial;

PWMServo myservoa, myservob; // create servo objects to control servos
int SteeringPin = 10;  // This corresponds to Servo Output 1
int MotorPin = 6;  // This correspondes to Servo Output 2
int CH3 = 5; // Corresponds to Servo Output 3
int CH4 = 4; // Corresponds to Servo Output 4
int CH5 = 9; // Corresponds to Servo Outut 5
int steer, motor;

/*
 * Define pins used to provide RC PWM signal to Arduino
 * Pins 8, 9 and 10 are used since they work on both ATMega328 and 
 * ATMega32u4 board. So this code will work on Uno/Mini/Nano/Micro/Leonardo
 * See PinChangeInterrupt documentation for usable pins on other boards
 */
const byte channel_pin[] = {8, 9, 10};
volatile unsigned long rising_start[] = {0, 0, 0};
volatile long channel_length[] = {0, 0, 0};


unsigned long time;
unsigned long lasttime = 0;
bool LEDState = LOW;

const int BAUD_RATE = 9600;

void setup() {
    Serial.begin(BAUD_RATE);
    myservoa.attach(SteeringPin);// attach servo on Output 1 to servo object
    myservob.attach(MotorPin);// attach servo on Output 2 to servo object
    altSerial.begin(9600);   // This is for the debug console
    altSerial.println("Hello, world");
    pinMode(LED_BUILTIN, OUTPUT);  // enable LED 
      pinMode(channel_pin[0], INPUT);
    pinMode(channel_pin[1], INPUT);
    pinMode(channel_pin[2], INPUT);

    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(channel_pin[0]), onRising0, CHANGE);
    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(channel_pin[1]), onRising1, CHANGE);
    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(channel_pin[2]), onRising2, CHANGE);
}

void processPin(byte pin) {
  uint8_t trigger = getPinChangeInterruptTrigger(digitalPinToPCINT(channel_pin[pin]));

  if(trigger == RISING) {
    rising_start[pin] = micros();
  } else if(trigger == FALLING) {
    channel_length[pin] = micros() - rising_start[pin];
  }
}

void onRising0(void) {
  processPin(0);
}

void onRising1(void) {
  processPin(1);
}

void onRising2(void) {
  processPin(2);
}
void loop() {
  time = millis();
  if (time > lasttime + 1000) {   // flash the LED every second to show "resting" mode
      lasttime = time;
      LEDState = !LEDState; // reverse the LED state
      digitalWrite(LED_BUILTIN, LEDState);   // turn on or off the LED
       }
  while (Serial.available() > 0) {
   // look for the next valid integer in the incoming serial stream:
    int tempsteer = Serial.parseInt();
//    mySerial.println(steer);   // turn this on for debgging. Warning: it will make the servos pulse
    // do it again:
    int tempmotor = Serial.parseInt();
//    mySerial.println(motor);   // turn this on for debgging. Warning: it will make the servos pulse
    // look for the newline. That's the end of the commands
    if (Serial.read() == '\n') {
        steer=tempsteer;
        motor=tempmotor;
        LEDState = !LEDState; // reverse the LED state
        digitalWrite(LED_BUILTIN, LEDState);   // turn on or off the LED to show activity
    }
  }
  myservoa.write(steer); // send values to output
  myservob.write(motor);
  delay (20);
}



