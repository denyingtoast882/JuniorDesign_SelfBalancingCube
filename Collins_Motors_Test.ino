
// basic motor test
// no encoder/buzzer functionality

#include <Arduino.h>
#include "driver/ledc.h"      // https://docs.espressif.com/projects/arduino-esp32/en/latest/api/ledc.html?highlight=ledcWrite
#include "esp32-hal-ledc.h"

#define BRAKE       26  // brake pin
#define BUZZER      27  // buzzer pin

#define DIR1        4   // motor 1 cw/ccw
#define ENC1_1      35  // motor 1 encoder A input
#define ENC1_2      33  // motor 1 encoder B input
#define PWM1        32  // motor 1 PWM
#define PWM1_CH     0   // motor 1 PWM channel

#define DIR2        15  // motor 2 cw/ccw
#define ENC2_1      13  // motor 2 encoder A input
#define ENC2_2      14  // motor 2 encoder B input
#define PWM2        25  // motor 2 PWM
#define PWM2_CH     1   // motor 2 PWM channel

#define DIR3        5   // motor 3 cw/ccw
#define ENC3_1      16  // motor 3 encoder A input
#define ENC3_2      17  // motor 3 encoder B input
#define PWM3        18  // motor 3 PWM
#define PWM3_CH     2   // motor 3 PWM channel

#define BASE_FREQ  5000 // 5k Hz for PWM frequency
#define RESOLUTION  8   // 8 bit resolution, 0-255

// function prototypes
void pwmSet(uint8_t channel, uint8_t duty);  // sets pwm duty cylce in a certain channel
void Motor1_control(uint8_t speed, bool direction);          // sets speed and direction for motor 1, 0 = ccw, 0 <= speed <= 255
void Motor2_control(uint8_t speed, bool direction);          // sets speed and direction for motor 2, 0 = ccw, 0 <= speed <= 255
void Motor2_control(uint8_t speed, bool direction);          // sets speed and direction for motor 3, 0 = ccw, 0 <= speed <= 255
void MotorAll_control(uint8_t speed, bool direction);        // setes speed and direction for all motors, 0 = ccw, 0 <= speed <= 255

void setup()
{
  Serial.begin(115200); // communication of 115200 bits per second

  // the following sets up the pins as outputs or inputs
  // misc
  pinMode(BUZZER, OUTPUT);  // pin 27 is output
  pinMode(BRAKE, OUTPUT);   // pin 26 is output
  // cw/ccw
  pinMode(DIR1, OUTPUT);    // pin 4 is output
  pinMode(DIR2, OUTPUT);    // pin 15 is output
  pinMode(DIR3, OUTPUT);    // pin 5 is output
  // encoders
  pinMode(ENC1_1, INPUT);   // pin 35 in input
  pinMode(ENC1_2, INPUT);   // pin 33 is input
  pinMode(ENC2_1, INPUT);   // pin 13 in input
  pinMode(ENC2_2, INPUT);   // pin 14 is input
  pinMode(ENC3_1, INPUT);   // pin 16 in input
  pinMode(ENC3_2, INPUT);   // pin 17 is input

  // the following sets up the pin, operating frequency, the resolution of the signal, and channel
  ledcAttachChannel(PWM1, BASE_FREQ, RESOLUTION, PWM1_CH);  // PWM1 setup
  ledcAttachChannel(PWM2, BASE_FREQ, RESOLUTION, PWM2_CH);  // PWM2 setup
  ledcAttachChannel(PWM3, BASE_FREQ, RESOLUTION, PWM3_CH);  // PWM3 setup

}

// main
void loop()
{
  digitalWrite(BRAKE, HIGH);  // brake off
  // motor 1 routine
  Motor1_control(127, 0);        // motor 1 half speed, ccw
  delay(1000);
  Motor1_control(127, 1);       // motor 1 half speed, cw
  delay(1000);
  Motor1_control(255, 0);        // motor 1 full speed, ccw
  delay(1000);
  Motor1_control(255, 1);       // motor 1 full speed, cw
  delay(1000);
  Motor1_control(1, 0);          // motor 1 no speed
  delay(1000);                 
  // motor 2 routine
  Motor2_control(127, 0);       // motor 2 half speed, cw
  delay(1000);
  Motor2_control(127, 1);       // motor 2 half speed, ccw
  delay(1000);
  Motor2_control(255, 0);       // motor 2 full speed, cw
  delay(1000);
  Motor2_control(255, 1);       // motor 2 full speed, ccw
  delay(1000);
  Motor2_control(0, 0);         // motor 2 no speed
  delay(1000);           
  // motor 3 routine
  Motor3_control(127, 0);       // motor 3 half speed, cw
  delay(1000);
  Motor3_control(127, 1);       // motor 3 half speed, ccw
  delay(1000);
  Motor3_control(255, 0);       // motor 3 full speed, cw
  delay(1000);
  Motor3_control(255, 1);       // motor 3 full speed, ccw
  delay(1000);
  Motor3_control(0, 0);         // motor 3 no speed
  delay(1000);           
  // all motor routine
  MotorAll_control(127,0);      // all motors half speed, cw
  delay(1000);
  MotorAll_control(127, 1);     // all motors half speed, ccw
  delay(1000);
  MotorAll_control(255,0);      // all motors full speed, cw
  delay(1000);
  MotorAll_control(255,1);     // all mototrs full speed, ccw
  delay(1000);          
  //digitalWrite(BRAKE,LOW);    // brake on
  //delay(1000);                  

}

// functions
void pwmSet(uint8_t channel, uint8_t duty)
{
  ledcWrite(channel, duty);  // essentially the same function as 'pwmSet' but without the 'uint8_t' needing to be typed everytime
}

void Motor1_control(uint8_t speed, bool direction)
{
  if (direction == 0){
    digitalWrite(DIR1, LOW);}       // if speed is +, ccw
  else{
    digitalWrite(DIR1, HIGH);}      // if speed is -, cw
  ledcWrite(PWM1, 255-speed);  // sets speed of motor. lower = higher?
}

void Motor2_control(uint8_t speed, bool direction)
{
  if (direction == 0){
    digitalWrite(DIR2, LOW);}       // if speed is +, cw
  else{
    digitalWrite(DIR2, HIGH);}      // if speed is -, ccw
  ledcWrite(PWM2, 255-speed);  // sets speed of motor. lower = higher?
}

void Motor3_control(uint8_t speed, bool direction)
{
  if (direction == 0){
    digitalWrite(DIR3, LOW);}       // if speed is +, cw
  else{
    digitalWrite(DIR3, HIGH);}      // if speed is -, ccw
  ledcWrite(PWM3, 255-speed);  // sets speed of motor. lower = higher?
}

void MotorAll_control(uint8_t speed, bool direction)
{
  if (direction == 0){
    digitalWrite(DIR1, LOW);        // if speed is +, cw
    digitalWrite(DIR2, LOW);
    digitalWrite(DIR3, LOW);}       
  else{
    digitalWrite(DIR1, HIGH);       // if speed is -, ccw
    digitalWrite(DIR2, HIGH);
    digitalWrite(DIR3, HIGH);}      
  ledcWrite(PWM1, 255-speed);  // sets speed of motor. lower = higher?
  ledcWrite(PWM2, 255-speed);
  ledcWrite(PWM3, 255-speed);  
}
