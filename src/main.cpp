// Include Wire Library for I2C
#include <SPI.h>
#include <Wire.h>
 
// Include Adafruit PCA9685 Servo Library
#include <Adafruit_PWMServoDriver.h>
 
// Creat object to represent PCA9685 at default I2C address
Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(0x40);
 
// Define maximum and minimum number of "ticks" for the servo motors
// Range from 0 to 4095
// This determines the pulse width
 
#define SERVOMIN  103  // Minimum value
#define SERVOMAX  410  // Maximum value
 
// Define servo motor connections (expand as required)
#define SER0  0   //Servo Motor 0 on connector 0
#define SER1  15  //Servo Motor 1 on connector 12
#define SER2  14  //Servo Motor 1 on connector 12

#define HIP  10   //Servo Motor 0 on connector 0
#define ELBO  9  //Servo Motor 1 on connector 12
#define PAlM  8  //Servo Motor 1 on connector 12
 
// Variables for Servo Motor positions (expand as required)
int pwm0;
int pwm1;

void writeServo(int num, byte value){
    // Write to PCA9685
	pca9685.setPin(num, map(value, 0, 180, SERVOMIN, SERVOMAX));
}

void step(){
	writeServo(ELBO, 90);
	writeServo(PAlM, 0);
	delay(100);
	writeServo(HIP, 160);
	delay(100);
	writeServo(ELBO, 160);
	writeServo(PAlM, 45);
}

void forward(){
	writeServo(HIP, 20);
}

void setup() {
	
	// Serial monitor setup
	Serial.begin(115200);
	
	// Print to monitor
	Serial.println("PCA9685 Servo Test");
	
	// Initialize PCA9685
	pca9685.begin();
	
	// Set PWM Frequency to 50Hz
	pca9685.setPWMFreq(50);
	writeServo(SER0, 90);
	writeServo(SER1, 90);
}
 
void loop() {
	step();
	delay(300);
	forward();
	delay(500);
}

