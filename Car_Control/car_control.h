#ifndef CAR_CONTROL_H
#define CAR_CONTROL_H
#include <stdio.h>
#include <pigpio.h>
#include <unistd.h>
#include <math.h>
#include <iostream>

using namespace std;

class Car {
private:
	int servoPin; // servo pin
	
	int f1; // forward pin of motor 1
	
	int r1; // reverse pin of motor 1
	
	int f2; // forward pin of motor 2
	
	int r2; // reverse pin of motor 2
	
	int fe; // forward enable pin
	
	int re; // reverse enable pin
	
	int zeroPulseWidth; // pulse width in microseconds which puts servo at 0 degrees (using 0 to 180 degrees scale)
	
	int maxSpeed; // cut off max speed to this value (must be between 0 to 100)

	double servoSeconds; // number of seconds to wait for servo to move to position
	
	int isForward; // is 1 if car is going >= 0, or it is 0 if car is going < 0
    
	
public:
	/**
	 * Constructor
	 */
	Car();
	
	/**
	 * Destructor
	 */
	~Car();
	
	/**
	 * set steering angle to a value between -60 to 60 degrees
	 */ 
	void setAngle(int angle);
	
	/**
	 * set speed from -255 to 255 (-100% to 100%)
	 * acceleration is by default 25500 but can be changed
         * if acceleration=255, car can go from 0 to 255 in 1 second
	 * if acceleration=25500, car can go from 0 to 255 in 0.01 seconds
	 * NOTE: you can only define default input values in header file
	 */ 
	void setSpeed(int speed, int acceleration=25500);
	
	/**
	 * returns current steering angle value between -60 to 60 degrees
	 */ 
	int getAngle();
	
	/**
	 * returns current speed value between -255 to 255
	 * NOTE: This function needs to return actual speed of car in inches/second,
	 * NOT the percentage speed
	 */ 	
	int getSpeed();
	
	/**
	 * uses setAngle() and setSpeed() to set angle and speed in one function call
	 */
	void setState(int angle, int speed, int acceleration=25500);
	
	/**
	 * uses getAngle() and getSpeed() to return angle and speed in one function call
	 * the return type is a pointer to a 1D array
	 * The array format: {angle, speed}
	 */
	int* getState();
	
	/**
	 * For Debugging Purposes:
	 * Prints out current state as: {angle, state}
	 */
	void printState();
	
	/**
	 * returns maximum speed of car
	 */
	int getMaxSpeed();
		
};

#endif /* CAR_CONTROL_H */


