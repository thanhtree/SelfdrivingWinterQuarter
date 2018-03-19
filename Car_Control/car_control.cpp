#include "car_control.h"

Car::Car() {
	gpioInitialise();

	servoPin = 24; // servo pin
	
	f1 = 13; // forward pin of motor 1
	
	r1 = 6; // reverse pin of motor 1
	
	f2 = 21; // forward pin of motor 2
	
	r2 = 20; // reverse pin of motor 2
	
	fe = 12; // forward enable pin
	
	re = 26; // reverse enable pin
	
	zeroPulseWidth = 925; // pulse width in microseconds which puts servo at 0 degrees (using 0 to 180 degrees scale)
	
	maxSpeed = 40; // cut off max speed to this value (must be between 0% to 100%)

	servoSeconds = 0.001; // number of seconds to wait for servo to move to position
	
    
    /*
     * Initialize pins as outptus
     */
    if (gpioInitialise() >= 0) {
		gpioSetMode(servoPin, PI_OUTPUT);
		
		gpioSetMode(f1, PI_OUTPUT);
		gpioSetMode(r1, PI_OUTPUT);
		gpioSetMode(f2, PI_OUTPUT);
		gpioSetMode(r2, PI_OUTPUT);

		gpioSetMode(fe, PI_OUTPUT);
		gpioSetMode(re, PI_OUTPUT);

		/*
		 * initialize servo to center and speed to zero
		 */
		int pulseWidth = round(90*5.555555555555555555555555556 + zeroPulseWidth);
		gpioServo(servoPin, pulseWidth);
		
		gpioPWM(fe, 255);
		gpioPWM(re, 255);
		
		gpioPWM(f1, 0);
		gpioPWM(f2, 0);

		isForward = 1; // if car is going >= 0 speed, then it is going forward
		
		// you have to sleep so servo has time to move to position.
		// after much experimentation, I've found that waiting 0.15 seconds is minimum,
		// assuming we want servo to be able to turn 90 degrees with one function call.
		// I wouldn't make seconds larger than 0.2.
 
		usleep(servoSeconds*1000000);


    }

    else {
	cout << "Error - car_control.cpp Car::Car() gpioInitialise() failed" << endl;
    }

}

Car::~Car() {

	/*
	 * finalize servo to center and speed to zero
	 */
	int pulseWidth = round(90*5.555555555555555555555555556 + zeroPulseWidth);
	gpioServo(servoPin, pulseWidth);
	
	gpioPWM(fe, 255);
	gpioPWM(re, 255);
	
	gpioPWM(f1, 0);
	gpioPWM(f2, 0);
	
	// you have to sleep so servo has time to move to position
	usleep(servoSeconds*1000000);
	gpioTerminate(); // free gpio resources
}

void Car::setAngle(int angle) {
	if (gpioInitialise() < 0) {
		fprintf(stderr, "PiGPIO initialisation failed.\n\r");
	}
	else {
		//gpioSetMode(servoPin, PI_OUTPUT);
		int pulseWidth;
		// car chassis blocks wheels from turning too much
		// limit angle range from -60 to 60 degrees
		if (angle < -60) {
			angle = -60;
		}
		else if (angle > 60) {
			angle = 60;
		}
		
		angle = angle + 90; // convert from -90 to 90 range to 0 to 180 range
		pulseWidth = round(angle*5.555555555555555555555555556 + zeroPulseWidth);
		gpioServo(servoPin, pulseWidth);

		usleep(int(servoSeconds*1000000)); // sleeps for number of microseconds
	}
}

void Car::setSpeed(int speed, int acceleration) {

	if (gpioInitialise() < 0) {
		fprintf(stderr, "PiGPIO initialisation failed.\n\r");
	}

	else {

		// make sure speed is within -100 to 100
		if (speed > 255*(maxSpeed/100.0)) {
			//speed = 255;
			speed = round(255*(maxSpeed/100.0));
			cout << "hit max speed" << endl;
		}
		else if (speed < -255*(maxSpeed/100.0)) {
			//speed = -255;
			speed = -1*round(255*(maxSpeed/100.0));
			cout << "hit max speed" << endl;
		}
		//speed = 255*(speed/100.0)*(maxSpeed/100.0);
		//cout << "speed: " << speed << endl;
		// gpioPWM() takes in a range from 0 to 255

		//cout << "speed before conversion: " << speed << endl;
		//speed = round((speed/100.0)*(maxSpeed/100.0)*255.0);
		//speed = round((maxSpeed/100.0)*speed);
		//cout << "speed after conversion: " << speed << endl;

		double seconds = 1.0/acceleration; // number of seconds to change speed by 1 unit speed
		//cout << "seconds: " << seconds << endl;
		int previousSpeed = -256;
		int currentSpeed;
		

		// we don't know if car is currently going forward or rerverse
		// so we need to check both the forward and reverse pins
		//int previousSpeedForward = gpioGetPWMdutycycle(21);
		//int previousSpeedReverse = gpioGetPWMdutycycle(20);
		// if pwm from forward pin is valid, then use that as previousSpeed
		//if (previousSpeedForward >= 0 && previousSpeedForward <= 255) {
		//cout << "isForward: " << isForward << endl;
		if (isForward == 1 && gpioGetPWMdutycycle(f2) >= 0) {
			//cout << "Previous speed was forward" << endl;
			
			previousSpeed = gpioGetPWMdutycycle(f2);

			//cout << "previousSpeed: " << previousSpeed << endl << endl;


		}

		// if pwm from reverse pin is valid, then use that as previousSpeed
		//if (previousSpeedReverse >= 0 && previousSpeedReverse <= 255) {
		else if (isForward == 0 && gpioGetPWMdutycycle(r2) >= 0) {
			//cout << "Previous speed was reverse" << endl;
			previousSpeed = -1*gpioGetPWMdutycycle(r2);

			//cout << "previousSpeed: " << previousSpeed << endl << endl;

		}
		// make sure previousSpeed was updated
		// if not, then neither the forward or reverse pin was initialized
		if (previousSpeed >= -255 && previousSpeed <= 255) {
		    if (previousSpeed <= speed) {

			// going from small to big speed
			for (currentSpeed = previousSpeed; currentSpeed <= speed; currentSpeed++) {
			    gpioPWM(fe, 255);
			    gpioPWM(re, 255);

			    // if car was previously going forward, then use forward pins
			    if (currentSpeed >= 0) {
				gpioPWM(f1, currentSpeed);
				gpioPWM(f2, currentSpeed);
				isForward = 1;
			    }
			    // if car was previously going reverse, then use reverse pins
			    else {
				    gpioPWM(r1, -1*currentSpeed);
				    gpioPWM(r2, -1*currentSpeed);
				    isForward = 0;
			    }

			    usleep(int(seconds*1000000));

			}

		    }
		    // if previousSpeed is greater than desired speed
		    // slowly decrease speed to desired speed
		    else if (previousSpeed > speed) {
			//cout << "Previous speed was greater than desired speed" << endl;
			//cout << "Decreasing speed" << endl << endl;
			// going from big to small speed
			for (currentSpeed = previousSpeed; currentSpeed >= speed; currentSpeed--) {
			    // if we are about to go from forward to reverse
			    // then we need to change gpio pins
			    //cout << "currentSpeed: " << currentSpeed << endl;
			    gpioPWM(fe, 255);
			    gpioPWM(re, 255);

			    if (currentSpeed >= 0) {
				gpioPWM(f1, currentSpeed);
				gpioPWM(f2, currentSpeed);
				isForward = 1;
			    }
			    else {
				    
				    gpioPWM(r1, -1*currentSpeed);
				    gpioPWM(r2, -1*currentSpeed);
				    isForward = 0;
			    }

			    usleep(int(seconds*1000000));
			}
			    

		    }
		}

		else {
			//cout << "Neither pin was valid" << endl;
			gpioPWM(fe, 255);
			gpioPWM(re, 255);
			if (speed >= 0) {

				gpioPWM(f1, speed);
				gpioPWM(f2, speed);
				isForward = 1;
				//cout << "isForward: " << isForward << endl;
			}
			else {
				gpioPWM(r1, -1*speed);
				gpioPWM(r2, -1*speed);
				isForward = 0;
				//cout << "isForward: " << isForward << endl;
			}
			

		}
		



		}


}

int Car::getAngle() {

    int pulseWidth = gpioGetServoPulsewidth(servoPin);
    int angle = round((pulseWidth - zeroPulseWidth)/5.555555555555555555555555556);
    angle = angle - 90;
    return angle;
}

int Car::getSpeed() {
    //round((maxSpeed/100.0)*255.0);
    if (isForward == 1) {
	    //return round(((gpioGetPWMdutycycle(f2)/255.0)*100.0*100.0)/maxSpeed);
	    return gpioGetPWMdutycycle(f2);
    }
    else if (isForward == 0) {
	    //return -1*round(((gpioGetPWMdutycycle(r2)/255.0)*100.0*100.0)/maxSpeed);
	    return -1*gpioGetPWMdutycycle(r2);
    }
    else {
	return 0;
    }
}

void Car::setState(int angle, int speed, int acceleration) {
    this->setAngle(angle);
    this->setSpeed(speed, acceleration);
}

int* Car::getState() {
    // NOTE: you cannot return the address of a local variable
    // make the variable static
    static int state[2];
    state[0] = this->getAngle();
    state[1] = this->getSpeed();
    return state;

}

void Car::printState() {
    int* state;
    state = this->getState();
    cout << "{Angle: " << state[0] << ", Speed: " << state[1] << "}" << endl;
}

int Car::getMaxSpeed() {
    return this->maxSpeed;
}
