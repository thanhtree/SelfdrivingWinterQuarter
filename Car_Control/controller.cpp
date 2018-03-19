#include "joystick.hpp"
#include "car_control.h"
#include <unistd.h>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <fstream>

using namespace std;
//using namespace cv;

/**
 * Compile and Run:
 * g++ $(pkg-config --libs --cflags opencv) -Wall -pthread -o controller controller.cpp joystick.cpp car_control.cpp -lpigpio -lrt -std=gnu++11
 * sudo ./controller
 *
 * joystick.event = 1 is for vertical axis of left joystick
 * all the way down is 32767; all the way up is -32767
 *
 * joystick.event = 2 is for horizontal axis of right joystick
 * all the way to the left is -32767; all the way to the right is 32767
 *
 * left joystick in vertical direction controls speed
 * right joystick in horizontal position controls steering angle
 * press triangle button to exit the program
 *
 */


int main(int argc, char** argv)
{

  /**
   * Setup camera
   */

  cv::VideoCapture cap(0);

  int frame_height = 180;
  int frame_width = 320;
  int count = 0; // number of frames

  cap.set(3,frame_width);
  cap.set(4,frame_height);

  if (!cap.isOpened()) {
    cerr << "ERROR: Unable to open the camera" << endl;
    return 1;
  }

  cv::Mat frame;

  /**
   * Setup ps4 controller
   */

  // Create an instance of Joystick
  Joystick joystick("/dev/input/js0");


  // Ensure that it was found and that we can use it
  if (!joystick.isFound())
  {
    printf("open failed.\n");
    exit(1);
  }

  /**
   * Setup output log
   */

  int output_log_number = 14; // which output_log to save to
  string save_output_log_path = "output_logs/output_log" + to_string(output_log_number) + "/output_log" + to_string(output_log_number) + ".txt";
  ofstream output_log;
  output_log.open(save_output_log_path);


  /**
   * Setup car
   */

  int speed, angle; //instantiate speed and angle
  Car car; //instantiate car object
  
  

  /**
   * Begin main program
   */
  while (true)
  {
    // Restrict rate
    usleep(1000);

    // Attempt to sample an event from the joystick
    JoystickEvent event;
    if (joystick.sample(&event))
    {
      if (event.isButton())
      {
	//if triangle button is pressed, terminate the program
	if (event.number == 3) {
	    if (event.value == 1) {
		output_log.close();
		car.setState(0,0);
		cout << "Triangle button pressed" << endl;
		cout << "event.number: " << event.number << endl;
		printf("Exiting program\n");

		exit(0);
	    }
	}

      }
      else if (event.isAxis())
      {
	// vertical axis using left joystick
	if (event.number == 1) {
	    // speed range is from -255*(maxSpeed/100) to 255*(maxSpeed/100)
	    speed = -1*round((event.value/32767.0)*255.0*(car.getMaxSpeed()/100.0));
	    //cout << "speed: " << speed << endl;
	    car.setSpeed(speed);
	}
	// horizontal axis using right joystick
	if (event.number == 2) {
	    // angle range is from -60 to 60
	    angle = round((event.value/32767.0)*60.0); // multiply by negative one because car is upside down
	    //cout << "angle: " << angle << endl;
	    car.setAngle(angle);
	}
	cap >> frame;
	if (frame.empty()) {
	    car.setState(0,0);
	    cerr << "ERROR: Unable to grab from the camera" << endl;
	    break;
	}

	//path to save image
	string save_image_path = "output_logs/output_log" + to_string(output_log_number) + "/frames/frame" + to_string(count) + ".jpg";

	//cv::imshow("Live",frame);
	cv::imwrite(save_image_path,frame);
	count += 1; // increment frame count
	output_log << angle << "\t" << car.getSpeed() << "\t" << save_image_path << "\n";

	car.printState();


      }

    }

  }

}
