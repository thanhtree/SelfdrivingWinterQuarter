
#include "car_control.h"


/**
 * Compile and Run:
 * g++ -Wall -pthread -o car_control_test car_control_test.cpp car_control.cpp -lpigpio -lrt
 * sudo ./car_control_test
*/

int main() {
    
    Car car;

    cout << "initialized car" << endl;
    for (int i = -60; i <= 60; i++) {
	car.setState(i,i*2);
	cout << "i*2: " << i*2 << endl;
	//car.printState();
	usleep(0.1*1000000);
    }
    car.setState(0,0);
    return 0;
}

