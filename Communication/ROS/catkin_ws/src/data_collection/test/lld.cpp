// test.cpp
//
// Simulates the output of the C++ implementation of
// the lane line detection algorithm
//

#include <iostream>
#include <cstdlib>
#include "zhelpers.hpp"
#include <stdint.h>

using namespace std;

int main () {
  zmq::context_t context(1);

  // subscriber tells us when it's ready here
  zmq::socket_t sync(context, ZMQ_PULL);
  sync.bind("tcp://*:5564");

  // we send updates via this socket
  zmq::socket_t publisher(context, ZMQ_PUB);

  // prevent publisher overflow from slow subscribers
  uint64_t hwm = 1;
  publisher.setsockopt(ZMQ_HWM, &hwm, sizeof(hwm));

  // specify swap space in bytes, this covers all subscribers
  uint64_t swap = 25000000;
  publisher.setsockopt(ZMQ_SWAP, &swap, sizeof (swap));
  publisher.bind("tcp://*:5565");

  // wait for synchronization request
  s_recv(sync);

  // algorithm output
  float center_offset;
  float left_radius_of_curvature;
  float right_radius_of_curvature; 

  while (1) {
    center_offset = rand() * -60;
    left_radius_of_curvature = rand() * 180;
    right_radius_of_curvature = rand() * 180; 
    cout << "center_offset: " << center_offset << endl;
    cout << "left_radius_of_curvature: " << left_radius_of_curvature << endl;
    cout << "right_radius_of_curvature: " << right_radius_of_curvature << endl;
    s_send(publisher, .str());
    s_send(publisher, "END");
    sleep(1);
  } 

 
  return 0; 
}


