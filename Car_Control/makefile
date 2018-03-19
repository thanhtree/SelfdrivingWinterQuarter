CC = g++
CFLAGS = -Wall -pthread 
SRCS = controller.cpp car_control.cpp joystick.cpp
PROG = controller

OPENCV = `pkg-config opencv --cflags --libs`
LIBS = $(OPENCV)
FLAGS = -lpigpio -lrt -std=gnu++11
$(PROG):$(SRCS)
	$(CC) $(CFLAGS) -o $(PROG) $(SRCS) $(LIBS) $(FLAGS)
