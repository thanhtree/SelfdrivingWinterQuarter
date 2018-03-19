CC = g++
CFLAGS = -std=c++11 -Wall

all: program

lld.o: lld.cpp
	$(CC) $(CFLAGS) -c lld.cpp 

link:
	$(CC) lld.o -o program.out

program: lld.o link

clean:
	rm -rf *.o *.out .*.*.swo .*.*.swp *.exe
