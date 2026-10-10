CFLAGS = $(shell pkg-config --cflags sdl3)
LDFLAGS = $(shell pkg-config --libs sdl3)

all: clean build

testbuild:
	g++ main.cpp -o main -g -std=c++11 $(CFLAGS) $(LDFLAGS)

build:
	g++ main.cpp -O2 -o main -g -std=c++11 $(CFLAGS) $(LDFLAGS)

clean:
	rm -f main