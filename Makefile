CFLAGS = $(shell pkg-config --cflags sdl3)
LDFLAGS = $(shell pkg-config --libs sdl3)

all: clean build

build:
	g++ main.cpp -o main -g -std=c++11 $(CFLAGS) $(LDFLAGS)

clean:
	rm -f main