CC=gcc
CXX=g++
RM=rm -f

TARGET=../demps

INCLUDES=-I/usr/local/include -I/usr/local/include/osrm -I../include
LDFLAGS=-L/usr/local/lib
LDLIBS=-lGeographic -lCGAL -losrm -lboost_system -lboost_filesystem -lboost_iostreams -lboost_thread -lrt -ltbb -lpthread -lSDL

CXXFLAGS=-std=c++14 -Wall -O3 

DIR_OBJ=../obj

SRCS=$(wildcard *.cc)

OBJS=$(patsubst %.cc,$(DIR_OBJ)/%.o,$(SRCS))


all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) -o $@ $^ $(CXXFLAGS) $(LDLIBS) $(LDFLAGS) 

$(DIR_OBJ)/%.o: %.cc
	$(CXX) -c -o $@ $< $(CXXFLAGS) $(INCLUDES)

clean:
	$(RM) $(OBJS)
	$(RM) *~

distclean: clean
	$(RM) $(TARGET)

.PHONY: all clean distclean
	