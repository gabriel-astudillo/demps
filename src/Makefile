CC=gcc
CXX=g++
RM=rm -f

TARGET=../demps

INCLUDES=-I/usr/local/include -I/usr/local/include/osrm -I../include
LDFLAGS=-L/usr/local/lib -L/home/gabriel/demps/dependencias/tbb/build/linux_intel64_gcc_cc7_libc2.27_kernel4.15.0_release
LDLIBS=-lGeographic -lCGAL -losrm -lboost_system -lboost_filesystem -lboost_iostreams -lboost_thread -lrt -ltbb -lpthread -fopenmp -lgmp

CXXFLAGS=-std=c++14 -Wall -O2 -fopenmp

DIR_OBJ=objs

SRCS=$(wildcard *.cc)

OBJS=$(patsubst %.cc,$(DIR_OBJ)/%.o,$(SRCS))


all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) -o $@ $^ $(CXXFLAGS) $(LDLIBS) $(LDFLAGS) 

$(DIR_OBJ)/%.o: %.cc
	@mkdir -p $(DIR_OBJ)
	$(CXX) -c -o $@ $< $(CXXFLAGS) $(INCLUDES)

clean:
	@$(RM) $(OBJS)
	@$(RM) *~ core

distclean: clean
	@$(RM) $(TARGET)

.PHONY: all clean distclean
	