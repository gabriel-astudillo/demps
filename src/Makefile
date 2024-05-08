CC=gcc
CXX=g++
RM=rm -f

TARGET=../sim/demps

RESTCLIENT_DIR=/usr/local/restclient-cpp
RESTCLIENT_INCLUDE=$(RESTCLIENT_DIR)/include
RESTCLIENT_LIB=$(RESTCLIENT_DIR)/lib
RESTCLIENT_LDFLAGS=$(RESTCLIENT_LIB) -Wl,-rpath -Wl,$(RESTCLIENT_LIB)

INCLUDES=-I/usr/local/include -I/usr/local/include/osrm -I./include -I. -I/usr/include -I$(RESTCLIENT_INCLUDE)
LDFLAGS=-L/usr/local/lib -L/usr/lib -L$(RESTCLIENT_LDFLAGS) -g
LDLIBS=-lGeographic -losrm -lboost_system -lboost_filesystem -lboost_iostreams -lboost_thread -lrt -lpthread -fopenmp -lgmp 
LDLIBS+=-lrestclient-cpp -luuid
LDLIBS+=-lcurl

CXXFLAGS=-std=c++17 -Wall -O3 -fopenmp -g

DIR_OBJ=objs

SRCS=$(wildcard *.cc)

OBJS=$(patsubst %.cc,$(DIR_OBJ)/%.o,$(SRCS))


all: $(TARGET)
	@echo Made [ $? ] OK :\)
	
$(TARGET): $(OBJS)
	@echo Linking [$@]
	@eval $(CXX) -o $@ $^ $(CXXFLAGS) $(LDLIBS) $(LDFLAGS) 

$(DIR_OBJ)/%.o: %.cc
	@echo Compiling [$@]
	@mkdir -p $(DIR_OBJ)
	@eval $(CXX) -c -o $@ $< $(CXXFLAGS) $(INCLUDES)

install:
	@echo Install demps to $(DIR_INSTALL)
	@sudo $(CP) $(TARGET) $(DIR_INSTALL)/$(TARGET)

clean:
	@$(RM) -r $(DIR_OBJ)
	@$(RM) *~ core

distclean: clean
	@$(RM) $(TARGET)
	@$(RM) -r $(DIR_OBJ)

.PHONY: all clean distclean
	
