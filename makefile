EXECUTABLE = fm_transmitter
VERSION = 0.9.4
FLAGS = -Wall -O3 -std=c++11
TRANSMITTER = -fno-strict-aliasing -I/opt/vc/include
ifeq ($(GPIO21), 1)
	TRANSMITTER += -DGPIO21
endif

all: main.o mailbox.o sample.o wave_reader.o transmitter.o
	g++ -L/opt/vc/lib -o $(EXECUTABLE) $^ -lm -lpthread -lbcm_host

mailbox.o: mailbox.c mailbox.h
	g++ $(FLAGS) -c $<

sample.o: sample.cpp sample.hpp
	g++ $(FLAGS) -c $<

wave_reader.o: wave_reader.cpp wave_reader.hpp
	g++ $(FLAGS) -c $<

transmitter.o: transmitter.cpp transmitter.hpp
	g++ $(FLAGS) $(TRANSMITTER) -c $<

main.o: main.cpp
	g++ $(FLAGS) -DVERSION=\"$(VERSION)\" -DEXECUTABLE=\"$(EXECUTABLE)\" -c $<

clean:
	rm *.o
