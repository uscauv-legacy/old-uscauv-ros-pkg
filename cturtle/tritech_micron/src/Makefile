main : test-TritechMicron.o TritechMicron.o
	g++ -std=c++0x test-TritechMicron.o TritechMicron.o -o test-TritechMicron

test-TritechMicron.o : test-TritechMicron.cpp
	g++ -std=c++0x test-TritechMicron.cpp -c -o test-TritechMicron.o

TritechMicron.o : TritechMicron.cpp ../include/TritechMicron.h ../include/Constants.h ../include/MessageTypes.h
	g++ -std=c++0x TritechMicron.cpp -c -o TritechMicron.o

clean:
	rm *.o


