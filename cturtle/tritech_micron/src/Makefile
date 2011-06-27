main : test-TritechMicron.o TritechMicron.o Serial.o
	g++ -std=c++0x test-TritechMicron.o TritechMicron.o Serial.o -o test-TritechMicron -g -pthread

test-TritechMicron.o : test-TritechMicron.cpp
	g++ -std=c++0x test-TritechMicron.cpp -c -o test-TritechMicron.o -g -pthread 

TritechMicron.o : TritechMicron.cpp ../include/TritechMicron.h ../include/Constants.h ../include/MessageTypes.h
	g++ -std=c++0x TritechMicron.cpp -c -o TritechMicron.o -g -pthread

Serial.o : Serial.cpp Serial.h
	g++ -std=c++0x Serial.cpp -c -o Serial.o -g -pthread

clean:
	rm *.o


