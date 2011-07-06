CXXFLAGS=-I/usr/local/include -g -pthread -L/opt/local/lib -lboost_thread-mt
main : test-TritechMicron.o TritechMicron.o Serial.o ../include/MessageTypes.h
	g++ -std=c++0x test-TritechMicron.o TritechMicron.o Serial.o -o test-TritechMicron ${CXXFLAGS}

test-TritechMicron.o : test-TritechMicron.cpp ../include/MessageTypes.h
	g++ -std=c++0x test-TritechMicron.cpp -c -o test-TritechMicron.o ${CXXFLAGS}

TritechMicron.o : TritechMicron.cpp ../include/TritechMicron.h ../include/Constants.h ../include/MessageTypes.h
	g++ -std=c++0x TritechMicron.cpp -c -o TritechMicron.o ${CXXFLAGS}

Serial.o : Serial.cpp ../include/Serial.h
	g++ -std=c++0x Serial.cpp -c -o Serial.o ${CXXFLAGS}

clean:
	rm *.o test-TritechMicron


