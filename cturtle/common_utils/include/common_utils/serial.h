#include <stdio.h> // standard input / output functions
#include <string.h> // string function definitions
#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h> // File control definitions
#include <errno.h> // Error number definitions
#include <termios.h> // POSIX terminal control definitionss
#include <time.h>   // time calls
#include <string>
#include <vector>
#include <boost/thread.hpp>

#ifndef SERIAL_H_
#define SERIAL_H_

class SerialPort
{

public:

	enum BlockingMode
	{
		Blocking, NonBlocking
	};

	SerialPort();
	~SerialPort();

  //! Connect to the serial port
	bool connect(std::string dev, speed_t baudRate = B115200);

	//! Is this serial port currently connected
	bool connected();

  //! Close down our opened file descriptor
  void disconnect();

	int writeVector( std::vector<uint8_t> const& bytes );
	std::vector<uint8_t> read( size_t bytes );

private:
	int itsFileDescriptor;
};

#endif /* SERIAL_H_ */
