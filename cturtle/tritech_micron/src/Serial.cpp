#include "../include/Serial.h"
#include <iostream>

// ######################################################################
SerialPort::SerialPort()
{ }

// ######################################################################
SerialPort::~SerialPort()
{
  ::close(itsFileDescriptor);
}

// ######################################################################
bool SerialPort::connect(std::string dev, speed_t baudRate)
{
  struct termios toptions;

  itsFileDescriptor = open(dev.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (itsFileDescriptor == -1)  
  {
    perror("init_serialport: Unable to open port ");
    return -1;
  }

  if (tcgetattr(itsFileDescriptor, &toptions) < 0) 
  {
    perror("init_serialport: Couldn't get term attributes");
    return -1;
  }

  cfsetispeed(&toptions, baudRate);
  cfsetospeed(&toptions, baudRate);

  // 8N1
  toptions.c_cflag &= ~PARENB;
  toptions.c_cflag &= ~CSTOPB;
  toptions.c_cflag &= ~CSIZE;
  toptions.c_cflag |= CS8;
  // no flow control
  toptions.c_cflag &= ~CRTSCTS;

  toptions.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
  toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl

  toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
  toptions.c_oflag &= ~OPOST; // make raw

  // see: http://unixwiz.net/techtips/termios-vmin-vtime.html
  toptions.c_cc[VMIN]  = 0;
  toptions.c_cc[VTIME] = 20;

  if(tcsetattr(itsFileDescriptor, TCSANOW, &toptions) < 0) 
  {
    perror("init_serialport: Couldn't set term attributes");
    return -1;
  }
  return true;
}

// ######################################################################
int SerialPort::writeVector(std::vector<uint8_t> const& bytes)
{
  int bytesWritten = ::write(itsFileDescriptor, &bytes[0], bytes.size());
  if(bytesWritten != bytes.size())
    std::cerr << "Error writing to serial port: " << strerror(errno) << std::endl;
  return bytesWritten;
}



// ######################################################################
std::vector<uint8_t> SerialPort::read(size_t bytes)
{
  std::vector<uint8_t> ret(bytes);
  int bytesRead = ::read(itsFileDescriptor, &ret[0], bytes);

  if(bytesRead < 0) 
  {
    std::cerr << "Error reading from serial port: " << strerror(errno) << std::endl;
    return std::vector<uint8_t>();
  }
  ret.resize(bytesRead);
  return ret;
}
