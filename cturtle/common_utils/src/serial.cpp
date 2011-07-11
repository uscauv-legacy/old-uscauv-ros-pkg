#include <common_utils/serial.h>
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
void SerialPort::disconnect()
{
  if(itsFileDescriptor != -1) close(itsFileDescriptor);
}

// ######################################################################
bool SerialPort::connect(std::string dev, speed_t baudRate)
{
  if(itsFileDescriptor != -1) close(itsFileDescriptor);
  itsFileDescriptor = -1;

  struct termios toptions;

  itsFileDescriptor = open(dev.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (itsFileDescriptor == -1)  
  {
    perror("init_serialport: Unable to open port ");
    return false;
  }

  if (tcgetattr(itsFileDescriptor, &toptions) < 0) 
  {
    if(itsFileDescriptor != -1) close(itsFileDescriptor);
    itsFileDescriptor = -1;
    perror("init_serialport: Couldn't get term attributes");
    return false;
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
    if(itsFileDescriptor != -1) close(itsFileDescriptor);
    itsFileDescriptor = -1;
    perror("init_serialport: Couldn't set term attributes");
    return false;
  }

	// flush the port
	tcflush(itsFileDescriptor, TCIOFLUSH );

  return true;
}

// ######################################################################
bool SerialPort::connected()
{ return itsFileDescriptor != -1; }

// ######################################################################
int SerialPort::writeVector(std::vector<uint8_t> const& bytes)
{
  if(itsFileDescriptor == -1)
  { std::cerr << "Cannot write to closed serial port" << std::endl; return -1; }

  int bytesWritten = ::write(itsFileDescriptor, &bytes[0], bytes.size());
  if(bytesWritten != bytes.size())
    std::cerr << "Error writing to serial port: " << strerror(errno) << std::endl;
  return bytesWritten;
}

// ######################################################################
std::vector<uint8_t> SerialPort::read(size_t bytes)
{
  if(itsFileDescriptor == -1)
  { std::cerr << "Cannot read from closed serial port" << std::endl; return std::vector<uint8_t>(); }

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

