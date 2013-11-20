/*!@file Devices/Serial.C class for interfacing with a serial port */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2003   //
// by the University of Southern California (USC) and the iLab at USC.  //
// See http://iLab.usc.edu for information about this project.          //
// //////////////////////////////////////////////////////////////////// //
// Major portions of the iLab Neuromorphic Vision Toolkit are protected //
// under the U.S. patent ``Computation of Intrinsic Perceptual Saliency //
// in Visual Environments, and Applications'' by Christof Koch and      //
// Laurent Itti, California Institute of Technology, 2001 (patent       //
// pending; application number 09/912,225 filed July 23, 2001; see      //
// http://pair.uspto.gov/cgi-bin/final/home.pl for current status).     //
// //////////////////////////////////////////////////////////////////// //
// This file is part of the iLab Neuromorphic Vision C++ Toolkit.       //
//                                                                      //
// The iLab Neuromorphic Vision C++ Toolkit is free software; you can   //
// redistribute it and/or modify it under the terms of the GNU General  //
// Public License as published by the Free Software Foundation; either  //
// version 2 of the License, or (at your option) any later version.     //
//                                                                      //
// The iLab Neuromorphic Vision C++ Toolkit is distributed in the hope  //
// that it will be useful, but WITHOUT ANY WARRANTY; without even the   //
// implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      //
// PURPOSE.  See the GNU General Public License for more details.       //
//                                                                      //
// You should have received a copy of the GNU General Public License    //
// along with the iLab Neuromorphic Vision C++ Toolkit; if not, write   //
// to the Free Software Foundation, Inc., 59 Temple Place, Suite 330,   //
// Boston, MA 02111-1307 USA.                                           //
// //////////////////////////////////////////////////////////////////// //
//
// Primary maintainer for this file: Nitin Dhavale <dhavale@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/Serial.C $
// $Id: Serial.C 12962 2010-03-06 02:13:53Z irock $
//

#include <uscauv_common/serial.h>

// ######################################################################

SerialPort::SerialPort()
{
  itsDeviceDescription = "";
  itsSearchPrefix = "ttyUSB";
  itsBaud = 9600;
  itsCharBits = 8;
  itsStopBits = 1;
  itsUseParity = false;
  itsParityOdd = false;
  itsFlowHard = false;
  itsFlowSoft = false;
  itsRdTout = 0;
  itsBlocking = true;
  itsSerialOk = false;
  dev = -1;
  itsDevName = "";
  serialErrno = serialErrSuccess;
}

bool SerialPort::connect(std::string dev, speed_t baudRate)
{
  configure( dev.c_str(), baudRate );
  return connect();
}

bool SerialPort::connected()
{
  return dev > 0;
}

void SerialPort::disconnect()
{
  closePort();
}

int SerialPort::writeVector(std::vector<uint8_t> const& bytes)
{
  if(dev == -1)
    { ROS_ERROR_STREAM("Cannot write to closed serial port" ); return -1; }

  int bytesWritten = ::write(dev, &bytes[0], bytes.size());
  if((size_t)bytesWritten != bytes.size())
    ROS_ERROR_STREAM( "Error writing to serial port: " << strerror(errno));
  return bytesWritten;
}

// ######################################################################
std::vector<uint8_t> SerialPort::read(size_t bytes)
{
  if(dev == -1)
    { ROS_ERROR_STREAM( "Cannot read from closed serial port" ); return std::vector<uint8_t>(); }

  std::vector<uint8_t> ret(bytes);
  int bytesRead = ::read(dev, &ret[0], bytes);

  if(bytesRead < 0)
    {
      ROS_ERROR_STREAM("Error reading from serial port: " << strerror(errno) );
      return std::vector<uint8_t>();
    }
  ret.resize(bytesRead);
  return ret;
}

// ######################################################################
bool SerialPort::connect()
{
  //Check to see if we have a hardcoded device name. If so, then let's just
  //go ahead and enable that port. If
  ROS_INFO(" Looking Device Name [%s]",itsDevName.c_str());
  if(itsDevName != "")
    {
      ROS_INFO( "Opening %s", itsDevName.c_str());
      enablePort(itsDevName);
    } else if (itsDevName == "search") {
    ROS_INFO("Searching for devices");
    itsCmdDevName = "";

    DIR *directory_p;
    struct dirent *entry_p;

    //Open the device directory to search for devices whose names match the search prefix
    directory_p = ::opendir ("/dev");
    if (directory_p == NULL)
      ROS_FATAL("Could Not Open /dev Directory!");
		
    //Iterate through the directory entries
    while ((entry_p = ::readdir (directory_p)))
      {
	std::string entryName(entry_p->d_name);
	if(entryName.find(itsSearchPrefix.c_str()) != std::string::npos)
	  {//If the directory entry name matches our search prefix, then let's try configuring a serial
	    //port on that device, sending it an identity request command (0x00), and comparing the result
	    //with our required device description

	    enablePort("/dev/" + entryName);
	    unsigned char cmd[1] = {0};

	    write(cmd,1);
	    std::vector<unsigned char> deviceStringVec = readFrame(cmd[0], 255);

	    std::string deviceString(deviceStringVec.begin(), deviceStringVec.end());
	    ROS_INFO("%s : %s", entryName.c_str(), deviceString.c_str());

	    if(deviceString == itsDeviceDescription)
	      {
		itsCmdDevName = "/dev/"+entryName;
		break;
	      }
	  }


      }
    (void) ::closedir (directory_p);
    if(itsCmdDevName == "")
      {
	ROS_FATAL("Could Not Find Serial Device Matching Descriptor (%s)", itsDeviceDescription.c_str());
      }
  } else {
    ROS_INFO("Opening from cmd line %s", itsCmdDevName.c_str());
    enablePort(itsCmdDevName);
  }
  return connected();
}


// ######################################################################
void SerialPort::enablePort(std::string DeviceName)
{
  closePort();
  serialErrno = serialErrSuccess;      // clear the error flag

  openPort(DeviceName);                // open the device
  perror();

  setSpeed(itsBaud);          // Set the baud rate
  perror();

  setFlowControl(itsFlowHard,
		 itsFlowSoft);           // Set flow control
  perror();

  setCharBits(itsCharBits);   // set no of bits in a char
  perror();

  setParity(itsUseParity,
	    itsParityOdd);          // Set even or odd parity
  perror();

  setBlocking(itsBlocking);   // blocking mode?
  perror();
}


#include <cstdio>
// ######################################################################
void SerialPort::closePort()
{
  if(dev >= 0) {
    //sendBreak();
    close(dev);
    dev = -1; }
}

// ######################################################################
void SerialPort::configureSearch(const std::string DeviceDescription, const int speed, const std::string SearchPrefix, const char *format,
				 const bool flowSoft, const bool flowHard,
				 const int tout)
{
  ROS_INFO("Search for %s in speed %d",DeviceDescription.c_str(),speed);
  itsDevName = "search";
  itsDeviceDescription = DeviceDescription;
  itsSearchPrefix = SearchPrefix;
  //enable serch
  //configure("search", speed, format, flowSoft, flowHard, tout);
  //Bypass search
  configure("", speed, format, flowSoft, flowHard, tout);
}

// ######################################################################
void SerialPort::configure(const char *dev, const int speed, const char *format,
			   const bool flowSoft, const bool flowHard,
			   const int tout)
{
  itsDevName = dev;


  itsBaud = speed;
  itsFlowSoft = flowSoft;
  itsFlowHard = flowHard;
  itsRdTout = tout;

  if (strlen(format) != 3) ROS_FATAL("Incorrect format string: %s", format);

  switch(format[0]) {
  case '5': itsCharBits = 5; break;
  case '6': itsCharBits = 6; break;
  case '7': itsCharBits = 7; break;
  case '8': itsCharBits = 8; break;
  default: ROS_FATAL("Invalid charbits: %c (should be 5..8)", format[0]);
  }

  switch(format[1]) {
  case 'N': itsUseParity = false; break;
  case 'E': itsUseParity = true; itsParityOdd = false; break;
  case 'O': itsUseParity = true; itsParityOdd = true; break;
  default: ROS_FATAL("Invalid parity: %c (should be N,E,O)", format[1]);
  }

  switch(format[2]) {
  case '1': itsStopBits = 1; break;
  case '2': itsStopBits = 2; break;
  default: ROS_FATAL("Invalid stopbits: %c (should be 1..2)", format[2]);
  }
}

// ######################################################################
serialError SerialPort::setSpeed(const int speed)
{
  struct termios options;
  unsigned int rate;

  switch(speed)
    {
    case 115200:
      rate = B115200;
      break;
    case 57600:
      rate = B57600;
      break;
    case 38400:
      rate = B38400;
      break;
    case 19200:
      rate = B19200;
      break;
    case 9600:
      rate = B9600;
      break;
    case 4800:
      rate = B4800;
      break;
    case 2400:
      rate = B2400;
      break;
    case 1200:
      rate = B1200;
      break;
    case 600:
      rate = B600;
      break;
    case 300:
      rate = B300;
      break;
    case 110:
      rate = B110;
      break;
    case 0:
      rate = B0;
      break;
    default:
      return serialError(serialErrSpeedInvalid);
    }

  // get current options
  if(tcgetattr(dev, &options)==-1){
    serialErrno = serialErrTcGetAttrFailed;
    return serialErrno;
  }

  // set the speed
  cfsetispeed(&options, rate);
  cfsetospeed(&options, rate);

  // change the terminals parameter instantly
  if( tcsetattr(dev, TCSANOW, &options) == -1){
    serialErrno = serialErrTcSetAttrFailed;
    return serialErrno;
  }

  if( tcgetattr(dev, &options) == -1) {
    serialErrno = serialErrTcGetAttrFailed;
    return serialErrno;
  }

  // update our ModelParam:
  itsBaud = speed;

  return serialErrSuccess;
}


// ######################################################################
serialError SerialPort::setFlowControl(const bool useHard, const bool useSoft)
{
  termios options;

  if( tcgetattr(dev, &options) == -1){
    serialErrno = serialErrTcGetAttrFailed;
    return serialErrno;
  }

  options.c_cflag &= ~CRTSCTS;
  options.c_iflag &= ~(IXON | IXANY | IXOFF);

  if (useSoft) options.c_iflag |= (IXON | IXANY | IXOFF);
  if (useHard) options.c_cflag |= CRTSCTS;

  if(tcsetattr(dev, TCSANOW, &options) == -1){
    serialErrno = serialErrTcGetAttrFailed;
    return serialErrno;
  }

  // update our ModelParams:
  itsFlowHard = useHard;
  itsFlowSoft = useSoft;

  return serialErrSuccess;
}

// ######################################################################
serialError SerialPort::setCharBits(const int bits)
{
  termios options;

  if(tcgetattr(dev, &options)==-1){
    serialErrno = serialErrTcGetAttrFailed;
    return serialErrno;
  }

  options.c_cflag &= ~CSIZE; // mask off the 'size' bits

  switch(bits)
    {
    case 5: options.c_cflag |= CS5; break;
    case 6: options.c_cflag |= CS6; break;
    case 7: options.c_cflag |= CS7; break;
    case 8: options.c_cflag |= CS8; break;
    default: return serialError(serialErrCharsizeInvalid);
    }

  if( tcsetattr(dev, TCSANOW, &options) == -1 ){
    serialErrno = serialErrTcSetAttrFailed;
    return serialErrno;
  }

  // update our ModelParam:
  itsCharBits = bits;

  return serialErrSuccess;
}

// ######################################################################
serialError SerialPort::setParity(const bool useParity, const bool oddParity)
{
  struct termios options;

  if(tcgetattr(dev, &options)==-1){
    serialErrno = serialErrTcGetAttrFailed;
    return serialErrno;
  }

  options.c_cflag &= ~(PARENB | PARODD);
  if (useParity)
    {
      if (oddParity)
	options.c_cflag |= (PARENB | PARODD);
      else
	options.c_cflag |= PARENB;
    }

  if(tcsetattr(dev, TCSANOW, &options) == -1){
    serialErrno = serialErrTcSetAttrFailed;
    return serialErrno;
  }

  // update our ModelParams:
  itsUseParity = useParity;
  itsParityOdd = oddParity;

  return serialErrSuccess;
}

// ######################################################################
serialError SerialPort::setStopBits(const int bits)
{
  struct termios options;

  if(tcgetattr(dev, &options)==-1){
    serialErrno = serialErrTcGetAttrFailed;
    return serialErrno;
  }

  options.c_cflag &= ~CSTOPB;
  if (bits == 2) options.c_cflag |= CSTOPB;
  else if (bits != 1) return serialError(serialErrStopbitsInvalid);

  if(tcsetattr(dev, TCSANOW, &options) == -1){
    serialErrno = serialErrTcSetAttrFailed;
    return serialErrno;
  }

  // update our ModelParam:
  itsStopBits = bits;

  return serialErrSuccess;
}

// ######################################################################
serialError SerialPort::setBlocking(const bool blocking)
{
  int flags = fcntl(dev, F_GETFL, 0);
  if (flags == -1) ROS_ERROR("Cannot get flags");
  if (blocking) flags &= (~O_NONBLOCK); else flags |= O_NONBLOCK;
  if (fcntl(dev, F_SETFL, flags) == -1) ROS_ERROR("Cannot set flags");

  itsBlocking = blocking;

  return serialErrSuccess;
}

// ######################################################################
void SerialPort::toggleDTR(const time_t ms)
{
  struct termios tty, old;
  if(tcgetattr(dev, &tty) == -1 || tcgetattr(dev, &old) == -1){
    serialErrno = serialErrTcGetAttrFailed;
  }

  cfsetospeed(&tty, B0);
  cfsetispeed(&tty, B0);

  if(tcsetattr(dev, TCSANOW, &tty) == -1){
    serialErrno = serialErrTcSetAttrFailed;
  }

  if(ms)
    usleep(ms*1000);

  if(tcsetattr(dev, TCSANOW, &old) == -1){
    serialErrno = serialErrTcSetAttrFailed;
  }
}

// ######################################################################
void SerialPort::sendBreak(void)
{
  // Send a Hangup to the port
  tcsendbreak(dev, 0);
}

// ######################################################################
serialError SerialPort::error(const serialError serialErrorNum)
{
  serialErrno = serialErrorNum;
  return serialErrorNum;
}

// ######################################################################
int SerialPort::openPort(std::string DeviceName)
{
  /*const pid_t fuser =
    rutz::unixcall::get_file_user_pid(DeviceName.c_str());

    if (fuser != 0 && DeviceName.compare("/dev/null") != 0)
    {
    printf("INFO: serial device %s is already in use by process pid=%d;"
    "\ttry using /dev/null or a different serial device instead",
    DeviceName.c_str(), int(fuser));
    }*/

  int flags= O_RDWR | O_NOCTTY;
  termios options;

  // don't set the flag if we are setting a timeout on
  // the descriptor
  if (itsRdTout < 0) flags |= O_NDELAY;

  ROS_INFO("Opening port %s", DeviceName.c_str());
  dev = ::open(DeviceName.c_str(), flags);
  if (dev == -1) { serialErrno = serialErrOpenFailed; return dev; }
  ROS_INFO("Done");

  // Save current state
  if( tcgetattr(dev, &savedState) == -1 ){
    serialErrno = serialErrTcGetAttrFailed;
    return -1;
  }

  // reset all the flags
  if( fcntl(dev, F_SETFL, 0) == -1 ){
    serialErrno = serialErrFcntlFailed;
    return -1;
  }

  if(tcgetattr(dev, &options) == -1 ){
    serialErrno = serialErrTcGetAttrFailed;
    return -1;
  }

  // get raw input from the port
  options.c_cflag |= ( CLOCAL     // ignore modem control lines
		       | CREAD ); // enable the receiver

  options.c_iflag &= ~(  IGNBRK    // ignore BREAK condition on input
			 | BRKINT  // If IGNBRK is not set, generate SIGINT
			 // on BREAK condition, else read BREAK as \0
			 | PARMRK
			 | ISTRIP  // strip off eighth bit
			 | INLCR   // donot translate NL to CR on input
			 | IGNCR   // ignore CR
			 | ICRNL   // translate CR to newline on input
			 | IXON    // disable XON/XOFF flow control on output
			 );

  // disable implementation-defined output processing
  options.c_oflag &= ~OPOST;
  options.c_lflag &= ~(ECHO  // dont echo i/p chars
		       | ECHONL // do not echo NL under any circumstance
		       | ICANON // disable cannonical mode
		       | ISIG   // do not signal for INTR, QUIT, SUSP etc
		       | IEXTEN // disable platform dependent i/p processing
		       );

  // set a timeout on the descriptor
  if(itsRdTout > 0) {
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = itsRdTout;
  }

  if( tcsetattr(dev, TCSANOW, &options) == -1){
    serialErrno = serialErrTcSetAttrFailed;
    return -1;
  }
  return dev;
}


// ######################################################################
int SerialPort::read(void* buffer, const int nbytes)
{
  int n = ::read(dev, buffer, nbytes);
  if(n==-1)
    serialErrno = serialErrReadFailed;

  if(n==0)
    serialErrno = serialErrReadTimedOut;

  return n;
}


// ######################################################################
char SerialPort::read(void)
{
  char ch;

  this->read(&ch, 1);

  return ch;
}

std::vector<unsigned char> SerialPort::readFrame(const unsigned char frameStart, const unsigned char frameEnd, const int frameLength, const double timeout)
{

  std::vector<unsigned char> data;
  unsigned char buffer[1024];

  //Timer timer;

  unsigned int startPos  =  0;
  unsigned int currPos   =  0;
  int frameSize          = -1;
  bool done              = false;
  int startFrameOffset   = 2;

  if(frameLength > 0)
    {
      frameSize = frameLength;
      startFrameOffset = 1;
    }

  //timer.reset();
  while(!done)
    {
      //If we have reached the timeout, then just quit return an empty vector
      //if(timeout > 0 && timer.getSecs() > timeout)
      //        return std::vector<unsigned char>();

      //Read data into the data buffer
      int bytesRead = read(buffer, 1024);

      if(bytesRead > 0)
	{
	  //If we read some bytes, then let's append them onto the data buffer
	  std::vector<unsigned char> newData(buffer, buffer+bytesRead*(sizeof(buffer[0])));
	  data.insert(data.end(), newData.begin(), newData.end());

	  while(currPos < data.size())
	    {
	      //If we have reached the timeout, then just quit return an empty vector
	      //if(timeout > 0 && timer.getSecs() > timeout)
	      //        return std::vector<unsigned char>();

	      if(data.at(startPos) != frameStart)
		{ //Get the start byte position settled
		  currPos++;
		  startPos = currPos;
		}
	      else
		{ //Here, we already have our start byte

		  if(frameLength == -1 && frameSize == -1 && (startPos+1 < data.size()) )
		    { //Find the frame size if one was not supplied, and we haven't yet found it
		      frameSize = data.at(startPos+1);
		      currPos++;
		    }
		  else if(frameSize > -1 && data.size() <= currPos+frameSize+1)
		    { //If we know our frame size, but our data buffer is not large enough,
		      //then just break and we will read more data at the top of the while loop
		      break;
		    }
		  else if(frameSize > -1 && data.size() > currPos+frameSize+1 && data.at(currPos+frameSize+1) != frameEnd)
		    { //If we know our frame size, and have enough data, and our frame stop
		      //byte is not there, then we need to move our startPos hypothesis one
		      //byte forward and start the whole thing over again
		      if(frameLength == -1)
			frameSize = -1;
		      startPos++;
		      currPos = startPos;
		    }
		  else if(frameSize > -1 && data.size() > currPos+frameSize+1 && data.at(currPos+frameSize+1) == frameEnd)
		    { //All constraints satisfied - we have our frame
		      return std::vector<unsigned char>(data.begin()+startPos+startFrameOffset, data.begin()+startPos+startFrameOffset+frameSize);
		    }
		}
	    }
	}
      else
	{ //If the serial read returned 0 bytes, then just wait for a bit before reading again
	  usleep(1000);

	  //If the timer has expired, then let's just return an empty vector
	  //if(timer.getSecs() > timeout && timeout > 0)
	  //        return std::vector<unsigned char>();
	}
    }
  return std::vector<unsigned char>();
}


// ######################################################################
int SerialPort::write(const void* buffer, const int nbytes)
{

  //char c[3] = { 0xff, 0x01, 0xff };
  //::write(dev, c, 3);

  int n = ::write(dev, buffer, nbytes);

  /*
  //printf("INFO: ----------------- WRITE --------------------");
  for(int i=0; i<n; ++i)
  {
  fprintf(stderr, "%02X ",
  (static_cast<const unsigned char*>(buffer)[i])&0xff);
  }
  fprintf(stderr,"");
  //printf("INFO: -------------------------------------------");
  */

  if(n==-1)
    {
      serialErrno = serialErrWriteFailed;
      ROS_ERROR("Error writing data to serial port");
    }


  return n;
}

// ######################################################################
int SerialPort::write(const unsigned char character)
{
  return write(&character, 1);
}


// ######################################################################
void SerialPort::flush(void)
{
  //Flush the input
  int status = tcflush(dev,TCIFLUSH);
  if(status != 0)
    ROS_ERROR("BAD SERIAL FLUSH");

}

// ######################################################################
void SerialPort::perror(void)
{
  switch(serialErrno)
    {
    case serialErrSuccess:
      ROS_ERROR("SerialPort::OK");
      return;
    case serialErrOpenNoTty:
      ROS_ERROR("SerialPort::Open() No TTY Failed");
      break;
    case serialErrOpenFailed:
      ROS_ERROR("SerialPort::Open() Failed");
      break;
    case serialErrSpeedInvalid:
      ROS_ERROR("SerialPort::setSpeed() Invalid Speed Param");
      break;
    case serialErrFlowInvalid:
      ROS_ERROR("SerialPort::setFlow() Invalid Flow Param");
      break;
    case serialErrParityInvalid:
      ROS_ERROR("SerialPort::setParity() Invalid parity Param");
      break;
    case serialErrCharsizeInvalid:
      ROS_ERROR("SerialPort::setCharSize() Invalid Char Size");
      break;
    case serialErrStopbitsInvalid:
      ROS_ERROR("SerialPort::setStopBits() Invalid Stop Bits");
      break;
    case serialErrOptionInvalid:
      ROS_ERROR("SerialPort::read() invalid option");
      break;
    case serialErrOutput:
      ROS_ERROR("SerialPort::read() err output");
      break;
    case serialErrReadFailed:
      ROS_ERROR("SerialPort::read() Failed");
      break;
    case serialErrReadTimedOut:
      ROS_ERROR("SerialPort::read() Timed Out");
      break;
    case serialErrWriteFailed:
      ROS_ERROR("SerialPort::Write() Failed");
      break;
    case serialErrFcntlFailed:
      ROS_ERROR("SerialPort::Fcntl() Failed");
      break;
    case serialErrTcGetAttrFailed:
      ROS_ERROR("SerialPort::tcgetattr() Failed");
      break;
    case serialErrTcSetAttrFailed:
      ROS_ERROR("SerialPort::tcsetattr() Failed");
      break;
    default:
      ROS_ERROR("Unknow error!");
    }
}

SerialPort::~SerialPort(void)
{
  closePort();
}
