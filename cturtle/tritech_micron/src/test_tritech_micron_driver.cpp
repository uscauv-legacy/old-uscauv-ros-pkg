#include <tritech_micron/tritech_micron_driver.h>
#include <iostream>
#include <vector>


void processScanline(float angle, float metersPerBin, std::vector<uint8_t> scanline)
{
  std::cout << "Got Scanline: " << angle << " " << scanline.size() << " bins @ " << metersPerBin << " meters/bin" << std::endl;
}

int main(int argc, char** argv)
{
  if(argc != 2)
  {
    std::cerr << "Usage: " << argv[0] << " serialdev" << std::endl;
    return -1;
  }

//  tritech::mtHeadCommandMsg msg;
//  for(uint8_t b : msg.construct())
//    std::cout << std::hex << int(b) << " ";
//  std::cout << std::endl;

  TritechMicronDriver micron(true);

  micron.registerScanLineCallback(std::bind(processScanline, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  if(!micron.connect(argv[1])) return -1;

  while(1) 
  { usleep(100000); }

/*
  std::vector<uint8_t> mtVersionDataMsgVec = {
    0x40, 0x30, 0x30, 0x31, 0x33, 0x13, 0x00, 0x02, 0xFF, 0x0E, 0x01, 0x80, 0x02,
    0x31, 0x11, 0x0D, 0x8C, 0x83, 0xA8, 0x00, 0x00, 0x3C, 0x88, 0x02, 0x0A };

  for(unsigned char byte : mtVersionDataMsgVec)
  {
    micron.processByte(byte);

    //printf("Processed Byte: 0x%x - sz=%lu State=%d bl=%d tx=%d rx=%d ct=%d tp=%d\n",
    //    byte, micron.itsRawMsg.size(), micron.itsState, micron.itsMsg.binLength, 
    //    micron.itsMsg.txNode, micron.itsMsg.rxNode, micron.itsMsg.count, micron.itsMsg.type);
  }


  std::vector<uint8_t> mtAliveMsgVec = {
    0x40, 0x30, 0x30, 0x31, 0x30, 0x10, 0x00, 0x02, 0xFF, 0x0B, 0x04, 0x80, 0x02,
    0x80, 0xAA, 0x10, 0x00, 0x00, 0x80, 0x0C, 0x5D, 0x0A };

  for(unsigned char byte : mtAliveMsgVec)
    micron.processByte(byte);

  std::vector<uint8_t> mtHeadDataMsgVec = {
    0x40, 0x30, 0x30, 0x35, 0x34, 0x54, 0x00, 0x02, 0xFF, 0x00, 0x02, 0x80, 0x02, 0x4C,
    0x00, 0x02, 0x10, 0x05, 0x85, 0xA3, 0x3C, 0x00, 0x66, 0x66, 0x66, 0x05, 0x6B, 0x7D,
    0x00, 0x32, 0x2C, 0x00, 0x00, 0x6B, 0x00, 0x40, 0x06, 0xC0, 0x12, 0x10, 0x80, 0x0A,
    0x2D, 0x00, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
    21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41,
    42, 43, 44, 45, 0x0A}; 

  for(unsigned char byte : mtHeadDataMsgVec)
    micron.processByte(byte);
    */

  return 0;
}

