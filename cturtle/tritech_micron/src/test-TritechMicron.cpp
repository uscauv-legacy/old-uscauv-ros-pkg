#include "../include/TritechMicron.h"
#include <iostream>
#include <vector>

int main()
{
  TritechMicron micron;

  std::vector<uint8_t> msg = {
    0x40,
    0x30,
    0x30,
    0x31,
    0x33,
    0x13,
    0x00,
    0x02,
    0xFF,
    0x0E,
    0x01,
    0x80,
    0x02,
    0x31,
    0x11,
    0x0D,
    0x8C,
    0x83,
    0xA8,
    0x00,
    0x00,
    0x3C,
    0x88,
    0x02,
    0x0A
  };

  for(unsigned char byte : msg)
  {
    micron.processByte(byte);

    printf("Processed Byte: 0x%x - sz=%lu State=%d bl=%d tx=%d rx=%d ct=%d tp=%d\n",
        byte, micron.itsRawMsg.size(), micron.itsState, micron.itsMsg.binLength, 
        micron.itsMsg.txNode, micron.itsMsg.rxNode, micron.itsMsg.count, micron.itsMsg.type);
  }


  return 0;
}
