#include "../include/TritechMicron.h"
#include <iostream>
#include <vector>

int main()
{
  TritechMicron micron;

  std::vector<uint8_t> msg = {0x40, 0x30, 0x30, 0x30, 0x38, 0x08, 0x00, 0xFF, 0x02, 0x03, 0x17,0x80,0x02, 0x0A, 0x00, 0x00};

  for(unsigned char byte : msg)
  {
    micron.processByte(byte);

    printf("Processed Byte: 0x%x - sz=%lu State=%d bl=%d tx=%d rx=%d ct=%d tp=%d\n",
        byte, micron.itsRawMsg.size(), micron.itsState, micron.itsMsg.binLength, 
        micron.itsMsg.txNode, micron.itsMsg.rxNode, micron.itsMsg.count, micron.itsMsg.type);
  }


  return 0;
}
