#!/bin/env python
#==========================================================================
# (c) 2007  Total Phase, Inc.
#--------------------------------------------------------------------------
# Project : Cheetah Examples
# File    : eeprom.py
#--------------------------------------------------------------------------
# Perform simple read and write operations to an SPI EEPROM device
#--------------------------------------------------------------------------
# Redistribution and use of this file in source and binary forms, with
# or without modification, are permitted.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#==========================================================================

#==========================================================================
# IMPORTS
#==========================================================================
from cheetah_py import *
from time import *


#=========================================================================
# CONSTANTS
#=========================================================================
PAGE_SIZE             = 32      # Should be a power of 2 for code to work
PAGE_WRITE_BATCH_SIZE = 32
PAGE_WRITE_DELAY_NS   = 5000000


#=========================================================================
# FUNCTIONS
#=========================================================================
def _writeMemory (handle, addr, length, zero):

    data_out = array('B', [0 for i in range(3 + PAGE_SIZE)])
    # Write to the SPI EEPROM
    # 
    # The AT25080A EEPROM has 32 byte pages.  Data can be written
    # in pages, to reduce the number of overall SPI transactions
    # executed through the Cheetah adapter.
    n = 0
    while (n < length):
        ch_spi_queue_clear(handle)
        ch_spi_queue_oe(handle, 1)

        # Send PAGE_WRITE_BATCH_SIZE number of pages to the Cheetah per 
        # batch shift
        for i in range(PAGE_WRITE_BATCH_SIZE):
            if (n >= length):
                break

            # Send write enable command
            ch_spi_queue_ss(handle, 0x1)
            ch_spi_queue_byte(handle, 1, 0x06)
            ch_spi_queue_ss(handle, 0)

            # Assemble the write command and address
            sys.stdout.write( "addr = 0x%04x; " % addr)
            data_out[0] = 0x02
            data_out[1] = (addr >> 8) & 0xff
            data_out[2] = (addr >> 0) & 0xff

            # Assemble the data
            j = 3
            if(not ((n < length) and (addr & (PAGE_SIZE - 1)))):
                if(zero):
                    data_out[j] = 0
                else:
                    data_out[j] = n & 0xff
                j += 1
                addr += 1
                n += 1
                
            while( (n < length) and (addr & (PAGE_SIZE - 1))):
                if(zero):
                    data_out[j] = 0
                else:
                    data_out[j] = n & 0xff
                j += 1
                addr += 1
                n += 1
            
            print "num bytes = %d" % (j-3)
            sys.stdout.flush()

            # Queue the write transaction
            ch_spi_queue_ss(handle, 0x1)
            ch_spi_queue_array(handle, data_out)
            ch_spi_queue_ss(handle, 0)

            # Put in a wait for the write cycle time
            ch_spi_queue_delay_ns(handle, PAGE_WRITE_DELAY_NS)

        # Shift the page writes
        # Don't need the results back from the shift
        print "Shifting data"
        sys.stdout.flush()
        batch = ch_spi_batch_length(handle)
        (count, data) = ch_spi_batch_shift(handle, 0)
        if (count != batch):
            print "Expected %d bytes but only received %d bytes" \
                  % (batch, count)
            sys.exit(1)
        
        print "Shift complete"
        sys.stdout.flush()

def _readMemory (handle, addr, length):
    data_in = array('B', [0 for i in range(3 + length)])
   
    ch_spi_queue_clear(handle)
    ch_spi_queue_oe(handle, 1)

    # Queue the read command, address, and data
    ch_spi_queue_ss(handle, 0x1)
    ch_spi_queue_byte(handle, 1, 0x03)
    ch_spi_queue_byte(handle, 1, (addr >> 8) & 0xff)
    ch_spi_queue_byte(handle, 1, (addr >> 0) & 0xff)
    ch_spi_queue_byte(handle, length, 0x00)
    ch_spi_queue_ss(handle, 0)

    (count, data_in) = ch_spi_batch_shift(handle, length + 3)

    if (count < 0):
        print "error: %s" % ch_status_string(count)
    elif (count != length+3):
        print "error: read %d bytes (expected %d)" % (count-3, length)
    
    # Dump the data to the screen
    print "\nData read from device:",
    for i in range(length):
        if ((i & 0x0f) == 0):
            print "\n%04x: " % int(int(addr) + i),
        print "%02x" % int(data_in[i+3] & 0xff),
        if (((i+1)&0x07) == 0):
            print "",
    
    print ""
    sys.stdout.flush()


#==========================================================================
# USAGE INFORMATION
#==========================================================================
def print_usage ():
    print \
"""
usage: eeprom PORT BITRATE MODE read  ADDR LENGTH
usage: eeprom PORT BITRATE MODE write ADDR LENGTH
usage: eeprom PORT BITRATE MODE zero  ADDR LENGTH

MODE possibilities are:
  mode 0 : pol = 0, phase = 0
  mode 1 : pol = 0, phase = 1
  mode 2 : pol = 1, phase = 0
  mode 3 : pol = 1, phase = 1
"""


#==========================================================================
# MAIN PROGRAM
#==========================================================================
if (len(sys.argv) < 7):
    print_usage()
    sys.exit(1)

port     = int(sys.argv[1])
bitrate  = int(sys.argv[2])
mode     = int(sys.argv[3])
command  = sys.argv[4]
addr     = long(sys.argv[5])
length   = int(sys.argv[6])
    
# Open the device
handle = ch_open(port)
if (handle <= 0):
    print "Unable to open Cheetah device on port %d" % port
    print "Error code = %d (%s)" % (handle, ch_status_string(handle))
    sys.exit(1)
    
print "Opened Cheetah device on port %d" % port

ch_host_ifce_speed_string = ""

if (ch_host_ifce_speed(handle)):
    ch_host_ifce_speed_string = "high speed"
else:
    ch_host_ifce_speed_string = "full speed"

print "Host interface is %s" % ch_host_ifce_speed_string

# Ensure that the SPI subsystem is configured
ch_spi_configure(handle, (mode >> 1), mode & 1, CH_SPI_BITORDER_MSB, 0x0)
print "SPI configuration set to mode %d, MSB shift, SS[2:0] active low" % mode
sys.stdout.flush()

# Power the target using the Cheetah adapter's power supply
ch_target_power(handle, CH_TARGET_POWER_ON)
ch_sleep_ms(100)

# Set the bitrate
bitrate = ch_spi_bitrate(handle, bitrate)
print "Bitrate set to %d kHz" % bitrate
sys.stdout.flush()

# Shift a dummy byte to clear the EEPROM state
ch_spi_queue_clear(handle)
ch_spi_queue_oe(handle, 1)
ch_spi_queue_ss(handle, 0x1)
ch_spi_queue_byte(handle, 1, 0x00)
ch_spi_queue_ss(handle, 0)
ch_spi_batch_shift(handle, 0)

# Perform the requested operation
if (command == "write"):
    _writeMemory(handle, addr, length, 0)
    print "Wrote to EEPROM"
    
elif (command == "read"):
    _readMemory(handle, addr, length)
    
elif (command == "zero"):
    _writeMemory(handle, addr, length, 1)
    print "Zeroed EEPROM"
    
else: 
    print "unknown command: %s" % command

# Close and exit
ch_close(handle)
sys.exit(0)


