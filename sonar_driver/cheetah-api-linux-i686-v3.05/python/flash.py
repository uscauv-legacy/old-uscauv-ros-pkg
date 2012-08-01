#!/bin/env python
#==========================================================================
# (c) 2007  Total Phase, Inc.
#--------------------------------------------------------------------------
# Project : Cheetah Examples
# File    : flash.py
#--------------------------------------------------------------------------
# Read from, erase, write to, and verify data contents of the High-Speed
# SPI Flash board
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
INTEGRITY_CHECK_NUM_BYTES  =  0x1000000     # When to stop data verify
INTEGRITY_LOOP_SIZE        =  (0x400000/4)  # Flash size divided by 4

# If page size is larger than 1K, _write function will have to be modified
# to take into account the possibility of starting the write in the middle 
# of a page.
PAGE_SIZE                  =  256
PAGE_WRITE_BATCH_SIZE      =  16
PAGE_PROGRAM_CYCLE_TIME_NS =  1400000

COMMAND_READ   = 0
COMMAND_WRITE  = 1
COMMAND_ERASE  = 2
COMMAND_VERIFY = 3


#=========================================================================
# FUNCTIONS
#=========================================================================
def _timeMicroseconds():
    return long(time() * 1000000L)

def _connect (port, bitrate, mode):
    # Open the device.
    handle = ch_open(port)
    if (handle <= 0):
        print "Unable to open Cheetah device on port %d" % port
        print "Error code = %d" % handle
        return (1, handle)
    print "Opened Cheetah device on port %d" % port

    ch_host_ifce_speed_string = ""
    
    if (ch_host_ifce_speed(handle)):
        ch_host_ifce_speed_string = "high speed"
    else:
        ch_host_ifce_speed_string = "full speed"
        
    print "Host interface is %s" % ch_host_ifce_speed_string

    # Ensure that the SPI subsystem is configured.
    ch_spi_configure(handle, (mode >> 1), mode & 1, CH_SPI_BITORDER_MSB, 0x0)
    print "SPI configuration set to mode %d, MSB shift, SS[2:0] active low" \
          % mode
    sys.stdout.flush()
    
    # Power the flash using the Cheetah adapter's power supply.
    ch_target_power(handle, CH_TARGET_POWER_ON)
    ch_sleep_ms(100)
    
    # Set the bitrate.
    bitrate = ch_spi_bitrate(handle, bitrate)
    if (bitrate < 0):
        print "Could not set bitrate."
        print "Error code = %d" % bitrate
        return (1, handle)
    
    print "Bitrate set to %d kHz" %  bitrate
    sys.stdout.flush()
    return (0, handle)

def _read (handle, addr, length):
    # Convert address and length from KB to bytes
    addr   *= 1024
    length *= 1024
    
    # Create the buffer to receive the flash values into.
    data_in = array('B', [0 for i in range(length)])
    
    ch_spi_queue_clear(handle)
    ch_spi_queue_oe(handle, 1)
    
    # Set slave select to deasserted state, in case it was left
    # low by a previously interrupted transaction (ctrl-c).  This
    # will reset the state machine inside the flash.
    ch_spi_queue_ss(handle, 0)
    ch_spi_queue_ss(handle, 1)
    
    # Queue fast read command code.
    ch_spi_queue_byte(handle, 1, 0x0b)
    
    # Queue 3 bytes of address.
    ch_spi_queue_byte(handle, 1, (addr >> 16) & 0xff)
    ch_spi_queue_byte(handle, 1, (addr >>  8) & 0xff)
    ch_spi_queue_byte(handle, 1, (addr >>  0) & 0xff)
    
    # Queue dummy byte.
    ch_spi_queue_byte(handle, 1, 0)
    
    # Shift the queued fast read command.
    (count, data_in) = ch_spi_batch_shift(handle, 0)
    if (count != 5):
        print "Expected 5 bytes from initial shift"
        return 1
    
    # Read the data.
    # Set the value to send while reading the flash.
    ch_spi_queue_clear(handle)
    ch_spi_queue_byte(handle, length, 0x00)
    (count, data_in) = ch_spi_batch_shift(handle, length)
    if (count != length):
        print "Expected %d bytes from read shift (got %d)" & (length, count)
        return 1
    
    # Dump the data to the screen
    print "\nData read from device:",
    for i in range(length):
        if ((i & 0x0f) == 0):
            print "\n%04x: " % int(int(addr)+i),
        print "%02x" % (data_in[i] & 0xff),
        if (((i + 1) & 0x07) == 0):
            print "",
            
    print ""
    sys.stdout.flush()
    
    # Clear the state of the bus.
    ch_spi_queue_clear(handle)
    ch_spi_queue_ss(handle, 0)
    ch_spi_queue_oe(handle, 0)
    ch_spi_batch_shift(handle, 0)
    
    return 0

def _write (handle, addr, length):
    # Buffer for outgoing data.
    data_page = array('B', [0 for i in range(4 + PAGE_SIZE)]) 

    # Reset the state of the bus.
    ch_spi_queue_clear(handle)
    ch_spi_queue_ss(handle, 0)
    ch_spi_queue_oe(handle, 1)
    ch_spi_batch_shift(handle, 0)
    
    # Convert address and length from KB to bytes
    addr   *= 1024
    length *= 1024

    # Set the starting counter based on the address.
    val = addr/4L

    while (length):
        # Start the write sequence.
        ch_spi_queue_clear(handle)
        ch_spi_queue_oe(handle, 1)

        # Send PAGE_WRITE_BATCH_SIZE number of pages to the Cheetah per 
        # batch shift.
        for i in range(PAGE_WRITE_BATCH_SIZE):
            # Check if we've reached the end.
            if (not length):
                break
            
            # Queue the write enable instruction for the flash.
            ch_spi_queue_ss(handle, 0x1)
            ch_spi_queue_byte(handle, 1, 0x06)
            ch_spi_queue_ss(handle, 0)

            # Queue the write instruction for the flash.
            ch_spi_queue_ss(handle, 0x1)
            data_page[0] = 0x02
            data_page[1] = (addr >> 16) & 0xff
            data_page[2] = (addr >>  8) & 0xff
            data_page[3] = (addr >>  0) & 0xff

            print "addr = 0x%06x num bytes = %d" % (addr, PAGE_SIZE)
            sys.stdout.flush()

            # Set the data to be written to the flash to incrementing
            # 32-bit values starting from 0.
            j = 0
            while (j < PAGE_SIZE):
                data_page[4+j+0] = 0
                data_page[4+j+1] = (val >> 16) & 0xff
                data_page[4+j+2] = (val >>  8) & 0xff
                data_page[4+j+3] = (val >>  0) & 0xff
                j += 4
                val += 1
            
            ch_spi_queue_array(handle,  data_page)
            ch_spi_queue_ss(handle, 0)

            # Give the flash time to commit the written values.
            # Using ch_spi_queue_delay_ns is much more accurate than
            # using ch_sleep_ms.
            ch_spi_queue_delay_ns(handle, PAGE_PROGRAM_CYCLE_TIME_NS)

            addr += PAGE_SIZE
            length -= PAGE_SIZE
        
        # Shift out the write command.  (Don't need the data back.)
        print "Shifting data"
        sys.stdout.flush()
        batch = ch_spi_batch_length(handle)
        (count, temp)  = ch_spi_batch_shift(handle, 0)
        if (count != batch):
            print "Expected %d bytes but only received %d bytes" \
                  % (batch, count)
            return 1
        
        print "Shift complete"
        sys.stdout.flush()

    # Reset the state of the bus.
    ch_spi_queue_clear(handle)
    ch_spi_queue_oe(handle, 0)
    ch_spi_batch_shift(handle, 0)

    return 0

def _erase (handle, sector, num):
    # Reset the state of the bus.
    ch_spi_queue_clear(handle)
    ch_spi_queue_ss(handle, 0)
    ch_spi_queue_oe(handle, 0)
    ch_spi_batch_shift(handle, 0)

    eraseAll = 0
    if (sector == 0 and num == 64):
        eraseAll = 1

    str = ""
    if (eraseAll):  str = "Bulk"
    else:           str = "Block"

    while (num):
        # Make sure the sector is a valid one.
        if (sector < 0 or sector > 63):
            break

        addr = sector << 16
        if (not eraseAll):
            print "Erasing sector %02d (bytes 0x%06x to 0x%06x)..." \
                  % (sector, addr, (addr | 0xffff))
            sys.stdout.flush()
        else:
            print "Erasing entire device..."
            sys.stdout.flush()

        # Start the erase sequence.
        ch_spi_queue_clear(handle)
        ch_spi_queue_oe(handle, 1)
    
        # Queue the write enable instruction for the flash.
        ch_spi_queue_ss(handle, 0x1)
        ch_spi_queue_byte(handle, 1, 0x06)
        ch_spi_queue_ss(handle, 0)

        ch_spi_queue_ss(handle, 0x1)

        if (not eraseAll):
            # Queue the sector erase command.
            ch_spi_queue_byte(handle, 1, 0xd8)
            ch_spi_queue_byte(handle, 1, (addr >> 16) & 0xff)
            ch_spi_queue_byte(handle, 1, (addr >>  8) & 0xff)
            ch_spi_queue_byte(handle, 1, (addr >>  0) & 0xff)
        
        else:
            # Queue the bulk erase command.
            ch_spi_queue_byte(handle, 1, 0xc7)

        ch_spi_queue_ss(handle, 0)
        # Shift the queued commands.  (Don't need the data back.)
        batch = ch_spi_batch_length(handle)
        (count, temp) = ch_spi_batch_shift(handle, 0)
        if (count != batch):
            print "Expected %d bytes but only received %d bytes\n" \
                  % (batch, count)
            return 1
        
        start = _timeMicroseconds()
        ch_spi_queue_clear(handle)
        ch_spi_queue_ss(handle, 0x1)
        ch_spi_queue_byte(handle, 1, 0x05)
        ch_spi_queue_byte(handle, 1, 0x00)
        ch_spi_queue_ss(handle, 0)
        while (1):
            ch_sleep_ms(10)
            status_in = array('B', [0, 0])
            (count, status_in) = ch_spi_batch_shift(handle, 2)
            if (not (status_in[1] & 0x01)):
                break
        
        end = _timeMicroseconds()
        print "%s erase took %.03f seconds" % (str, (end-start)/1000000.0)
        sys.stdout.flush()
        
        if (not eraseAll):
            sector += 1
            num -= 1

        else:
            sector += 64
            num     = 0
        
    # Reset the state of the bus.
    ch_spi_queue_clear(handle)
    ch_spi_queue_oe(handle, 0)
    ch_spi_batch_shift(handle, 0)
    
    return 0

def _printProgress (percent, elapsedTime, blockNum, blocksize): 
    progressbar = " " * 32
    tenths = int(percent/5)
    progressbar = "["
    if(tenths != 0):
        progressbar += tenths * " "
    progressbar += "#"
    if(20-tenths != 0):
        progressbar += (20-tenths) * " "
    progressbar += "]"
    print "\r%s %3d%% %7d KB %.2lf seconds" % \
          (progressbar, percent, blocksize / 1024 * (blockNum+1), elapsedTime),
    sys.stdout.flush()
    
def _verify (handle, length, blocksize): 
    if (length == 0 or blocksize == 0):
        return 2

    iter = int((length-1)/blocksize + 1)

    # Create the buffer to receive the flash values into.
    data_in = array('B', [0 for i in range(blocksize)])

    ch_spi_queue_clear(handle)
    ch_spi_queue_oe(handle, 1)
    
    # Set slave select to deasserted state, in case it was left
    # low by a previously interrupted transaction (ctrl-c).  This
    # will reset the state machine inside the flash.
    ch_spi_queue_ss(handle, 0)
    ch_spi_queue_ss(handle, 1)

    # Queue fast read command code.
    ch_spi_queue_byte(handle, 1, 0x0b)

    # Queue 3 bytes of address.
    ch_spi_queue_byte(handle, 1, 0)
    ch_spi_queue_byte(handle, 1, 0)
    ch_spi_queue_byte(handle, 1, 0)

    # Queue dummy byte.
    ch_spi_queue_byte(handle, 1, 0)

    # Shift the queued fast read command.
    (count, data_in) = ch_spi_batch_shift(handle, 5)
    if (count != 5):
        print "Expected 5 bytes from initial shift"
        return 1

    # Set the value to send while reading the flash.
    ch_spi_queue_clear(handle)
    ch_spi_queue_byte(handle, blocksize, 0x00)

    start = _timeMicroseconds()

    # Variables for the integrity check.
    integrity_counter = 0L
    integrity_errors = 0
    integrity_nb = 0
    isSucceed = 1

    _printProgress(0, 0, 0, blocksize)

    # Read one block at a time.
    for i in range(iter):
        # Read the next block of data.
        (count, data_in) = ch_spi_batch_shift(handle, blocksize)
        if (count != blocksize):
            print "Expected %d bytes from block shift (got %d)" \
                  % (blocksize, count)
            isSucceed = 0
            break
        
        # Check if the data in the flash matches a predefined
        # sequence.  Namely, there should be a running 32 bit
        # counter in the data.
        j = 0
        while (integrity_nb < INTEGRITY_CHECK_NUM_BYTES and j < count):
            val = long((data_in[j+0] << 24) \
                     | (data_in[j+1] << 16) \
                     | (data_in[j+2] <<  8) \
                     | (data_in[j+3] <<  0))

            if (val != integrity_counter % INTEGRITY_LOOP_SIZE): 
                integrity_errors += 1 
            
            integrity_counter += 1
            integrity_nb += 4
            j += 4
        
        # Print out the progress.
        currTime = _timeMicroseconds()

        _printProgress((i+1) * 100 / iter, \
                       (currTime - start) / 1000000.0, i, blocksize)
        
    # Clear the state of the flash.
    ch_spi_queue_clear(handle)
    ch_spi_queue_ss(handle, 0)
    ch_spi_queue_oe(handle, 0)
    ch_spi_batch_shift(handle, 0)
    
    print "\nThere were %d data errors." % integrity_errors
    sys.stdout.flush()
    
    if(integrity_errors):
        return 2
    else:
        return 0


#==========================================================================
# USAGE INFORMATION
#==========================================================================
def print_usage ():
    print \
"""
FLASH - utility for exercising the High-Speed SPI Flash board
usage: flash PORT BITRATE MODE read   ADDR_KB   LEN_KB
usage: flash PORT BITRATE MODE write  ADDR_KB   LEN_KB
usage: flash PORT BITRATE MODE erase  SECTOR    NUM
usage: flash PORT BITRATE MODE verify LENGTH_KB BLOCK_SIZE_KB

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
    
commandID = -1
if(command == "read"):
    commandID = COMMAND_READ

elif (command == "write"):
    commandID = COMMAND_WRITE

elif (command == "erase"):
    commandID = COMMAND_ERASE

elif (command == "verify"):
    commandID = COMMAND_VERIFY
    
else:
    print "Unknown option: %s" % command
    print "Valid options are: read, write, erase, and verify\n"
    print_usage();
    sys.exit(1)

# Connect to and configure the Cheetah.
(ret_code, handle) = _connect(port, bitrate, mode)
if(ret_code):
    sys.exit(1)

# Execute the appropriate command.
if(commandID ==  COMMAND_READ):
    ret = _read(handle, int(sys.argv[5]), int(sys.argv[6]))

elif (commandID == COMMAND_WRITE):
    ret = _write(handle, int(sys.argv[5]), int(sys.argv[6])) 

elif (commandID == COMMAND_ERASE):
    ret = _erase(handle, int(sys.argv[5]), int(sys.argv[6]))
    
elif (commandID == COMMAND_VERIFY):
    ret = _verify(handle, int(sys.argv[5]) * 1024, int(sys.argv[6]) * 1024)

# Close the device.
ch_close(handle);
sys.exit(0)
