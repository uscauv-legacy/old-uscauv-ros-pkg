#!/bin/env python
#==========================================================================
# (c) 2007  Total Phase, Inc.
#--------------------------------------------------------------------------
# Project : Cheetah Examples
# File    : blast.py
#--------------------------------------------------------------------------
# Blast SPI data using the Cheetah host adapter
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
# Uncomment to show the data returned by the shifting device
SHOW_DATA = 1

# Comment to show the data returned by the shifting device
#SHOW_DATA = 0

# Add a delay between bytes by changing this constant (in nanoseconds)
BYTE_DELAY = 0


#=========================================================================
# UTILITY FUNCTIONS
#=========================================================================
def _timeMillis():
    return long(time() * 1000L)


#=========================================================================
# FUNCTIONS
#=========================================================================
def _blast(handle, length):
    start = _timeMillis()

    # Queue the read sequence
    ch_spi_queue_clear(handle)
    ch_spi_queue_oe(handle, 1)
    ch_spi_queue_ss(handle, 0)
    ch_spi_queue_ss(handle, 0x1)

    for j in range(length):
        ch_spi_queue_byte(handle, 1, j & 0xff);
        delay = ch_spi_queue_delay_ns(handle, BYTE_DELAY)
        
    print "Queued delay of %d ns between bytes." % delay
    sys.stdout.flush()
    
    ch_spi_queue_ss(handle, 0)
    ch_spi_queue_oe(handle, 0)

    elapsed = (_timeMillis() - start) / 1000.0
    print "Took %.2lf seconds to queue the batch." % elapsed
    sys.stdout.flush()
    
    # Perform the shift
    batch    = ch_spi_batch_length(handle);

    start = _timeMillis()
    (count, data_in) = ch_spi_batch_shift(handle, batch)

    elapsed = (_timeMillis() - start) / 1000.0
    print "Took %.2lf seconds to shift the batch." % elapsed
    sys.stdout.flush()
    
    if (count != batch):
        print "Expected %d bytes but only received %d bytes" & (batch, count)

    if(SHOW_DATA):
        # Output the data to the screen
        sys.stdout.write( "\nData:")
        for i in range(length):
            if ((i & 0x07) == 0):
                print "\n%04x: " % i,
            print "%02x/%02x" % ( (i & 0xff), data_in[i]),
        print ""
        sys.stdout.flush()


#==========================================================================
# USAGE INFORMATION
#==========================================================================
def print_usage ():
    print \
"""
usage: blast PORT BITRATE MODE BITORDER LENGTH

MODE possibilities are:
  mode 0 : pol = 0, phase = 0
  mode 1 : pol = 0, phase = 1
  mode 2 : pol = 1, phase = 0
  mode 3 : pol = 1, phase = 1
"""


#==========================================================================
# MAIN PROGRAM
#==========================================================================
if (len(sys.argv) < 6):
    print_usage()
    sys.exit(1)

port     = int(sys.argv[1])
bitrate  = int(sys.argv[2])
mode     = int(sys.argv[3])
bitorder = int(sys.argv[4])
length   = int(sys.argv[5])
    
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

# Ensure that the SPI subsystem is configured.
ch_spi_configure(handle, (mode >> 1), mode & 1, (bitorder != 0), 0x0)
print "SPI configuration set to mode %d," % mode,
if (bitorder == 0):
    print "MSB",
else:
    print "LSB",
print "shift, SS[2:0] active low"
sys.stdout.flush()

# Power the target using the Cheetah adapter's power supply.
ch_target_power(handle, CH_TARGET_POWER_ON)
ch_sleep_ms(100)

# Set the bitrate.
bitrate = ch_spi_bitrate(handle, bitrate)
print "Bitrate set to %d kHz" % bitrate
sys.stdout.flush()

_blast(handle, length)

# Close and exit.
ch_close(handle)
sys.exit(0)


