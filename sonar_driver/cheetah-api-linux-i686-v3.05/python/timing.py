#!/bin/env python
#==========================================================================
# (c) 2007  Total Phase, Inc.
#--------------------------------------------------------------------------
# Project : Cheetah Examples
# File    : timing.py
#--------------------------------------------------------------------------
# Demonstrate user specified timing and delays
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
# FUNCTIONS
#=========================================================================
def _timing (handle):
    # Test the SS timing
    ch_spi_queue_clear(handle)
    print "Testing inter-SS delays..."
    sys.stdout.flush()
    ch_spi_queue_oe(handle, 1)
    ch_spi_queue_ss(handle, 0x1)

    cycles = ch_spi_queue_delay_cycles(handle, 51)
    print "  Queued delay of %d cycles within first SS assert/deassert." \
             % cycles
    sys.stdout.flush()
    ch_spi_queue_ss(handle, 0)

    ch_spi_queue_ss(handle, 0x1)

    ns = ch_spi_queue_delay_ns(handle, 1500000)
    print "  Queued delay of %d ns within second SS assert/deassert." % ns
    sys.stdout.flush()
    ch_spi_queue_ss(handle, 0)
    
    ch_spi_batch_shift(handle, 0)

    # Test data timing in read mode
    print "Testing inter-data (read) delays..."
    sys.stdout.flush()
    ch_spi_queue_clear(handle)
    ch_spi_queue_ss(handle, 0x1)

    ch_spi_queue_byte(handle, 1, 0xca)
    ns = ch_spi_queue_delay_ns(handle, 250000)
    print "  Queued delay of %d ns after first byte (0xca)." % ns
    sys.stdout.flush()

    ch_spi_queue_byte(handle, 2, 0xfe)
    cycles = ch_spi_queue_delay_cycles(handle, 995)
    print "  Queued delay of %d cycles after second byte (0xfe)." % cycles
    sys.stdout.flush()

    ch_spi_queue_byte(handle, 3, 0x00)
    cycles = ch_spi_queue_delay_cycles(handle, 20)
    print "  Queued delay of %d cycles after last byte (0x00)." % cycles
    sys.stdout.flush()

    ch_spi_queue_ss(handle, 0)

    ch_spi_batch_shift(handle, 6)

    # Test data timing with write mode
    print "Testing inter-data (write) delays..."
    sys.stdout.flush()
    ch_spi_queue_clear(handle)
    ch_spi_queue_ss(handle, 0x1)

    ch_spi_queue_byte(handle, 1, 0xba)
    ns = ch_spi_queue_delay_ns(handle, 80000)
    print "  Queued delay of %d ns after first byte (0xba)." % ns
    sys.stdout.flush()

    ch_spi_queue_byte(handle, 2, 0xbe)
    cycles = ch_spi_queue_delay_cycles(handle, 995)
    print "  Queued delay of %d cycles after second byte (0xbe)." % cycles
    sys.stdout.flush()

    ch_spi_queue_byte(handle, 3, 0x00)
    cycles = ch_spi_queue_delay_cycles(handle, 20)
    print "  Queued delay of %d cycles after last byte (0x00)." % cycles
    sys.stdout.flush()

    ch_spi_queue_ss(handle, 0)
    ch_spi_queue_oe(handle, 0)

    ch_spi_batch_shift(handle, 1)


#==========================================================================
# USAGE INFORMATION
#==========================================================================
def print_usage ():
    print \
"""
usage: timing PORT BITRATE MODE

MODE possibilities are:
  mode 0 : pol = 0, phase = 0
  mode 1 : pol = 0, phase = 1
  mode 2 : pol = 1, phase = 0
  mode 3 : pol = 1, phase = 1
"""


#==========================================================================
# MAIN PROGRAM
#==========================================================================
if (len(sys.argv) < 4):
    print_usage()
    sys.exit(1)

port     = int(sys.argv[1])
bitrate  = int(sys.argv[2])
mode     = int(sys.argv[3])

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

_timing(handle)

# Close and exit
ch_close(handle)
sys.exit(0)
