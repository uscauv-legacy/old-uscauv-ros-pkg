#!/bin/env python
#==========================================================================
# (c) 2007  Total Phase, Inc.
#--------------------------------------------------------------------------
# Project : Cheetah Examples
# File    : async.py
#--------------------------------------------------------------------------
# Use the asynchronous interface of the Cheetah host adapter
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
#SHOW_DATA = 1

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
def _blast_async (handle, txnlen, iter):

    # Make a simple queue to just assert OE.
    ch_spi_queue_clear(handle)
    ch_spi_queue_oe(handle, 1)
    ch_spi_batch_shift(handle, 0)
    
    # Queue the batch which is a sequence of SPI packets
    # (back-to-back) each of length 4.
    ch_spi_queue_clear(handle)

    count = 0
    data_out = array('B', [0 for i in range(4)])

    for i in range(txnlen):
        ch_spi_queue_ss(handle, 0x1)

        # Break count into bytes.
        data_out[0] = (count >> 24) & 0xff
        data_out[1] = (count >> 16) & 0xff
        data_out[2] = (count >>  8) & 0xff
        data_out[3] = (count >>  0) & 0xff
        
        count += 1
        
        ch_spi_queue_array(handle, data_out)
        ch_spi_queue_ss(handle, 0x0)
    
    start = _timeMillis()

    # Submit first batch.
    ch_spi_async_submit(handle)

    for n in range(iter-1):
        # Submit another batch, while the previous one is in
        # progress.  The application may even clear the current
        # batch queue and queue a different set of SPI
        # transactions before submitting this batch
        # asynchronously.
        ch_spi_async_submit(handle)
        
        # The application can now perform some other functions
        # while the Cheetah is both finishing the previous batch
        # and shifting the current batch as well.  In order to
        # keep the Cheetah's pipe full, this entire loop must
        # complete AND another batch must be submitted
        # before the current batch completes.
        ch_sleep_ms(25)
        
        # Collect the previous batch.
        ret = ch_spi_async_collect(handle, 0)
        elapsed = (_timeMillis() - start) / 1000.0
        print "collected batch #%03d in %.2lf seconds" % (n+1, elapsed)
        if (ret < 0):  print "status error: %s" % ch_status_string(ret)
        sys.stdout.flush()
        start = _timeMillis();
        
        # The current batch is now shifting out on the SPI
        # interface. The application can again do some more tasks
        # here but this entire loop must finish so that a new
        # batch is armed before the current batch completes.
        ch_sleep_ms(25);
    
    # Collect batch the last batch.
    (ret, data_in) = ch_spi_async_collect(handle, 0)

    elapsed = (_timeMillis() - start) / 1000.0
    print "collected batch #%03d in %.2lf seconds" % (n+1, elapsed)
    if (ret < 0):
        print "status error: %s" % ch_status_string(ret)
    sys.stdout.flush()


#==========================================================================
# USAGE INFORMATION
#==========================================================================
def print_usage ():
    print \
"""
usage: async PORT BITRATE TXN_LENGTH ITER

    TXN_LENGTH is the number of SPI packets, each of length 4, to queue in a
    single batch.

    ITER is the number of batches to process asynchronously.
"""


#==========================================================================
# MAIN PROGRAM
#==========================================================================
if (len(sys.argv) < 5):
    print_usage()
    sys.exit(1)

port     = int(sys.argv[1])
bitrate  = int(sys.argv[2])
txnlen   = int(sys.argv[3])
iter     = int(sys.argv[4])
    
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
mode = 0

ch_spi_configure(handle, (mode >> 1), mode & 1, CH_SPI_BITORDER_MSB, 0x0)
print "SPI configuration set to mode %d, MSB shift, SS[2:0] active low" % mode
sys.stdout.flush()

# Power the target using the Cheetah adapter's power supply.
ch_target_power(handle, CH_TARGET_POWER_ON)
ch_sleep_ms(100)

# Set the bitrate.
bitrate = ch_spi_bitrate(handle, bitrate)
print "Bitrate set to %d kHz" % bitrate
sys.stdout.flush()

_blast_async(handle, txnlen, iter)

# Close and exit.
ch_close(handle)
sys.exit(0)


