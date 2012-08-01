/*=========================================================================
| (C) 2007-2008  Total Phase, Inc.
|--------------------------------------------------------------------------
| Project : Cheetah Sample Code
| File    : blast.cs
|--------------------------------------------------------------------------
| Blast SPI data using the Cheetah host adapter
|--------------------------------------------------------------------------
| Redistribution and use of this file in source and binary forms, with
| or without modification, are permitted.
|
| THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
| "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
| LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
| FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
| COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
| INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
| BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
| LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
| CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
| LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
| ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
| POSSIBILITY OF SUCH DAMAGE.
 ========================================================================*/
using System;
using System.IO;
using TotalPhase;
 

/*=========================================================================
| CLASS
 ========================================================================*/
public class blast {


    /*=====================================================================
    | CONSTANTS
     ====================================================================*/
    // Uncomment to show the data returned by the shifting device
    private const bool SHOW_DATA = true;

    // Uncomment to hide the data returned by the shifting device
    // private const bool SHOW_DATA = false;

    // Add a delay between bytes by changing this constant (in nanoseconds)
    private const int BYTE_DELAY = 0; // ns


    /*=====================================================================
    | UTILITY FUNCTIONS
     ====================================================================*/
    static ulong _timeMillis () {
         DateTime CurrTime = DateTime.Now;
         return ((ulong)CurrTime.Ticks / 10000);
    }


    /*=====================================================================
    | FUNCTIONS
     ====================================================================*/
    static void _blast (int handle, int length) {
        double elapsed;
        ulong start = _timeMillis();

        // Queue the read sequence
        CheetahApi.ch_spi_queue_clear(handle);

        CheetahApi.ch_spi_queue_oe(handle, 1);
        CheetahApi.ch_spi_queue_ss(handle, 0);
        CheetahApi.ch_spi_queue_ss(handle, 0x1);

        int delay = 0;
        int j;
        for (j = 0; j < length; ++j) {
            CheetahApi.ch_spi_queue_byte(handle, 1, (byte)(j & 0xff));
            delay = CheetahApi.ch_spi_queue_delay_ns(handle, BYTE_DELAY);
        }
        Console.Write("Queued delay of {0:d} ns between bytes.\n", delay);
        Console.Out.Flush();

        CheetahApi.ch_spi_queue_ss(handle, 0);
        CheetahApi.ch_spi_queue_oe(handle, 0);

        elapsed = ((double)(_timeMillis() - start)) / 1000;
        Console.Write("Took {0:f2} seconds to queue the batch.\n", elapsed);
        Console.Out.Flush();

        // Perform the shift
        int batch      = CheetahApi.ch_spi_batch_length(handle);
        byte[] data_in = new byte[batch];

        start = _timeMillis();
        int count = CheetahApi.ch_spi_batch_shift(handle, batch, data_in);

        elapsed = ((double)(_timeMillis() - start)) / 1000;
        Console.Write("Took {0:f2} seconds to shift the batch.\n", elapsed);
        Console.Out.Flush();

        if (count != batch) {
            Console.Write("Expected {0:d} bytes but only received {1:d} " +
                          "bytes\n", batch, count);
        }

        if (SHOW_DATA)
        {
            // Output the data to the screen
            Console.Write("\nData:");
            int i;
            for (i = 0; i < length; ++i) {
                if ((i&0x07) == 0)      Console.Write("\n{0:x4}:  ", i);
                Console.Write("{0:x2}/{1:x2} ", (i & 0xff), data_in[i]);
            }
            Console.Write("\n");
            Console.Out.Flush();
        }
    }


    /*======================================================================
    | USAGE INFORMATION
     =====================================================================*/
    static void print_usage ()
    {
        Console.Write(
"Usage: blast PORT BITRATE MODE BITORDER LENGTH\n" +
"\n" +
"  MODE possibilities are:\n" +
"    mode 0 : pol = 0, phase = 0\n" +
"    mode 1 : pol = 0, phase = 1\n" +
"    mode 2 : pol = 1, phase = 0\n" +
"    mode 3 : pol = 1, phase = 1\n" +
"\n" +
"  BITORDER should be 0 for MSB, 1 for LSB\n" +
"\n" +
"For product documentation and specifications, see www.totalphase.com.\n");
        Console.Out.Flush();
    }


   /*======================================================================
   | MAIN PROGRAM
    =====================================================================*/
    public static void Main (String[] args) {
        int handle     = 0;
        int port       = 0;      // open port 0 by default
        int bitrate    = 0;
        int mode       = 0;
        int bitorder   = 0;
        int length     = 0;

        if (args.Length < 5) {
            print_usage();
            Environment.Exit(1);
        }

        port     = Convert.ToInt32(args[0]);
        bitrate  = Convert.ToInt32(args[1]);
        mode     = Convert.ToInt32(args[2]);
        bitorder = Convert.ToInt32(args[3]);
        length   = Convert.ToInt32(args[4]);
     
        handle = CheetahApi.ch_open(port);
        if (handle <= 0) {
            Console.Error.Write(
                "Unable to open Cheetah device on port {0:d}\n", port);
            Console.Error.Write("Error code = {0:d} ({1:s})\n", handle, 
                                CheetahApi.ch_status_string(handle));
            Environment.Exit(1);
        }
        Console.Write("Opened Cheetah device on port {0:d}\n", port);

        Console.Write("Host interface is {0:s}\n",
                      ((CheetahApi.ch_host_ifce_speed(handle)) != 0) ? 
                          "high speed" : "full speed");

        // Ensure that the SPI subsystem is configured.
        // Make sure that the bitorder parameter is valid, defaulting to LSB
        CheetahSpiBitorder spi_bitorder =
            (bitorder == (int)CheetahSpiBitorder.CH_SPI_BITORDER_MSB) ?
            CheetahSpiBitorder.CH_SPI_BITORDER_MSB :
            CheetahSpiBitorder.CH_SPI_BITORDER_LSB;
        CheetahApi.ch_spi_configure(
            handle, 
            (CheetahSpiPolarity)(mode >> 1), 
            (CheetahSpiPhase)(mode & 1), 
            spi_bitorder,
            0x0);
        Console.Write(
            "SPI configuration set to mode {0:d}, {1:s} shift, " +
            "SS[2:0] active low\n", mode,
            (spi_bitorder == CheetahSpiBitorder.CH_SPI_BITORDER_MSB) ?
                "MSB" : "LSB");
        Console.Out.Flush();       

        // Power the target using the Cheetah adapter's power supply.
        CheetahApi.ch_target_power(handle, CheetahApi.CH_TARGET_POWER_ON);
        CheetahApi.ch_sleep_ms(100);

        // Set the bitrate.
        bitrate = CheetahApi.ch_spi_bitrate(handle, bitrate);
        Console.Write("Bitrate set to {0:d} kHz\n", bitrate);
        Console.Out.Flush();       

        _blast(handle, length);
        
        // Close and exit.
        CheetahApi.ch_close(handle);
    
        return;
    }
}
