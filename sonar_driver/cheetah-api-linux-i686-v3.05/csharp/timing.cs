/*=========================================================================
| (C) 2007-2008  Total Phase, Inc.
|--------------------------------------------------------------------------
| Project : Cheetah Sample Code
| File    : timing.cs
|--------------------------------------------------------------------------
| Demonstrate user specified timings and delays
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
public class timing {


    /*=====================================================================
    | FUNCTIONS
     ====================================================================*/
    static void _timing (int handle) {
        int cycles;
        int ns;

        byte[] noresult = new byte[1];

        // Test the SS timing
        CheetahApi.ch_spi_queue_clear(handle);
        Console.Write("Testing inter-SS delays...\n");
        Console.Out.Flush();
        CheetahApi.ch_spi_queue_oe(handle, 1);
        CheetahApi.ch_spi_queue_ss(handle, 0x1);

        cycles = CheetahApi.ch_spi_queue_delay_cycles(handle, 51);
        Console.Write("  Queued delay of {0:d} cycles within first SS " + 
                      "assert/deassert.\n", cycles);
        Console.Out.Flush();
        CheetahApi.ch_spi_queue_ss(handle, 0);

        CheetahApi.ch_spi_queue_ss(handle, 0x1);

        ns = CheetahApi.ch_spi_queue_delay_ns(handle, 1500000);
        Console.Write("  Queued delay of {0:d} ns within second SS " + 
                      "assert/deassert.\n", ns);
        Console.Out.Flush();
        CheetahApi.ch_spi_queue_ss(handle, 0);
        
        CheetahApi.ch_spi_batch_shift(handle, 0, noresult);

        // Test data timing in read mode
        Console.Write("Testing inter-data (read) delays...\n");
        Console.Out.Flush();
        CheetahApi.ch_spi_queue_clear(handle);
        CheetahApi.ch_spi_queue_ss(handle, 0x1);

        CheetahApi.ch_spi_queue_byte(handle, 1, 0xca);
        ns = CheetahApi.ch_spi_queue_delay_ns(handle, 250000);
        Console.Write("  Queued delay of {0:d} ns after first byte " + 
                      "(0xca).\n", ns);
        Console.Out.Flush();

        CheetahApi.ch_spi_queue_byte(handle, 2, 0xfe);
        cycles = CheetahApi.ch_spi_queue_delay_cycles(handle, 995);
        Console.Write("  Queued delay of {0:d} cycles after second byte " + 
                      "(0xfe).\n", cycles);
        Console.Out.Flush();

        CheetahApi.ch_spi_queue_byte(handle, 3, 0x00);
        cycles = CheetahApi.ch_spi_queue_delay_cycles(handle, 20);
        Console.Write("  Queued delay of {0:d} cycles after last byte " + 
                      "(0x00).\n", cycles);
        Console.Out.Flush();

        CheetahApi.ch_spi_queue_ss(handle, 0);

        byte[] data_in = new byte[6];
        CheetahApi.ch_spi_batch_shift(handle, 6, data_in);

        
        // Test data timing with write mode
        Console.Write("Testing inter-data (write) delays...\n");
        Console.Out.Flush();
        CheetahApi.ch_spi_queue_clear(handle);
        CheetahApi.ch_spi_queue_ss(handle, 0x1);

        CheetahApi.ch_spi_queue_byte(handle, 1, 0xba);
        ns = CheetahApi.ch_spi_queue_delay_ns(handle, 80000);
        Console.Write("  Queued delay of {0:d} ns after first byte " + 
                      "(0xba).\n", ns);
        Console.Out.Flush();

        CheetahApi.ch_spi_queue_byte(handle, 2, 0xbe);
        cycles = CheetahApi.ch_spi_queue_delay_cycles(handle, 995);
        Console.Write("  Queued delay of {0:d} cycles after second byte " + 
                      "(0xbe).\n", cycles);
        Console.Out.Flush();

        CheetahApi.ch_spi_queue_byte(handle, 3, 0x00);
        cycles = CheetahApi.ch_spi_queue_delay_cycles(handle, 20);
        Console.Write("  Queued delay of {0:d} cycles after last byte " + 
                      "(0x00).\n", cycles);
        Console.Out.Flush();

        CheetahApi.ch_spi_queue_ss(handle, 0);
        CheetahApi.ch_spi_queue_oe(handle, 0);

        CheetahApi.ch_spi_batch_shift(handle, 1, data_in);
    }


    /*=====================================================================
    | USAGE INFORMATION
     ====================================================================*/
    static void print_usage () {
        Console.Write(
"Usage: timing PORT BITRATE MODE\n" +
"\n" +
"  MODE possibilities are:\n" +
"    mode 0 : pol = 0, phase = 0\n" +
"    mode 1 : pol = 0, phase = 1\n" +
"    mode 2 : pol = 1, phase = 0\n" +
"    mode 3 : pol = 1, phase = 1\n" +
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
     
        if (args.Length < 3) {
            print_usage();
            Environment.Exit(1);
        }

        port     = Convert.ToInt32(args[0]);
        bitrate  = Convert.ToInt32(args[1]);
        mode     = Convert.ToInt32(args[2]);

        // Open the device
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

        // Ensure that the SPI subsystem is configured
        byte bitorder = 0;
        CheetahApi.ch_spi_configure(
            handle, 
            (TotalPhase.CheetahSpiPolarity)(mode >> 1), 
            (TotalPhase.CheetahSpiPhase)(mode & 1), 
            (TotalPhase.CheetahSpiBitorder)~~bitorder, 
            0x0);

        Console.Write("SPI configuration set to mode {0:d}, {1:s} shift, " + 
                      "SS[2:0] active low\n",
                      mode, (bitorder == 0) ? "MSB" : "LSB");
        Console.Out.Flush();       

        // Power the target using the Cheetah adapter's power supply
        CheetahApi.ch_target_power(handle, CheetahApi.CH_TARGET_POWER_ON);
        CheetahApi.ch_sleep_ms(100);

        // Set the bitrate
        bitrate = CheetahApi.ch_spi_bitrate(handle, bitrate);
        Console.Write("Bitrate set to {0:d} kHz\n", bitrate);
        Console.Out.Flush();       

        _timing(handle);
        
        // Close and exit
        CheetahApi.ch_close(handle);
    
        return;
    }
}
