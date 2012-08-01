/*=========================================================================
| (C) 2007-2008  Total Phase, Inc.
|--------------------------------------------------------------------------
| Project : Cheetah Sample Code
| File    : async.cs
|--------------------------------------------------------------------------
| Use the asynchronous interface of the Cheetah host adapter
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
public class async {

    /*=====================================================================
    | CONSTANTS
     ====================================================================*/
    // Add a delay between bytes by changing this constant (in nanoseconds)
    private const int BYTE_DELAY = 0; // ns


    /*======================================================================
    | UTILITY FUNCTIONS
     =====================================================================*/
    static ulong _timeMillis () {
         DateTime CurrTime = DateTime.Now;
         return ((ulong)CurrTime.Ticks / 10000);
    }


    /*======================================================================
    | FUNCTIONS
     =====================================================================*/
    static void _blast_async (int handle, int txnlen, int iter) {
        double elapsed = 0;

        byte[] noresult = new byte[1];

        // Make a simple queue to just assert OE.
        CheetahApi.ch_spi_queue_clear(handle);
        CheetahApi.ch_spi_queue_oe(handle, (byte)1);
        CheetahApi.ch_spi_batch_shift(handle, 0, noresult);

        
        // Queue the batch which is a sequence of SPI packets
        // (back-to-back) each of length 4.
        CheetahApi.ch_spi_queue_clear(handle);
        int i;
        int count = 0;
        byte[] data_out = new byte[4];
        for (i = 0; i < txnlen; ++i) {
            CheetahApi.ch_spi_queue_ss(handle, 0x1);
        
            data_out[0] = (byte)((count >> 24) & 0xff);
            data_out[1] = (byte)((count >> 16) & 0xff);
            data_out[2] = (byte)((count >>  8) & 0xff);
            data_out[3] = (byte)((count >>  0) & 0xff);
        
            ++count;
            
            CheetahApi.ch_spi_queue_array(handle, 4, data_out);
            CheetahApi.ch_spi_queue_ss(handle, 0x0);
        }
        
        ulong start = _timeMillis();

        // First, submit first batch 
        CheetahApi.ch_spi_async_submit(handle);

        int n, ret;
        for (n = 0; n < iter-1; ++n) {
            // Submit another batch, while the previous one is in
            // progress.  The application may even clear the current
            // batch queue and queue a different set of SPI
            // transactions before submitting this batch
            // asynchronously.
            CheetahApi.ch_spi_async_submit(handle);
            
            // The application can now perform some other functions
            // while the Cheetah is both finishing the previous batch
            // and shifting the current batch as well.  In order to
            // keep the Cheetah's pipe full, this entire loop must
            // complete AND another batch must be submitted
            // before the current batch completes.
            CheetahApi.ch_sleep_ms(25);
            
            // Collect the previous batch
            ret = CheetahApi.ch_spi_async_collect(handle, 0, noresult);
            elapsed = ((double)(_timeMillis() - start)) / 1000;
            Console.Write("collected batch #{0:d3} in {1:f2} seconds\n",
                                n+1, elapsed);
            if (ret < 0)  Console.Write("status error: {0:s}\n", 
                                CheetahApi.ch_status_string(ret));
            Console.Out.Flush();

            start = _timeMillis();
            
            // The current batch is now shifting out on the SPI
            // interface. The application can again do some more tasks
            // here but this entire loop must finish so that a new
            // batch is armed before the current batch completes.
            CheetahApi.ch_sleep_ms(25);
        }

        // Collect batch the last batch
        ret = CheetahApi.ch_spi_async_collect(handle, 0, noresult);
        elapsed = ((double)(_timeMillis() - start)) / 1000;
        Console.Write("collected batch #{0:d3} in {1:f2} seconds\n", 
                        n+1, elapsed);
        if (ret < 0)  Console.Write("status error: {0:s}\n", 
                                     CheetahApi.ch_status_string(ret));
        Console.Out.Flush();
    }    


    /*======================================================================
    | USAGE INFORMATION
     =====================================================================*/
    static void print_usage () {
        Console.Write(
"Usage: async PORT BITRATE TXN_LENGTH ITER\n" +
"  TXN_LENGTH is the number of SPI packets, each of length\n" +
"  4 to queue in a single batch.\n" +
"\n" +
"  ITER is the number of batches to process asynchronously.\n" + 
"\n" +
"For product documentation and specifications, see www.totalphase.com.\n");
        Console.Out.Flush();
    }


    /*======================================================================
    | MAIN PROGRAM
    =======================================================================*/
    public static void Main (String[] args) {
        int handle     = 0;
        int port       = 0;      // open port 0 by default
        int bitrate    = 0;
        int txnlen     = 0;
        int iter       = 0;

        if (args.Length < 4)
        {
            print_usage();
            Environment.Exit(1);
        }

        port     = Convert.ToInt32(args[0]);
        bitrate  = Convert.ToInt32(args[1]);
        txnlen   = Convert.ToInt32(args[2]);
        iter     = Convert.ToInt32(args[3]);


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
        byte mode = 0;
        CheetahApi.ch_spi_configure(
            handle, 
            (CheetahSpiPolarity)(mode >> 1), 
            (CheetahSpiPhase)(mode & 1), 
            CheetahSpiBitorder.CH_SPI_BITORDER_MSB,
            0x0);

        Console.Write("SPI configuration set to mode {0:d}, {1:s} shift, " + 
                      "SS[2:0] active low\n", mode, "MSB");
        Console.Out.Flush();       

        // Power the target using the Cheetah adapter's power supply.
        CheetahApi.ch_target_power(handle, CheetahApi.CH_TARGET_POWER_ON);
        CheetahApi.ch_sleep_ms(100);

        // Set the bitrate.
        bitrate = CheetahApi.ch_spi_bitrate(handle, bitrate);
        Console.Write("Bitrate set to {0:d} kHz\n", bitrate);
        Console.Out.Flush();       

        _blast_async(handle, txnlen, iter);
        
        // Close and exit.
        CheetahApi.ch_close(handle);
    
        return;
    }
}
