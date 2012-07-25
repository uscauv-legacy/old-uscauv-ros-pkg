/*=========================================================================
| (C) 2007-2008  Total Phase, Inc.
|--------------------------------------------------------------------------
| Project : Cheetah Sample Code
| File    : eeprom.cs
|--------------------------------------------------------------------------
| Perform simple read and write operations to an SPI EEPROM device
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
public class eeprom {


    /*=====================================================================
    | CONSTANTS
     ====================================================================*/
    private const int PAGE_SIZE = 32;  // Should be a power of 2
    private const int PAGE_WRITE_BATCH_SIZE =  32;
    private const int PAGE_WRITE_DELAY_NS   =  5000000;


    /*=====================================================================
    | FUNCTIONS
     ====================================================================*/
    static void _writeMemory (int handle, ushort addr, ushort length, int zero)
    {
        short i, j, n;
        int  batch, count;

        byte[] data_out = new byte[3+PAGE_SIZE];

        byte[] noresult = new byte[1];
        
        // Write to the SPI EEPROM
        // 
        // The AT25080A EEPROM has 32 byte pages.  Data can be written
        // in pages, to reduce the number of overall SPI transactions
        // executed through the Cheetah adapter.
        n = 0;
        while (n < length) {
            CheetahApi.ch_spi_queue_clear(handle);
            CheetahApi.ch_spi_queue_oe(handle, 1);

            // Send PAGE_WRITE_BATCH_SIZE number of pages to the Cheetah per 
            // batch shift
            for (i = 0; i < PAGE_WRITE_BATCH_SIZE; i++) {
                if (n >= length)
                    break;

                // Send write enable command
                CheetahApi.ch_spi_queue_ss(handle, 0x1);
                CheetahApi.ch_spi_queue_byte(handle, 1, 0x06);
                CheetahApi.ch_spi_queue_ss(handle, 0);

                // Assemble the write command and address
                Console.Write("addr = 0x{0:x4}; ", addr);
                data_out[0] = (byte)0x02;
                data_out[1] = (byte)((addr >> 8) & 0xff);
                data_out[2] = (byte)((addr >> 0) & 0xff);

                // Assemble the data
                j = 3;
                do {
                    data_out[j++] = (zero != 0) ? (byte)0 : (byte)n;
                    ++addr; ++n;
                } while ( (n < length) && ((addr & (PAGE_SIZE-1)) != 0) );

                Console.Write("num bytes = {0:d}\n", j-3);
                Console.Out.Flush();

                // Queue the write transaction
                CheetahApi.ch_spi_queue_ss(handle, 0x1);
                CheetahApi.ch_spi_queue_array(handle, j, data_out);
                CheetahApi.ch_spi_queue_ss(handle, 0);

                // Put in a wait for the write cycle time
                CheetahApi.ch_spi_queue_delay_ns(handle, PAGE_WRITE_DELAY_NS);
            }

            // Shift the page writes
            // Don't need the results back from the shift
            Console.Write("Shifting data\n");
            Console.Out.Flush();
            batch = CheetahApi.ch_spi_batch_length(handle);
            count = CheetahApi.ch_spi_batch_shift(handle, 0, noresult);
            if (count != batch) {
                Console.Write("Expected {0:d} bytes but only received " + 
                              "{1:d} bytes\n", batch, count);
                return;
            }
            Console.Write("Shift complete\n");
            Console.Out.Flush();
        }
    }

    static void _readMemory (int handle, ushort addr, ushort length)
    {
        int count;
        int i;

        byte[] data_in = new byte[length+3];
        for (int j = 0; j < length + 3; j++) data_in[j] = 0;         

        CheetahApi.ch_spi_queue_clear(handle);
        CheetahApi.ch_spi_queue_oe(handle, 1);

        // Queue the read command, address, and data
        CheetahApi.ch_spi_queue_ss(handle, 0x1);
        CheetahApi.ch_spi_queue_byte(handle, 1, 0x03);
        CheetahApi.ch_spi_queue_byte(handle, 1, (byte)((addr >> 8) & 0xff));
        CheetahApi.ch_spi_queue_byte(handle, 1, (byte)((addr >> 0) & 0xff));
        CheetahApi.ch_spi_queue_byte(handle, length, 0x00);
        CheetahApi.ch_spi_queue_ss(handle, 0);

        count = CheetahApi.ch_spi_batch_shift(handle, length+3, data_in);

        if (count < 0) {
            Console.Write("error: {0:s}\n",
                          CheetahApi.ch_status_string(count));
        }
        else if (count != length+3) {
            Console.Write("error: read {0:d} bytes (expected {1:d})\n", 
                          count-3, length);
        }

        // Dump the data to the screen
        Console.Write("\nData read from device:");
        for (i=0; i < length; ++i) {
            if ((i&0x0f) == 0)  Console.Write("\n{0:x4}:  ", addr+i);

            Console.Write("{0:x2} ", data_in[i+3] & 0xff);

            if (((i+1)&0x07) == 0)  Console.Write(" ");
        }
        Console.Write("\n");
        Console.Out.Flush();
    }


    /*=========================================================================
    | USAGE INFORMATION
     ========================================================================*/
    static void print_usage ()
    {
        Console.Write(
"Usage: eeprom PORT BITRATE MODE read  ADDR LENGTH\n" +
"Usage: eeprom PORT BITRATE MODE write ADDR LENGTH\n" +
"Usage: eeprom PORT BITRATE MODE zero  ADDR LENGTH\n" +
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


   /*=========================================================================
   | MAIN PROGRAM
    ========================================================================*/
    public static void Main (String[] args)
    {
        int handle      = 0;
        int port        = 0;      // open port 0 by default
        int bitrate     = 0;
        int mode        = 0;
        ushort addr     = 0;
        ushort length   = 0;
        String command  = "";

        if (args.Length < 6)
        {
            print_usage();
            Environment.Exit(1);
        }

        port     = Convert.ToInt32(args[0]);
        bitrate  = Convert.ToInt32(args[1]);
        mode     = Convert.ToInt32(args[2]);
        command  = args[3];
        addr     = Convert.ToUInt16(args[4]);
        length   = Convert.ToUInt16(args[5]);

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

        CheetahApi.ch_spi_configure(
            handle, 
            (TotalPhase.CheetahSpiPolarity)(mode >> 1), 
            (TotalPhase.CheetahSpiPhase)(mode & 1), 
            CheetahSpiBitorder.CH_SPI_BITORDER_MSB, 0x0);

        Console.Write("SPI configuration set to mode {0:d}, {1:s} shift, " + 
                      "SS[2:0] active low\n", mode, "MSB");
        Console.Out.Flush();       

        // Power the target using the Cheetah adapter's power supply
        CheetahApi.ch_target_power(handle, CheetahApi.CH_TARGET_POWER_ON);
        CheetahApi.ch_sleep_ms(100);

        // Set the bitrate
        bitrate = CheetahApi.ch_spi_bitrate(handle, bitrate);
        Console.Write("Bitrate set to {0:d} kHz\n", bitrate);
        Console.Out.Flush();       

        byte[] noresult = new byte[1];

        // Shift a dummy byte to clear the EEPROM state
        CheetahApi.ch_spi_queue_clear(handle);
        CheetahApi.ch_spi_queue_oe(handle, 1);
        CheetahApi.ch_spi_queue_ss(handle, 0x1);
        CheetahApi.ch_spi_queue_byte(handle, 1, 0x00);
        CheetahApi.ch_spi_queue_ss(handle, 0);
        CheetahApi.ch_spi_batch_shift(handle, 0, noresult);

        // Perform the requested operation
        if (command == "write") {
            _writeMemory(handle, addr, length, 0);
            Console.Write("Wrote to EEPROM\n");
        }
        else if (command == "read") {
            _readMemory(handle, addr, length);
        }
        else if (command == "zero") {
            _writeMemory(handle, addr, length, 1);
            Console.Write("Zeroed EEPROM\n");
        }
        else {
            Console.Write("unknown command: {0:s}\n", command);
        }

        // Close and exit
        CheetahApi.ch_close(handle);
    
        return;
    }
}
