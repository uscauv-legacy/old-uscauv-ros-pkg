/*=========================================================================
| (C) 2007-2008  Total Phase, Inc.
|--------------------------------------------------------------------------
| Project : Cheetah Sample Code
| File    : flash.cs
|--------------------------------------------------------------------------
| Read from, erase, write to, and verify data contents of the High-Speed
| SPI Flash board
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
public class flash {


    /*=====================================================================
    | CONSTANTS
     ====================================================================*/

    // When to stop data verify
    private const int INTEGRITY_CHECK_NUM_BYTES =  0x1000000;     

    // Flash size divided by 4
    private const int INTEGRITY_LOOP_SIZE       =  (0x400000/4);

    // If page size is larger than 1K, _write function will have to be
    // modified to take into account the possibility of starting the write
    // in the middle of a page.
    private const int PAGE_SIZE                  = 256;
    private const int PAGE_WRITE_BATCH_SIZE      = 16;
    private const int PAGE_PROGRAM_CYCLE_TIME_NS = 1400000;

    private const int COMMAND_READ   = 0;
    private const int COMMAND_WRITE  = 1;
    private const int COMMAND_ERASE  = 2;
    private const int COMMAND_VERIFY = 3;


    /*=====================================================================
    | FUNCTIONS
     ====================================================================*/
    static ulong _timeMicroseconds ()
    {
         DateTime CurrTime = DateTime.Now;
         return ((ulong)CurrTime.Ticks / 10);
    }

    static int _read (int handle, int addr, int length) {
        // Convert address and length from KB to bytes
        addr   *= 1024;
        length *= 1024;

        // Create the buffer to receive the flash values into.
        byte[] data_in = new byte[length];
        for (int j = 0; j < length; j++) data_in[j] = 0;  

        byte[] noresult = new byte[1];
        
        CheetahApi.ch_spi_queue_clear(handle);
        CheetahApi.ch_spi_queue_oe(handle, 1);
        
        // Set slave select to deasserted state, in case it was left
        // low by a previously interrupted transaction (ctrl-c).  This
        // will reset the state machine inside the flash.
        CheetahApi.ch_spi_queue_ss(handle, 0);
        CheetahApi.ch_spi_queue_ss(handle, 1);

        // Queue fast read command code.
        CheetahApi.ch_spi_queue_byte(handle, 1, 0x0b);

        // Queue 3 bytes of address.
        CheetahApi.ch_spi_queue_byte(handle, 1, (byte)(addr >> 16));
        CheetahApi.ch_spi_queue_byte(handle, 1, (byte)(addr >>  8));
        CheetahApi.ch_spi_queue_byte(handle, 1, (byte)(addr >>  0));

        // Queue dummy byte.
        CheetahApi.ch_spi_queue_byte(handle, 1, 0);

        // Shift the queued fast read command.
        int count = CheetahApi.ch_spi_batch_shift(handle, 0, data_in);
        if (count != 5) {
            Console.Error.Write("Expected 5 bytes from initial shift\n");
            return 1;
        }

        // Read the data.
        // Set the value to send while reading the flash.
        CheetahApi.ch_spi_queue_clear(handle);
        CheetahApi.ch_spi_queue_byte(handle, length, 0x00);
        count = CheetahApi.ch_spi_batch_shift(handle, length, data_in);
        if (count != length) {
            Console.Error.Write("Expected {0:d} bytes from read shift " +
                                "(got {1:d})\n", length, count);
            return 1;
        }

        // Dump the data to the screen
        int i;
        Console.Write("\nData read from device:");
        for (i=0; i < length; ++i) {
            if ((i&0x0f) == 0)      Console.Write("\n{0:x4}:  ", addr+i);
            Console.Write("{0:x2} ", data_in[i] & 0xff);
            if (((i+1)&0x07) == 0)  Console.Write(" ");
        }
        Console.Write("\n");
        Console.Out.Flush();

        // Clear the state of the bus.
        CheetahApi.ch_spi_queue_clear(handle);
        CheetahApi.ch_spi_queue_ss(handle, 0);
        CheetahApi.ch_spi_queue_oe(handle, 0);
        CheetahApi.ch_spi_batch_shift(handle, 0, noresult);

        return 0;
    }

    static int _write (int handle, int addr, int length)
    {
        // Buffer for outgoing data.
        byte[] data_page = new byte[4+PAGE_SIZE];

        byte[] noresult = new byte[1];

        // Reset the state of the bus.
        CheetahApi.ch_spi_queue_clear(handle);
        CheetahApi.ch_spi_queue_ss(handle, 0);
        CheetahApi.ch_spi_queue_oe(handle, 1);
        CheetahApi.ch_spi_batch_shift(handle, 0, noresult);
        
        // Convert address and length from KB to bytes
        addr   *= 1024;
        length *= 1024;

        // Set the starting counter based on the address.
        int val = addr/4;

        while (length != 0) {
            // Start the write sequence.
            CheetahApi.ch_spi_queue_clear(handle);
            CheetahApi.ch_spi_queue_oe(handle, 1);

            // Send PAGE_WRITE_BATCH_SIZE number of pages to the Cheetah per 
            // batch shift.
            int i;
            for (i = 0; i < PAGE_WRITE_BATCH_SIZE; i++) {
                // Check if we've reached the end.
                if (length == 0)
                    break;
                
                // Queue the write enable instruction for the flash.
                CheetahApi.ch_spi_queue_ss(handle, 0x1);
                CheetahApi.ch_spi_queue_byte(handle, 1, 0x06);
                CheetahApi.ch_spi_queue_ss(handle, 0);

                // Queue the write instruction for the flash.
                CheetahApi.ch_spi_queue_ss(handle, 0x1);
                data_page[0] = 0x02;
                data_page[1] = (byte)((addr >> 16) & 0xff);
                data_page[2] = (byte)((addr >>  8) & 0xff);
                data_page[3] = (byte)((addr >>  0) & 0xff);

                Console.Write("addr = 0x{0:x6}; num bytes = {1:d}\n", 
                                addr, PAGE_SIZE);
                Console.Out.Flush();

                // Set the data to be written to the flash to incrementing
                // 32-bit values starting from 0.
                int j = 0;
                while (j < PAGE_SIZE) {
                    data_page[4+j+0] = (byte)0;
                    data_page[4+j+1] = (byte)((val >> 16) & 0xff);
                    data_page[4+j+2] = (byte)((val >>  8) & 0xff);
                    data_page[4+j+3] = (byte)((val >>  0) & 0xff);
                    j += 4;
                    ++val;
                }

                CheetahApi.ch_spi_queue_array(handle, 4+PAGE_SIZE, data_page);
                CheetahApi.ch_spi_queue_ss(handle, 0);

                // Give the flash time to commit the written values.
                // Using ch_spi_queue_delay_ns is much more accurate than
                // using ch_sleep_ms.
                CheetahApi.ch_spi_queue_delay_ns(handle,
                                                 PAGE_PROGRAM_CYCLE_TIME_NS);

                addr += PAGE_SIZE;
                length -= PAGE_SIZE;
            }

            // Shift out the write command.  (Don't need the data back.)
            Console.Write("Shifting data\n");
            Console.Out.Flush();
            int batch = CheetahApi.ch_spi_batch_length(handle);
            int count = CheetahApi.ch_spi_batch_shift(handle, 0, noresult);
            if (count != batch) {
                Console.Write("Expected {0:d} bytes but only received " +
                              "{1:d} bytes\n", batch, count);
                return 1;
            }
            Console.Write("Shift complete\n");
            Console.Out.Flush();
        }

        // Reset the state of the bus.
        CheetahApi.ch_spi_queue_clear(handle);
        CheetahApi.ch_spi_queue_oe(handle, 0);
        CheetahApi.ch_spi_batch_shift(handle, 0, noresult);

        return 0;
    }

    static int _erase (int handle, int sector, int num)
    {
        byte[] noresult = new byte[1];

        // Reset the state of the bus.
        CheetahApi.ch_spi_queue_clear(handle);
        CheetahApi.ch_spi_queue_ss(handle, 0);
        CheetahApi.ch_spi_queue_oe(handle, 0);
        CheetahApi.ch_spi_batch_shift(handle, 0, noresult);

        int eraseAll = 0;
        if (sector == 0 && num == 64)
            eraseAll = 1;

        String str;
        if (eraseAll != 0)  str = "Bulk";
        else           str = "Block";

        while (num != 0) {
            // Make sure the sector is a valid one.
            if (sector < 0 || sector > 63)
                break;

            int addr = sector << 16;
            if (eraseAll == 0) {
                Console.Write("Erasing sector {0:d2} (bytes 0x{1:x6} " + 
                              "to 0x{2:x6})...\n",
                              sector, addr, addr | 0xffff);
                Console.Out.Flush();
            }
            else {
                Console.Write("Erasing entire device...\n");
                Console.Out.Flush();
            }

            // Start the erase sequence.
            CheetahApi.ch_spi_queue_clear(handle);
            CheetahApi.ch_spi_queue_oe(handle, 1);
        
            // Queue the write enable instruction for the flash.
            CheetahApi.ch_spi_queue_ss(handle, 0x1);
            CheetahApi.ch_spi_queue_byte(handle, 1, 0x06);
            CheetahApi.ch_spi_queue_ss(handle, 0);

            CheetahApi.ch_spi_queue_ss(handle, 0x1);

            if (eraseAll == 0) {
                // Queue the sector erase command.
                CheetahApi.ch_spi_queue_byte(handle, 1, 0xd8);
                CheetahApi.ch_spi_queue_byte(handle, 1, 
                                                (byte)((addr >> 16) & 0xff));
                CheetahApi.ch_spi_queue_byte(handle, 1, 
                                                (byte)((addr >>  8) & 0xff));
                CheetahApi.ch_spi_queue_byte(handle, 1, 
                                                (byte)((addr >>  0) & 0xff));
            }
            else {
                // Queue the bulk erase command.
                CheetahApi.ch_spi_queue_byte(handle, 1, 0xc7);
            }

            CheetahApi.ch_spi_queue_ss(handle, 0);
            // Shift the queued commands.  (Don't need the data back.)
            int batch = CheetahApi.ch_spi_batch_length(handle);
            int count = CheetahApi.ch_spi_batch_shift(handle, 0, noresult);
            if (count != batch) {
                Console.Write("Expected {0:d} bytes but only received " + 
                              "{1:d} bytes\n", batch, count);
                return 1;
            }

            ulong start = _timeMicroseconds();
            CheetahApi.ch_spi_queue_clear(handle);
            CheetahApi.ch_spi_queue_ss(handle, 0x1);
            CheetahApi.ch_spi_queue_byte(handle, 1, 0x05);
            CheetahApi.ch_spi_queue_byte(handle, 1, 0x00);
            CheetahApi.ch_spi_queue_ss(handle, 0);
            while (true) {
                CheetahApi.ch_sleep_ms(10);

                byte[] status_in = new byte[2];
                CheetahApi.ch_spi_batch_shift(handle, 2, status_in);
                if ((status_in[1] & 0x01) == 0)
                    break;
            }
            ulong end = _timeMicroseconds();
            Console.Write("{0:s} erase took {1:f3} seconds\n",
                   str, (double)(end-start)/1000000);
            Console.Out.Flush();

            if (eraseAll == 0) {
                ++sector;
                --num;
            }
            else {
                sector += 64;
                num     = 0;
            }
        }

        // Reset the state of the bus.
        CheetahApi.ch_spi_queue_clear(handle);
        CheetahApi.ch_spi_queue_oe(handle, 0);
        CheetahApi.ch_spi_batch_shift(handle, 0, noresult);
        
        return 0;
    }

    static void _printProgress (int percent,  double elapsedTime,
                                int blockNum, int    blocksize) 
    {
        String progressbar;
        int tenths = percent/5;
        progressbar = "[";
        for(int i = 0; i < tenths; i++)
           progressbar += " ";
        progressbar += "#";

        for(int i = 0; i < 20 - tenths; i++)
           progressbar += " ";
        progressbar += "]";

        Console.Write("\r{0:s} {1,3:d}% {2,7:d} KB {3:f2} seconds",
               progressbar, percent, 
               blocksize / 1024 * (blockNum+1),
               elapsedTime);
        Console.Out.Flush();
    }

    static int _verify (int handle, int length, int blocksize) 
    {
        if (length == 0 || blocksize == 0)
            return 2;
        byte[] noresult = new byte[1];

        int  iter = (length-1)/blocksize + 1;

        // Create the buffer to receive the flash values into.
        byte[] data_in = new byte[blocksize];
        for (int j = 0; j < blocksize; j++) data_in[j] = 0;  
  
        CheetahApi.ch_spi_queue_clear(handle);
        CheetahApi.ch_spi_queue_oe(handle, 1);
        
        // Set slave select to deasserted state, in case it was left
        // low by a previously interrupted transaction (ctrl-c).  This
        // will reset the state machine inside the flash.
        CheetahApi.ch_spi_queue_ss(handle, 0);
        CheetahApi.ch_spi_queue_ss(handle, 1);

        // Queue fast read command code.
        CheetahApi.ch_spi_queue_byte(handle, 1, 0x0b);

        // Queue 3 bytes of address.
        CheetahApi.ch_spi_queue_byte(handle, 1, 0);
        CheetahApi.ch_spi_queue_byte(handle, 1, 0);
        CheetahApi.ch_spi_queue_byte(handle, 1, 0);

        // Queue dummy byte.
        CheetahApi.ch_spi_queue_byte(handle, 1, 0);

        // Shift the queued fast read command.
        int count = CheetahApi.ch_spi_batch_shift(handle, 5, data_in);
        if (count != 5) {
            Console.Write("Expected 5 bytes from initial shift\n");
            return 1;
        }

        // Set the value to send while reading the flash.
        CheetahApi.ch_spi_queue_clear(handle);
        CheetahApi.ch_spi_queue_byte(handle, blocksize, 0x00);

        int i;
        ulong start = _timeMicroseconds();

        // Variables for the integrity check.
        uint integrity_counter = 0;
        int integrity_errors   = 0;
        int integrity_nb       = 0;

        _printProgress(0, 0, 0, blocksize);

        // Read one block at a time.
        for (i = 0; i < iter; ++i) {
            // Read the next block of data.
            count = CheetahApi.ch_spi_batch_shift(handle, blocksize, 
                                                  data_in);
            if (count != blocksize) {
                Console.Write("Expected {0:d} bytes from block shift " +
                              "(got {1:d})\n",
                        blocksize, count);
                break;
            }
            
            // Check if the data in the flash matches a predefined
            // sequence.  Namely, there should be a running 32 bit
            // counter in the data.
            int j = 0;
            while (integrity_nb < INTEGRITY_CHECK_NUM_BYTES && j < count) {
                uint val = (uint)((data_in[j+0] << 24) |
                                  (data_in[j+1] << 16) |
                                  (data_in[j+2] <<  8) |
                                  (data_in[j+3] <<  0));

                if (val != integrity_counter % INTEGRITY_LOOP_SIZE)
                    ++integrity_errors;
                
                ++integrity_counter;
                integrity_nb += 4;
                j += 4;
            }

            // Print out the progress.
            ulong currTime = _timeMicroseconds();

            _printProgress((i+1) * 100 / iter,
                           ((double)(currTime - start)) / 1000000,
                           i, blocksize);
        }

        // Clear the state of the flash.
        CheetahApi.ch_spi_queue_clear(handle);
        CheetahApi.ch_spi_queue_ss(handle, 0);
        CheetahApi.ch_spi_queue_oe(handle, 0);
        CheetahApi.ch_spi_batch_shift(handle, 0, noresult);

        Console.Out.Flush();
        Console.Error.Write("\nThere were {0:d} data errors.\n", 
                            integrity_errors);

        return (integrity_errors != 0 ) ? 2 : 0;
    }


    /*=====================================================================
    | USAGE INFORMATION
     ====================================================================*/
    static void print_usage ()
    {
        Console.Write(
"FLASH - utility for exercising the High-Speed SPI Flash board\n" +
"usage: flash PORT BITRATE MODE read   ADDR_KB   LEN_KB\n" +
"usage: flash PORT BITRATE MODE write  ADDR_KB   LEN_KB\n" +
"usage: flash PORT BITRATE MODE erase  SECTOR    NUM\n" +
"usage: flash PORT BITRATE MODE verify LENGTH_KB BLOCK_SIZE_KB\n"+
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
    public static void Main (String[] args)
    {
        int handle      = 0;
        int port        = 0;      // open port 0 by default
        int bitrate     = 0;
        int mode        = 0;
        ushort arg5     = 0;
        ushort arg6     = 0;
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
        arg5     = Convert.ToUInt16(args[4]);
        arg6     = Convert.ToUInt16(args[5]);

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

        CheetahApi.ch_spi_configure(
            handle, 
            (TotalPhase.CheetahSpiPolarity)(mode >> 1), 
            (TotalPhase.CheetahSpiPhase)(mode & 1), 
            CheetahSpiBitorder.CH_SPI_BITORDER_MSB, 0x0);

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


        // Determine which command transaction type was requested.
        int commandID = -1;
        if (command == "read") {
            commandID = COMMAND_READ;
        }
        else if (command == "write") {
            commandID = COMMAND_WRITE;
        }
        else if (command == "erase") {
            commandID = COMMAND_ERASE;
        }
        else if (command == "verify") {
            commandID = COMMAND_VERIFY;
        }
        else {
            Console.Write("Unknown option: {0:s}\n", command);
            Console.Write("Valid options are: read, write, erase, " + 
                          "and verify\n\n");
            print_usage();
            Environment.Exit(1);
        }

        // Execute the appropriate command.
        switch (commandID) {
          case COMMAND_READ:
            _read(handle, arg5, arg6);
            break;

          case COMMAND_WRITE:
            _write(handle, arg5, arg6); 
            break;

          case COMMAND_ERASE:
            _erase(handle, arg5, arg6); 
            break;

          case COMMAND_VERIFY:
            _verify(handle, arg5 * 1024, arg6 * 1024);
            break;
        }
        
        // Close the device.
        CheetahApi.ch_close(handle);
        return;
    }
}
