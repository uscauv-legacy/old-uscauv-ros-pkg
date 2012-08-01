/*=========================================================================
| (c) 2005-2007  Total Phase, Inc.
|--------------------------------------------------------------------------
| Project : Cheetah Sample Code
| File    : flash.c
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

//=========================================================================
// INCLUDES
//=========================================================================
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "cheetah.h"

#ifdef _WIN32
#include <time.h>
#else
#include <sys/time.h>
#endif


//=========================================================================
// CONSTANTS
//=========================================================================
#define INTEGRITY_CHECK_NUM_BYTES  0x1000000     // When to stop data verify
#define INTEGRITY_LOOP_SIZE        (0x400000/4)  // Flash size divided by 4

// If page size is larger than 1K, _write function will have to be modified
// to take into account the possibility of starting the write in the middle 
// of a page.
#define PAGE_SIZE                   256
#define PAGE_WRITE_BATCH_SIZE       16
#define PAGE_PROGRAM_CYCLE_TIME_NS  1400000

#define COMMAND_READ    0
#define COMMAND_WRITE   1
#define COMMAND_ERASE   2
#define COMMAND_VERIFY  3


//=========================================================================
// FUNCTIONS
//=========================================================================
static s64 _timeMicroseconds () {
#ifdef _WIN32
    return ((s64)clock()) * 1000000 / CLOCKS_PER_SEC;
#else
    struct timeval tv;
    gettimeofday(&tv, 0);
    return ((s64)tv.tv_sec * 1000000L) + (s64)(tv.tv_usec);
#endif
}

static void _printUsage () {
    printf("FLASH - utility for exercising the High-Speed SPI Flash board\n");
    printf("usage: flash PORT BITRATE MODE read   ADDR_KB   LEN_KB\n");
    printf("usage: flash PORT BITRATE MODE write  ADDR_KB   LEN_KB\n");
    printf("usage: flash PORT BITRATE MODE erase  SECTOR    NUM\n");
    printf("usage: flash PORT BITRATE MODE verify LENGTH_KB BLOCK_SIZE_KB\n");
    printf("\n");
    printf("MODE possibilities are:\n");
    printf("  mode 0 : pol = 0, phase = 0\n");
    printf("  mode 1 : pol = 0, phase = 1\n");
    printf("  mode 2 : pol = 1, phase = 0\n");
    printf("  mode 3 : pol = 1, phase = 1\n");
}

static int _connect (int port, int bitrate, int mode, Cheetah *handle) {
    // Open the device.
    *handle = ch_open(port);
    if (*handle <= 0) {
        printf("Unable to open Cheetah device on port %d\n", port);
        printf("Error code = %d\n", *handle);
        return 1;
    }

    printf("Opened Cheetah device on port %d\n", port);

    printf("Host interface is %s\n",
           (ch_host_ifce_speed(*handle)) ? "high speed" : "full speed");

    // Ensure that the SPI subsystem is configured.
    ch_spi_configure(*handle, (mode >> 1), mode & 1, CH_SPI_BITORDER_MSB, 0x0);
    printf("SPI configuration set to mode %d, MSB shift, SS[2:0] active low\n",
           mode);
    fflush(stdout);

    // Power the flash using the Cheetah adapter's power supply.
    ch_target_power(*handle, CH_TARGET_POWER_ON);
    ch_sleep_ms(100);

    // Set the bitrate.
    bitrate = ch_spi_bitrate(*handle, bitrate);
    if (bitrate < 0) {
        fprintf(stderr, "Could not set bitrate.\n");
        fprintf(stderr, "Error code = %d\n", bitrate);
        return 1;
    }
    printf("Bitrate set to %d kHz\n", bitrate);
    fflush(stdout);
    return 0;
}

static int _read (Cheetah handle, int addr, int length) {
    u08 *data_in;
    int  count;
    int  i;

    // Convert address and length from KB to bytes
    addr   *= 1024;
    length *= 1024;

    // Create the buffer to receive the flash values into.
    data_in = (u08 *)malloc(length);
    memset(data_in, 0, length);

    ch_spi_queue_clear(handle);
    ch_spi_queue_oe(handle, 1);
    
    // Set slave select to deasserted state, in case it was left
    // low by a previously interrupted transaction (ctrl-c).  This
    // will reset the state machine inside the flash.
    ch_spi_queue_ss(handle, 0);
    ch_spi_queue_ss(handle, 1);

    // Queue fast read command code.
    ch_spi_queue_byte(handle, 1, 0x0b);

    // Queue 3 bytes of address.
    ch_spi_queue_byte(handle, 1, (u08)(addr >> 16));
    ch_spi_queue_byte(handle, 1, (u08)(addr >>  8));
    ch_spi_queue_byte(handle, 1, (u08)(addr >>  0));

    // Queue dummy byte.
    ch_spi_queue_byte(handle, 1, 0);

    // Shift the queued fast read command.
    count = ch_spi_batch_shift(handle, 0, data_in);
    if (count != 5) {
        fprintf(stderr,"Expected 5 bytes from initial shift\n");
        return 1;
    }

    // Read the data.
    // Set the value to send while reading the flash.
    ch_spi_queue_clear(handle);
    ch_spi_queue_byte(handle, length, 0x00);
    count = ch_spi_batch_shift(handle, length, data_in);
    if (count != length) {
        fprintf(stderr, "Expected %d bytes from read shift (got %d)\n",
                length, count);
        return 1;
    }

    // Dump the data to the screen
    printf("\nData read from device:");
    for (i=0; i < length; ++i) {
        if ((i&0x0f) == 0)      printf("\n%04x:  ", addr+i);
        printf("%02x ", data_in[i] & 0xff);
        if (((i+1)&0x07) == 0)  printf(" ");
    }
    printf("\n");
    fflush(stdout);

    // Clear the state of the bus.
    ch_spi_queue_clear(handle);
    ch_spi_queue_ss(handle, 0);
    ch_spi_queue_oe(handle, 0);
    ch_spi_batch_shift(handle, 0, 0);

    // Free the packet
    free(data_in);

    return 0;
}

static int _write (Cheetah handle, int addr, int length) {
    // Buffer for outgoing data.
    u08 data_page[4+PAGE_SIZE];
    int i, j;
    int batch, count;
    u32 val;

    // Reset the state of the bus.
    ch_spi_queue_clear(handle);
    ch_spi_queue_ss(handle, 0);
    ch_spi_queue_oe(handle, 1);
    ch_spi_batch_shift(handle, 0, 0);
    
    // Convert address and length from KB to bytes
    addr   *= 1024;
    length *= 1024;

    // Set the starting counter based on the address.
    val = addr/4;

    while (length) {
        // Start the write sequence.
        ch_spi_queue_clear(handle);
        ch_spi_queue_oe(handle, 1);

        // Send PAGE_WRITE_BATCH_SIZE number of pages to the Cheetah per 
        // batch shift.
        for (i = 0; i < PAGE_WRITE_BATCH_SIZE; i++) {
            // Check if we've reached the end.
            if (!length)
                break;
            
            // Queue the write enable instruction for the flash.
            ch_spi_queue_ss(handle, 0x1);
            ch_spi_queue_byte(handle, 1, 0x06);
            ch_spi_queue_ss(handle, 0);

            // Queue the write instruction for the flash.
            ch_spi_queue_ss(handle, 0x1);
            data_page[0] = 0x02;
            data_page[1] = (addr >> 16) & 0xff;
            data_page[2] = (addr >>  8) & 0xff;
            data_page[3] = (addr >>  0) & 0xff;

            printf("addr = 0x%06x; num bytes = %d\n", addr, PAGE_SIZE);
            fflush(stdout);

            // Set the data to be written to the flash to incrementing
            // 32-bit values starting from 0.
            j = 0;
            while (j < PAGE_SIZE) {
                data_page[4+j+0] = 0;
                data_page[4+j+1] = (u08)((val >> 16) & 0xff);
                data_page[4+j+2] = (u08)((val >>  8) & 0xff);
                data_page[4+j+3] = (u08)((val >>  0) & 0xff);
                j += 4;
                ++val;
            }

            ch_spi_queue_array(handle, 4+PAGE_SIZE, data_page);
            ch_spi_queue_ss(handle, 0);

            // Give the flash time to commit the written values.
            // Using ch_spi_queue_delay_ns is much more accurate than
            // using ch_sleep_ms.
            ch_spi_queue_delay_ns(handle, PAGE_PROGRAM_CYCLE_TIME_NS);

            addr += PAGE_SIZE;
            length -= PAGE_SIZE;
        }

        // Shift out the write command.  (Don't need the data back.)
        printf("Shifting data\n");
        fflush(stdout);
        batch = ch_spi_batch_length(handle);
        count = ch_spi_batch_shift(handle, 0, 0);
        if (count != batch) {
            printf("Expected %d bytes but only received %d bytes\n",
                   batch, count);
            return 1;
        }
        printf("Shift complete\n");
        fflush(stdout);
    }

    // Reset the state of the bus.
    ch_spi_queue_clear(handle);
    ch_spi_queue_oe(handle, 0);
    ch_spi_batch_shift(handle, 0, 0);

    return 0;
}

static int _erase (Cheetah handle, int sector, int num) {
    int   addr;
    int   eraseAll = 0;
    int   batch, count;
    char *str;
    s64   start;
    s64   end;

    // Reset the state of the bus.
    ch_spi_queue_clear(handle);
    ch_spi_queue_ss(handle, 0);
    ch_spi_queue_oe(handle, 0);
    ch_spi_batch_shift(handle, 0, 0);

    if (sector == 0 && num == 64)
        eraseAll = 1;

    if (eraseAll)  str = "Bulk";
    else           str = "Block";

    while (num) {
        // Make sure the sector is a valid one.
        if (sector < 0 || sector > 63)
            break;

        addr = sector << 16;
        if (!eraseAll) {
            printf("Erasing sector %02d (bytes 0x%06x to 0x%06x)...\n",
                   sector, addr, addr | 0xffff);
            fflush(stdout);
        }
        else {
            printf("Erasing entire device...\n");
            fflush(stdout);
        }

        // Start the erase sequence.
        ch_spi_queue_clear(handle);
        ch_spi_queue_oe(handle, 1);
    
        // Queue the write enable instruction for the flash.
        ch_spi_queue_ss(handle, 0x1);
        ch_spi_queue_byte(handle, 1, 0x06);
        ch_spi_queue_ss(handle, 0);

        ch_spi_queue_ss(handle, 0x1);

        if (!eraseAll) {
            // Queue the sector erase command.
            ch_spi_queue_byte(handle, 1, 0xd8);
            ch_spi_queue_byte(handle, 1, (u08)((addr >> 16) & 0xff));
            ch_spi_queue_byte(handle, 1, (u08)((addr >>  8) & 0xff));
            ch_spi_queue_byte(handle, 1, (u08)((addr >>  0) & 0xff));
        }
        else {
            // Queue the bulk erase command.
            ch_spi_queue_byte(handle, 1, 0xc7);
        }

        ch_spi_queue_ss(handle, 0);
        // Shift the queued commands.  (Don't need the data back.)
        batch = ch_spi_batch_length(handle);
        count = ch_spi_batch_shift(handle, 0, 0);
        if (count != batch) {
            printf("Expected %d bytes but only received %d bytes\n",
                   batch, count);
            return 1;
        }

        start = _timeMicroseconds();
        ch_spi_queue_clear(handle);
        ch_spi_queue_ss(handle, 0x1);
        ch_spi_queue_byte(handle, 1, 0x05);
        ch_spi_queue_byte(handle, 1, 0x00);
        ch_spi_queue_ss(handle, 0);
        while (1) {
            u08 status_in[2];
            ch_sleep_ms(10);

            ch_spi_batch_shift(handle, 2, status_in);
            if (!(status_in[1] & 0x01))
                break;
        }
        end = _timeMicroseconds();
        printf("%s erase took %.03f seconds\n",
               str, (double)(end-start)/1000000);
        fflush(stdout);

        if (!eraseAll) {
            ++sector;
            --num;
        }
        else {
            sector += 64;
            num     = 0;
        }
    }

    // Reset the state of the bus.
    ch_spi_queue_clear(handle);
    ch_spi_queue_oe(handle, 0);
    ch_spi_batch_shift(handle, 0, 0);
    
    return 0;
}

static void _printProgress (int percent, double elapsedTime, int blockNum,  
                            int blocksize) 
{
    static char progressbar[32];
    int tenths = percent/5 + 1;
    sprintf(progressbar, "[%*c%*c", tenths, '#', 22-tenths, ']');
    printf("\r%s %3d%% %7d KB %.2lf seconds",
           progressbar, percent, 
           blocksize / 1024 * (blockNum+1),
           elapsedTime);
    fflush(stdout);
}

static int _verify (Cheetah handle, int length, int blocksize) 
{
    int  iter;
    int  count;
    int  i, j;
    u08 *data_in;
    s64  start, currTime;

    // Variables for the integrity check.
    u32 integrity_counter = 0;
    int integrity_errors = 0;
    int integrity_nb = 0;
    int isSucceed = 1;

    if (length == 0 || blocksize == 0)
        return 2;

    iter = (length-1)/blocksize + 1;

    // Create the buffer to receive the flash values into.
    data_in = (u08 *)malloc(blocksize);
    memset(data_in, 0, blocksize);

    ch_spi_queue_clear(handle);
    ch_spi_queue_oe(handle, 1);
    
    // Set slave select to deasserted state, in case it was left
    // low by a previously interrupted transaction (ctrl-c).  This
    // will reset the state machine inside the flash.
    ch_spi_queue_ss(handle, 0);
    ch_spi_queue_ss(handle, 1);

    // Queue fast read command code.
    ch_spi_queue_byte(handle, 1, 0x0b);

    // Queue 3 bytes of address.
    ch_spi_queue_byte(handle, 1, 0);
    ch_spi_queue_byte(handle, 1, 0);
    ch_spi_queue_byte(handle, 1, 0);

    // Queue dummy byte.
    ch_spi_queue_byte(handle, 1, 0);

    // Shift the queued fast read command.
    count = ch_spi_batch_shift(handle, 5, data_in);
    if (count != 5) {
        printf("Expected 5 bytes from initial shift\n");
        return 1;
    }

    // Set the value to send while reading the flash.
    ch_spi_queue_clear(handle);
    ch_spi_queue_byte(handle, blocksize, 0x00);

    start = _timeMicroseconds();
    _printProgress(0, 0, 0, blocksize);

    // Read one block at a time.
    for (i = 0; i < iter; ++i) {
        // Read the next block of data.
        count = ch_spi_batch_shift(handle, blocksize, data_in);
        if (count != blocksize) {
            fprintf(stderr, "Expected %d bytes from block shift (got %d)\n",
                    blocksize, count);
            isSucceed = 0;
            break;
        }
        
        // Check if the data in the flash matches a predefined
        // sequence.  Namely, there should be a running 32 bit
        // counter in the data.
        j = 0;
        while (integrity_nb < INTEGRITY_CHECK_NUM_BYTES && j < count)
        {
            unsigned int val = (data_in[j+0] << 24) |
                               (data_in[j+1] << 16) |
                               (data_in[j+2] <<  8) |
                               (data_in[j+3] <<  0);

            if (val != integrity_counter % INTEGRITY_LOOP_SIZE)
                ++integrity_errors;
            
            ++integrity_counter;
            integrity_nb += 4;
            j += 4;
        }

        // Print out the progress.
        currTime = _timeMicroseconds();

        _printProgress((i+1) * 100 / iter,
                       ((double)(currTime - start)) / 1000000,
                       i, blocksize);
    }

    // Clear the state of the flash.
    ch_spi_queue_clear(handle);
    ch_spi_queue_ss(handle, 0);
    ch_spi_queue_oe(handle, 0);
    ch_spi_batch_shift(handle, 0, 0);

    fflush(stdout);
    fprintf(stderr, "\nThere were %d data errors.\n", integrity_errors);

    // Release the buffer.
    free(data_in);

    return (integrity_errors) ? 2 : 0;
}

//=========================================================================
// MAIN PROGRAM
//=========================================================================
int main (int argc, char *argv[]) {
    Cheetah     handle;
    int         port    = 0;
    int         bitrate = 0;
    int         mode    = 0;
    const char *command;
    int         commandID = -1;
    int         ret = 0;

    // Print the syntax for usage.
    if (argc < 7) {
        _printUsage();
        return 1;
    }

    // Parse the arguments.
    port      = atoi(argv[1]);
    bitrate   = atoi(argv[2]);
    mode      = atoi(argv[3]);
    command   = argv[4];

    // Determine which command transaction type was requested.
    if (strcmp(command, "read") == 0) {
        commandID = COMMAND_READ;
    }
    else if (strcmp(command, "write") == 0) {
        commandID = COMMAND_WRITE;
    }
    else if (strcmp(command, "erase") == 0) {
        commandID = COMMAND_ERASE;
    }
    else if (strcmp(command, "verify") == 0) {
        commandID = COMMAND_VERIFY;
    }
    else {
        printf("Unknown option: %s\n", command);
        printf("Valid options are: read, write, erase, and verify\n\n");
        _printUsage();
        return 1;
    }

    // Connect to and configure the Cheetah.
    if (_connect(port, bitrate, mode, &handle)) {
        return 1;
    }

    // Execute the appropriate command.
    switch (commandID) {
      case COMMAND_READ:
        ret = _read(handle, atoi(argv[5]), atoi(argv[6]));
        break;

      case COMMAND_WRITE:
        ret = _write(handle, atoi(argv[5]), atoi(argv[6]));
        break;

      case COMMAND_ERASE:
        ret = _erase(handle, atoi(argv[5]), atoi(argv[6]));
        break;

      case COMMAND_VERIFY:
        ret = _verify(handle, atoi(argv[5]) * 1024, atoi(argv[6]) * 1024);
        break;
    }

    // Close the device.
    ch_close(handle);
    return 0;
}
