/*=========================================================================
| (c) 2005-2007  Total Phase, Inc.
|--------------------------------------------------------------------------
| Project : Cheetah Sample Code
| File    : eeprom.c
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

//=========================================================================
// INCLUDES
//=========================================================================
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "cheetah.h"


//=========================================================================
// CONSTANTS
//=========================================================================
#define PAGE_SIZE              32  // Should be a power of 2 for code to work
#define PAGE_WRITE_BATCH_SIZE  32
#define PAGE_WRITE_DELAY_NS    5000000


//=========================================================================
// FUNCTIONS
//=========================================================================
static void _writeMemory (Cheetah handle, u16 addr, u16 length, int zero)
{
    u16 i, j, n, batch, count;

    u08 data_out[3+PAGE_SIZE];
    
    // Write to the SPI EEPROM
    // 
    // The AT25080A EEPROM has 32 byte pages.  Data can written
    // in pages, to reduce the number of overall SPI transactions
    // executed through the Cheetah adapter.
    n = 0;
    while (n < length) {
        ch_spi_queue_clear(handle);
        ch_spi_queue_oe(handle, 1);

        // Send PAGE_WRITE_BATCH_SIZE number of pages to the Cheetah per 
        // batch shift
        for (i = 0; i < PAGE_WRITE_BATCH_SIZE; i++) {
            if (n >= length)
                break;

            // Send write enable command
            ch_spi_queue_ss(handle, 0x1);
            ch_spi_queue_byte(handle, 1, 0x06);
            ch_spi_queue_ss(handle, 0);

            // Assemble the write command and address
            printf("addr = 0x%04x; ", addr);
            data_out[0] = 0x02;
            data_out[1] = (addr >> 8) & 0xff;
            data_out[2] = (addr >> 0) & 0xff;

            // Assemble the data
            j = 3;
            do {
                data_out[j++] = zero ? 0 : (u08) n;
                ++addr; ++n;
            } while ( (n < length) && (addr & (PAGE_SIZE-1)) );

            printf("num bytes = %d\n", j-3);
            fflush(stdout);

            // Queue the write transaction
            ch_spi_queue_ss(handle, 0x1);
            ch_spi_queue_array(handle, j, data_out);
            ch_spi_queue_ss(handle, 0);

            // Put in a wait for the write cycle time
            ch_spi_queue_delay_ns(handle, PAGE_WRITE_DELAY_NS);
        }

        // Shift the page writes
        // Don't need the results back from the shift
        printf("Shifting data\n");
        fflush(stdout);
        batch = ch_spi_batch_length(handle);
        count = ch_spi_batch_shift(handle, 0, 0);
        if (count != batch) {
            printf("Expected %d bytes but only received %d bytes\n",
                   batch, count);
            return;
        }
        printf("Shift complete\n");
        fflush(stdout);
    }
}

static void _readMemory (Cheetah handle, u16 addr, u16 length)
{
    int count;
    int i;

    u08 *data_in = (u08 *)malloc(length+3);
    memset(data_in, 0, length+3);

    ch_spi_queue_clear(handle);
    ch_spi_queue_oe(handle, 1);

    // Queue the read command, address, and data
    ch_spi_queue_ss(handle, 0x1);
    ch_spi_queue_byte(handle, 1, 0x03);
    ch_spi_queue_byte(handle, 1, (u08)((addr >> 8) & 0xff));
    ch_spi_queue_byte(handle, 1, (u08)((addr >> 0) & 0xff));
    ch_spi_queue_byte(handle, length, 0x00);
    ch_spi_queue_ss(handle, 0);

    count = ch_spi_batch_shift(handle, length+3, data_in);

    if (count < 0) {
        printf("error: %s\n", ch_status_string(count));
    }
    else if (count != length+3) {
        printf("error: read %d bytes (expected %d)\n", count-3, length);
    }

    // Dump the data to the screen
    printf("\nData read from device:");
    for (i=0; i < length; ++i) {
        if ((i&0x0f) == 0)      printf("\n%04x:  ", addr+i);
        printf("%02x ", data_in[i+3] & 0xff);
        if (((i+1)&0x07) == 0)  printf(" ");
    }
    printf("\n");
    fflush(stdout);

    // Free the packet
    free(data_in);
}


//=========================================================================
// MAIN PROGRAM
//=========================================================================
int main (int argc, char *argv[]) {
    Cheetah handle;
    int port    = 0;
    int bitrate = 0;
    int mode    = 0;
    u16 addr;
    u16 length;

    const char *command;

    if (argc < 7) {
        printf("usage: eeprom PORT BITRATE MODE read  ADDR LENGTH\n");
        printf("usage: eeprom PORT BITRATE MODE write ADDR LENGTH\n");
        printf("usage: eeprom PORT BITRATE MODE zero  ADDR LENGTH\n");
        printf("\n");
        printf("MODE possibilities are:\n");
        printf("  mode 0 : pol = 0, phase = 0\n");
        printf("  mode 1 : pol = 0, phase = 1\n");
        printf("  mode 2 : pol = 1, phase = 0\n");
        printf("  mode 3 : pol = 1, phase = 1\n");
        return 1;
    }

    port    = atoi(argv[1]);
    bitrate = atoi(argv[2]);
    mode    = atoi(argv[3]);
    command = argv[4];
    addr    = (unsigned short)strtol(argv[5], 0, 0);
    length  = atoi(argv[6]);
    
    // Open the device
    handle = ch_open(port);
    if (handle <= 0) {
        printf("Unable to open Cheetah device on port %d\n", port);
        printf("Error code = %d (%s)\n", handle, ch_status_string(handle));
        return 1;
    }

    printf("Opened Cheetah device on port %d\n", port);

    printf("Host interface is %s\n",
           (ch_host_ifce_speed(handle)) ? "high speed" : "full speed");

    // Ensure that the SPI subsystem is configured
    ch_spi_configure(handle, (mode >> 1), mode & 1, CH_SPI_BITORDER_MSB, 0x0);
    printf("SPI configuration set to mode %d, MSB shift, SS[2:0] active low\n",
           mode);
    fflush(stdout);

    // Power the EEPROM using the Cheetah adapter's power supply
    ch_target_power(handle, CH_TARGET_POWER_ON);
    ch_sleep_ms(100);

    // Set the bitrate.
    bitrate = ch_spi_bitrate(handle, bitrate);
    printf("Bitrate set to %d kHz\n", bitrate);
    fflush(stdout);

    // Shift a dummy byte to clear the EEPROM state
    ch_spi_queue_clear(handle);
    ch_spi_queue_oe(handle, 1);
    ch_spi_queue_ss(handle, 0x1);
    ch_spi_queue_byte(handle, 1, 0x00);
    ch_spi_queue_ss(handle, 0);
    ch_spi_batch_shift(handle, 0, 0);

    // Perform the requested operation
    if (strcmp(command, "write") == 0) {
        _writeMemory(handle, addr, length, 0);
        printf("Wrote to EEPROM\n");
    }
    else if (strcmp(command, "read") == 0) {
        _readMemory(handle, addr, length);
    }
    else if (strcmp(command, "zero") == 0) {
        _writeMemory(handle, addr, length, 1);
        printf("Zeroed EEPROM\n");
    }
    else {
        printf("unknown command: %s\n", command);
    }

    // Close and exit
    ch_close(handle);
    return 0;
}
