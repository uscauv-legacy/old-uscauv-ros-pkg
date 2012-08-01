/*=========================================================================
| (c) 2005-2007  Total Phase, Inc.
|--------------------------------------------------------------------------
| Project : Cheetah Sample Code
| File    : timing.c
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

//=========================================================================
// INCLUDES
//=========================================================================
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "cheetah.h"


//=========================================================================
// FUNCTIONS
//=========================================================================
static void _timing (Cheetah handle)
{
    int cycles;
    int ns;
    u08 data_in[6];

    // Test the SS timing
    ch_spi_queue_clear(handle);
    printf("Testing inter-SS delays...\n");
    fflush(stdout);
    ch_spi_queue_oe(handle, 1);
    ch_spi_queue_ss(handle, 0x1);

    cycles = ch_spi_queue_delay_cycles(handle, 51);
    printf("  Queued delay of %d cycles within first SS assert/deassert.\n",
           cycles);
    fflush(stdout);
    ch_spi_queue_ss(handle, 0);

    ch_spi_queue_ss(handle, 0x1);

    ns = ch_spi_queue_delay_ns(handle, 1500000);
    printf("  Queued delay of %d ns within second SS assert/deassert.\n", ns);
    fflush(stdout);
    ch_spi_queue_ss(handle, 0);
    
    ch_spi_batch_shift(handle, 0, 0);

    // Test data timing in read mode
    printf("Testing inter-data (read) delays...\n");
    fflush(stdout);
    ch_spi_queue_clear(handle);
    ch_spi_queue_ss(handle, 0x1);

    ch_spi_queue_byte(handle, 1, 0xca);
    ns = ch_spi_queue_delay_ns(handle, 250000);
    printf("  Queued delay of %d ns after first byte (0xca).\n", ns);
    fflush(stdout);

    ch_spi_queue_byte(handle, 2, 0xfe);
    cycles = ch_spi_queue_delay_cycles(handle, 995);
    printf("  Queued delay of %d cycles after second byte (0xfe).\n", cycles);
    fflush(stdout);

    ch_spi_queue_byte(handle, 3, 0x00);
    cycles = ch_spi_queue_delay_cycles(handle, 20);
    printf("  Queued delay of %d cycles after last byte (0x00).\n", cycles);
    fflush(stdout);

    ch_spi_queue_ss(handle, 0);
    ch_spi_batch_shift(handle, 6, data_in);

    // Test data timing with write mode
    printf("Testing inter-data (write) delays...\n");
    fflush(stdout);
    ch_spi_queue_clear(handle);
    ch_spi_queue_ss(handle, 0x1);

    ch_spi_queue_byte(handle, 1, 0xba);
    ns = ch_spi_queue_delay_ns(handle, 80000);
    printf("  Queued delay of %d ns after first byte (0xba).\n", ns);
    fflush(stdout);

    ch_spi_queue_byte(handle, 2, 0xbe);
    cycles = ch_spi_queue_delay_cycles(handle, 995);
    printf("  Queued delay of %d cycles after second byte (0xbe).\n", cycles);
    fflush(stdout);

    ch_spi_queue_byte(handle, 3, 0x00);
    cycles = ch_spi_queue_delay_cycles(handle, 20);
    printf("  Queued delay of %d cycles after last byte (0x00).\n", cycles);
    fflush(stdout);

    ch_spi_queue_ss(handle, 0);
    ch_spi_queue_oe(handle, 0);

    ch_spi_batch_shift(handle, 1, data_in);
}


//=========================================================================
// MAIN PROGRAM
//=========================================================================
int main (int argc, char *argv[]) {
    Cheetah handle;
    int port    = 0;
    int bitrate = 0;
    int mode    = 0;

    if (argc < 4) {
        printf("usage: timing PORT BITRATE MODE\n");
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
    fflush(stdout);

    // Ensure that the SPI subsystem is configured
    ch_spi_configure(handle, (mode >> 1), mode & 1, CH_SPI_BITORDER_MSB, 0x0);
    printf("SPI configuration set to mode %d, MSB shift, SS[2:0] active low\n",
           mode);
    fflush(stdout);

    // Power the target using the Cheetah adapter's power supply
    ch_target_power(handle, CH_TARGET_POWER_ON);
    ch_sleep_ms(100);

    // Set the bitrate
    bitrate = ch_spi_bitrate(handle, bitrate);
    printf("Bitrate set to %d kHz\n", bitrate);
    fflush(stdout);

    _timing(handle);

    // Close and exit
    ch_close(handle);
    return 0;
}
