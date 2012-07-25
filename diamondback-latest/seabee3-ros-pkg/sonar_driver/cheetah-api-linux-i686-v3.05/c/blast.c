/*=========================================================================
| (c) 2005-2007  Total Phase, Inc.
|--------------------------------------------------------------------------
| Project : Cheetah Sample Code
| File    : blast.c
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
// Uncomment to show the data returned by the shifting device
#define SHOW_DATA

// Add a delay between bytes by changing this constant (in nanoseconds)
#define BYTE_DELAY 0


//=========================================================================
// UTILITY FUNCTIONS
//=========================================================================
static s64 _timeMillis () {
#ifdef _WIN32
    return ((s64)clock()) * 1000 / CLOCKS_PER_SEC;
#else
    struct timeval tv;
    gettimeofday(&tv, 0);
    return ((s64)tv.tv_sec * 1000L) + (s64)(tv.tv_usec / 1000L);
#endif
}


//=========================================================================
// FUNCTIONS
//=========================================================================
static void _blast (Cheetah handle, u32 length) {
    double elapsed;
    int delay = 0;
    u32 i;
    int batch;
    int count;
    u08 *data_in;
    s64 start = _timeMillis();

    // Queue the read sequence
    ch_spi_queue_clear(handle);

    ch_spi_queue_oe(handle, 1);
    ch_spi_queue_ss(handle, 0);
    ch_spi_queue_ss(handle, 0x1);

    for (i = 0; i < length; ++i) {
        ch_spi_queue_byte(handle, 1, (u08)(i & 0xff));

        delay = ch_spi_queue_delay_ns(handle, BYTE_DELAY);
    }
    printf("Queued delay of %d ns between bytes.\n", delay);
    fflush(stdout);

    ch_spi_queue_ss(handle, 0);
    ch_spi_queue_oe(handle, 0);

    elapsed = ((double)(_timeMillis() - start)) / 1000;
    printf("Took %.2lf seconds to queue the batch.\n", elapsed);
    fflush(stdout);

    // Perform the shift
    batch    = ch_spi_batch_length(handle);
    data_in = (u08 *)malloc(batch);

    start = _timeMillis();
    count = ch_spi_batch_shift(handle, batch, data_in);

    elapsed = ((double)(_timeMillis() - start)) / 1000;
    printf("Took %.2lf seconds to shift the batch.\n", elapsed);
    fflush(stdout);

    if (count != batch) {
        printf("Expected %d bytes but only received %d bytes\n",
               batch, count);
        goto cleanup;
    }

#ifdef SHOW_DATA
    // Output the data to the screen
    printf("\nData:");
    for (i = 0; i < length; ++i) {
        if ((i&0x07) == 0)      printf("\n%04x:  ", i);
        printf("%02x/%02x ", (i & 0xff), data_in[i]);
    }
    printf("\n");
    fflush(stdout);
#endif

cleanup:    
    // Cleanup and exit
    free(data_in);
}


//=========================================================================
// MAIN PROGRAM
//=========================================================================
int main (int argc, char *argv[]) {
    Cheetah handle;
    int port     = 0;
    int bitrate  = 0;
    int mode     = 0;
    int bitorder = 0;
    u32 length;

    if (argc < 6) {
        printf("usage: blast PORT BITRATE MODE BITORDER LENGTH\n");
        printf("\n");
        printf("MODE possibilities are:\n");
        printf("  mode 0 : pol = 0, phase = 0\n");
        printf("  mode 1 : pol = 0, phase = 1\n");
        printf("  mode 2 : pol = 1, phase = 0\n");
        printf("  mode 3 : pol = 1, phase = 1\n");
        printf("\n");
        printf("BITORDER should be 0 for MSB, 1 for LSB\n");
        return 1;
    }

    port     = atoi(argv[1]);
    bitrate  = atoi(argv[2]);
    mode     = atoi(argv[3]);
    bitorder = atoi(argv[4]);
    length   = atoi(argv[5]);
    
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

    // Ensure that the SPI subsystem is configured.
    ch_spi_configure(handle, (mode >> 1), mode & 1, !!bitorder, 0x0);
    printf("SPI configuration set to mode %d, %s shift, SS[2:0] active low\n",
           mode, (!bitorder) ? "MSB" : "LSB");
    fflush(stdout);

    // Power the target using the Cheetah adapter's power supply.
    ch_target_power(handle, CH_TARGET_POWER_ON);
    ch_sleep_ms(100);

    // Set the bitrate.
    bitrate = ch_spi_bitrate(handle, bitrate);
    printf("Bitrate set to %d kHz\n", bitrate);
    fflush(stdout);

    _blast(handle, length);
    
    // Close and exit.
    ch_close(handle);
    return 0;
}
