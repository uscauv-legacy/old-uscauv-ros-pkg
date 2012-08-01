/*=========================================================================
| (C) 2007-2008  Total Phase, Inc.
|--------------------------------------------------------------------------
| Project : Cheetah Sample Code
| File    : detect.cs
|--------------------------------------------------------------------------
| Simple Cheetah Device Detection Example
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
public class detect {


    /*=====================================================================
    | GENERIC DETECTION ROUTINE
     ====================================================================*/
    static void find_devices () {
        ushort[] ports      = new ushort[16];
        uint[]   unique_ids = new uint[16];
        int     nelem       = 16;

        // Find all the attached devices
        int count = CheetahApi.ch_find_devices_ext(nelem,
                                                   ports,
                                                   nelem,
                                                   unique_ids);
        int i;

        Console.Write("{0:d} device(s) found:\n", count);

        // Print the information on each device
        if (count > nelem)  count = nelem;
        for (i = 0; i < count; ++i) {
            // Determine if the device is in-use
            String status = "(avail) ";
            if ((ports[i] & CheetahApi.CH_PORT_NOT_FREE) != 0) {
                ports[i] &= unchecked((ushort)~CheetahApi.CH_PORT_NOT_FREE);
                status = "(in-use)";
            }

            // Display device port number, in-use status, and serial number
            Console.Write("    port={0,-3:d} {1:s} ({2:d4}-{3:d6})\n",
                   ports[i], status,
                   unique_ids[i]/1000000,
                   unique_ids[i]%1000000);
        }
    }


   /*======================================================================
   | MAIN PROGRAM ENTRY POINT
    =====================================================================*/
   public static void Main (String[] args) {
       Console.Write("Searching for Cheetah adapters...\n");
       find_devices();
       Console.Write("\n\n");
       return;
   }
}
