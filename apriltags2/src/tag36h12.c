/* Copyright (C) 2013-2016, The Regents of The University of Michigan.
All rights reserved.

This software was developed in the APRIL Robotics Lab under the
direction of Edwin Olson, ebolson@umich.edu. This software may be
available under alternative licensing terms; contact the address above.

An unlimited license is granted to use, adapt, modify, or embed the 2D
barcodes into any medium.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the Regents of The University of Michigan.
*/

#include <stdlib.h>
#include "apriltag.h"

apriltag_family_t *tag36h12_create()
{
   apriltag_family_t *tf = calloc(1, sizeof(apriltag_family_t));
   tf->name = strdup("tag36h12");
   tf->black_border = 1;
   tf->d = 6;
   tf->h = 12;
   tf->ncodes =79 ;
   tf->codes = calloc(79, sizeof(uint64_t));
   tf->codes[0] = 0x0000000d5d628584UL;
   tf->codes[1] = 0x00000006001134e5UL;
   tf->codes[2] = 0x00000001206fbe72UL;
   tf->codes[3] = 0x0000000ff8ad6cb4UL;
   tf->codes[4] = 0x000000085da9bc49UL; 
   tf->codes[5] = 0x0000000b461afe9cUL;
   tf->codes[6] = 0x00000006db51fe13UL;
   tf->codes[7] = 0x00000005248c541fUL;
   tf->codes[8] = 0x0000000008f34503UL;
   tf->codes[9] = 0x00000008ea462eceUL;
   tf->codes[10] = 0x0000000eac2be76dUL;
   tf->codes[11] = 0x00000001af615c44UL;
   tf->codes[12] = 0x0000000b48a49f27UL;
   tf->codes[13] = 0x00000002e4e1283bUL;
   tf->codes[14] = 0x000000078b1f2fa8UL;
   tf->codes[15] = 0x000000027d34f57eUL;
   tf->codes[16] = 0x000000089222fff1UL;
   tf->codes[17] = 0x00000004c1669406UL;
   tf->codes[18] = 0x0000000bf49b3511UL;
   tf->codes[19] = 0x0000000dc191cd5dUL;
   tf->codes[20] = 0x000000011d7c3f85UL;
   tf->codes[21] = 0x000000016a130e35UL;
   tf->codes[22] = 0x0000000e29f27effUL;
   tf->codes[23] = 0x0000000428d8ae0cUL;
   tf->codes[24] = 0x000000090d548477UL;
   tf->codes[25] = 0x00000002319cbc93UL;
   tf->codes[26] = 0x0000000c3b0c3dfcUL;
   tf->codes[27] = 0x00000000424bccc9UL;
   tf->codes[28] = 0x00000002a081d630UL;
   tf->codes[29] = 0x0000000762743d96UL;
   tf->codes[30] = 0x0000000d0645bf19UL;
   tf->codes[31] = 0x0000000f38d7fd60UL;
   tf->codes[32] = 0x0000000c6cbf9a10UL;
   tf->codes[33] = 0x00000003c1be7c65UL;
   tf->codes[34] = 0x0000000276f75e63UL;
   tf->codes[35] = 0x00000004490a3f63UL;
   tf->codes[36] = 0x0000000da60acd52UL;
   tf->codes[37] = 0x00000003cc68df59UL;
   tf->codes[38] = 0x0000000ab46f9daeUL;
   tf->codes[39] = 0x000000088d533d78UL;
   tf->codes[40] = 0x0000000b6d62ec21UL;
   tf->codes[41] = 0x0000000b3c02b646UL;
   tf->codes[42] = 0x000000022e56d408UL;
   tf->codes[43] = 0x0000000891cbcf34UL;
   tf->codes[44] = 0x000000025dd82410UL;
   tf->codes[45] = 0x0000000239551d34UL;
   tf->codes[46] = 0x00000008fe8f0c70UL;
   tf->codes[47] = 0x000000094106a970UL;
   tf->codes[48] = 0x000000082609b40cUL;
   tf->codes[49] = 0x00000000fc9caf36UL;
   tf->codes[50] = 0x0000000688181d11UL;
   tf->codes[51] = 0x0000000718613c08UL;
   tf->codes[52] = 0x00000000f1ab7629UL;
   tf->codes[53] = 0x0000000a357bfc18UL;
   tf->codes[54] = 0x00000004c03b7a46UL;
   tf->codes[55] = 0x0000000204dedce6UL;
   tf->codes[56] = 0x0000000ad6300d37UL;
   tf->codes[57] = 0x000000084cc4cd09UL;
   tf->codes[58] = 0x000000042160e5c4UL;
   tf->codes[59] = 0x000000087d2adfa8UL;
   tf->codes[60] = 0x00000007850e7749UL;
   tf->codes[61] = 0x00000004e750fc7cUL;
   tf->codes[62] = 0x0000000bf2e5dfdaUL;
   tf->codes[63] = 0x0000000d88324da5UL;
   tf->codes[64] = 0x0000000234b52f80UL;
   tf->codes[65] = 0x0000000378204514UL;
   tf->codes[66] = 0x0000000abdf2ad53UL;
   tf->codes[67] = 0x0000000365e78ef9UL;
   tf->codes[68] = 0x000000049caa6cb2UL;
   tf->codes[69] = 0x000000003c39ddf3UL;
   tf->codes[70] = 0x0000000c68c5385dUL;
   tf->codes[71] = 0x00000005bfcbbf67UL;
   tf->codes[72] = 0x0000000623241e21UL;
   tf->codes[73] = 0x0000000abc90d5ccUL;
   tf->codes[74] = 0x0000000388c6fe85UL;
   tf->codes[75] = 0x0000000da0e2d62dUL;
   tf->codes[76] = 0x000000010855dfe9UL;
   tf->codes[77] = 0x00000004d46efd6bUL;
   tf->codes[78] = 0x0000000d700476a2UL;






   return tf;
}

void tag36h12_destroy(apriltag_family_t *tf)
{
   free(tf->name);
   free(tf->codes);
   free(tf);
}
