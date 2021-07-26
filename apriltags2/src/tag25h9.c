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

apriltag_family_t *tag25h9_create()
{
   apriltag_family_t *tf = calloc(1, sizeof(apriltag_family_t));
   tf->name = strdup("tag25h9");
   tf->black_border = 1;
   tf->d = 5;
   tf->h = 9;
   tf->ncodes = 36;
   tf->codes = calloc(36, sizeof(uint64_t));
   tf->codes[0] = 0x0000000000ca109fUL;
   tf->codes[1] = 0x0000000000e8889fUL;
   tf->codes[2] = 0x0000000000e89a2eUL;
   tf->codes[3] = 0x0000000000232be2UL;
   tf->codes[4] = 0x0000000001f8383eUL;
   tf->codes[5] = 0x0000000000f87a2eUL;
   tf->codes[6] = 0x0000000001e08888UL;
   tf->codes[7] = 0x0000000000e8ba2eUL;
   tf->codes[8] = 0x0000000000e8bc2eUL;
   tf->codes[9] = 0x0000000000ecd66eUL;
   tf->codes[10] = 0x0000000000457e31UL;
   tf->codes[11] = 0x0000000001e8fa3eUL;
   tf->codes[12] = 0x0000000000e8c22eUL;
   tf->codes[13] = 0x0000000001e8c63eUL;
   tf->codes[14] = 0x0000000001f87a1fUL;
   tf->codes[15] = 0x0000000001f87a10UL;
   tf->codes[16] = 0x0000000000e85e2eUL;
   tf->codes[17] = 0x000000000118fe31UL;
   tf->codes[18] = 0x0000000001f2109fUL;
   tf->codes[19] = 0x000000000010c62eUL;
   tf->codes[20] = 0x0000000001197251UL;
   tf->codes[21] = 0x000000000108421fUL;
   tf->codes[22] = 0x00000000011dd631UL;
   tf->codes[23] = 0x00000000011cd671UL;
   tf->codes[24] = 0x0000000000e8c62eUL;
   tf->codes[25] = 0x0000000001e8fa10UL;
   tf->codes[26] = 0x0000000000e8c64dUL;
   tf->codes[27] = 0x0000000001e8fa51UL;
   tf->codes[28] = 0x0000000000f8383eUL;
   tf->codes[29] = 0x0000000001f21084UL;
   tf->codes[30] = 0x000000000118c62eUL;
   tf->codes[31] = 0x000000000118c544UL;
   tf->codes[32] = 0x000000000118d6aaUL;
   tf->codes[33] = 0x0000000001151151UL;
   tf->codes[34] = 0x0000000001151084UL;
   tf->codes[35] = 0x0000000001f1111fUL;
   return tf;
}

void tag25h9_destroy(apriltag_family_t *tf)
{
   free(tf->name);
   free(tf->codes);
   free(tf);
}
