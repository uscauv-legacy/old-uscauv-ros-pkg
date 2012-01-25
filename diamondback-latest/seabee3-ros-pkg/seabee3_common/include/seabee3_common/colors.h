/***************************************************************************
 *  include/seabee3_common/colors.h
 *  --------------------
 *
 *  Copyright (c) 2011, Edward T. Kaszubski ( ekaszubski@gmail.com )
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are
 *  met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer
 *    in the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of seabee3-ros-pkg nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **************************************************************************/

#ifndef SEABEE3COMMON_COLORS_H_
#define SEABEE3COMMON_COLORS_H_

#include <quickdev/pixel.h>
#include <vector>

namespace seabee3_common
{
namespace colors
{
    typedef quickdev::Pixel<uchar> _Pixel;
namespace values
{
    static const _Pixel RED    ( (uchar)255, (uchar)0  , (uchar)0   );
    static const _Pixel ORANGE ( (uchar)255, (uchar)127, (uchar)0   );
    static const _Pixel YELLOW ( (uchar)255, (uchar)255, (uchar)0   );
    static const _Pixel GREEN  ( (uchar)0  , (uchar)255, (uchar)0   );
    static const _Pixel BLUE   ( (uchar)0  , (uchar)0  , (uchar)255 );
    static const _Pixel BLACK  ( (uchar)0  , (uchar)0  , (uchar)0   );
    static const _Pixel WHITE  ( (uchar)255, (uchar)255, (uchar)255 );
} // values

namespace names
{
    static const size_t RED =    0;
    static const size_t ORANGE = 1;
    static const size_t YELLOW = 2;
    static const size_t GREEN =  3;
    static const size_t BLUE =   4;
    static const size_t BLACK =  5;
    static const size_t WHITE =  6;
} // names

    static const std::vector<_Pixel> PIXELS = {
        values::RED,
        values::ORANGE,
        values::YELLOW,
        values::GREEN,
        values::BLUE,
        values::BLACK,
        values::WHITE };

} // colors
} // seabee3_common

#endif // SEABEE3COMMON_COLORS_H_
