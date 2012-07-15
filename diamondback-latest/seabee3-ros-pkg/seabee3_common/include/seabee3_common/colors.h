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

#include <string>
#include <std_msgs/ColorRGBA.h>

namespace seabee
{

    typedef std_msgs::ColorRGBA _ColorRGBAMsg;

class Color
{
public:
    enum ColorType
    {
        BLACK,
        RED,
        ORANGE,
        YELLOW,
        GREEN,
        BLUE,
        WHITE
    };

protected:
    ColorType color_;

public:
    Color( ColorType color = BLACK )
    :
        color_( color )
    {
        //
    }

    Color( std::string const & color )
    {
        if( color == "red" ) color_ = RED;
        else if( color == "orange" ) color_ = ORANGE;
        else if( color == "yellow" ) color_ = YELLOW;
        else if( color == "green" ) color_ = GREEN;
        else if( color == "blue" ) color_ = BLUE;
        else if( color == "white" ) color_ = WHITE;
        else color_ = BLACK;
    }

    operator std::string() const
    {
        switch( color_ )
        {
        case BLACK:
            return "black";
        case RED:
            return "red";
        case ORANGE:
            return "orange";
        case YELLOW:
            return "yellow";
        case GREEN:
            return "green";
        case BLUE:
            return "blue";
        case WHITE:
            return "white";
        }

        return "";
    }

    operator _ColorRGBAMsg() const
    {
        _ColorRGBAMsg color_msg;
        color_msg.a = 1.0;

        switch( color_ )
        {
        case BLACK:
            break;
        case RED:
            color_msg.r = 1.0;
            break;
        case ORANGE:
            color_msg.r = 1.0;
            color_msg.g = 0.5;
            break;
        case YELLOW:
            color_msg.r = 1.0;
            color_msg.g = 1.0;
            break;
        case GREEN:
            color_msg.g = 1.0;
            break;
        case BLUE:
            color_msg.b = 1.0;
            break;
        case WHITE:
            color_msg.r = 1.0;
            color_msg.g = 1.0;
            color_msg.b = 1.0;
            break;
        }

        return color_msg;
    }
};

} // seabee

#endif // SEABEE3COMMON_COLORS_H_
