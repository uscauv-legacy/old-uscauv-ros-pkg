/*******************************************************************************
 *
 *      buoy_identifier
 * 
 *      Copyright (c) 2011, Dhruv Monga
 *      All rights reserved.
 *
 *      Redistribution and use in source and binary forms, with or without
 *      modification, are permitted provided that the following conditions are
 *      met:
 *      
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above
 *        copyright notice, this list of conditions and the following disclaimer
 *        in the documentation and/or other materials provided with the
 *        distribution.
 *      * Neither the name of "seabee3-ros-pkg" nor the names of its
 *        contributors may be used to endorse or promote products derived from
 *        this software without specific prior written permission.
 *      
 *      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *      "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *      LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *      A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *      OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *      SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *      LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *      DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *      THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *      (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *      OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************/

#include <sensor_msgs/Image.h>
#include <image_matcher/MatchImage.h>
#include <yaml-cpp/yaml.h>
#include <base_image_proc/base_image_proc.h>
#include <fstream>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>

std::string rlimg;
std::string fkimg;

void import()
{
	rlimg = std::string( "../rlbuoy.png" );
	fkimg = std::string( "../fkbuoy.png" );

	load();
}

void load()
{
	IplImage* rlimg_;
	IplImage* fkimg_;

	rlimg_ = cvLoadImage( rlimg.c_str(), 1 );
	fkimg_ = cvLoadImage( fkimg.c_str(), 1 );

}

void identifier()
{
	int t, r, pxmatch, negx, negy, ttpxl, rand_accept, imgcenter_x = 64, imgcenter_y = 62, tmpimgcenter_x, tmpimgcenter_y, dummy_rand;
	float prob = 100;
	bool accept;
	srand( time( NULL ) );

	ttlpxl = 128 * 123;

	for ( t = 0; t < 200; t++ )
	{
		negx = rand() % 10;
		negy = rand() % 10;


		//move template image
		if ( prob >= 2 )
		{
			dummy_rand = rand() * ( prob - prob % 1 );
			if ( negx > 5 )
			{
				tmpimgcenter_x = imgcenter_x + dummy_rand;
			}
			else if ( imgcenter_x - dummy_rand >= 64 )
			{
				tmpimgcenter_x = imgcenter_x - dummy_rand;
			}
		}
		else
		{
			dummy_rand = rand() % 10;
			if ( negx > 5 )
			{
				tmpimgcenter_x = imgcenter_x + dummy_rand;
			}
			else if ( imgcenter_x - dummy_rand >= 64 )
			{
				tmpimgcenter_x = imgcenter_x - dummy_rand;
			}
		}

		if ( prob >= 2 )
		{
			dummy_rand = rand() * ( prob - prob % 1 );
			if ( negy > 5 )
			{
				tmpimgcenter_y = imgcenter_y + dummy_rand;
			}
			else if ( imgcenter_y - dummy_rand >= 62 )
			{
				tmpimgcenter_y = imgcenter_y - dummy_rand;
			}
		}
		else
		{
			dummy_rand = rand() % 10;
			if ( negy > 5 )
			{
				tmpimgcenter_y = imgcenter_y + dummy_rand;
			}
			else if ( imgcenter_y - dummy_rand >= 62 )
			{
				tmpimgcenter_y = imgcenter_y - dummy_rand;
			}
		}

		if ( pxmatch( tmpimgcenter_x, tmpimgcenter_y ) / ttlpixel > pxmatch( imgcenter_x, imgcenter_y ) / ttlpixel )
		{
			accept = true;
		}
		else
		{
			if ( rand() % 100 < prob )
			{
				accept = true;
			}
		}

		if ( accept = true )
		{
			imgcenter_x = tmpimgcenter_x;
			imgcenter_y = tmpimgcenter_y;
		}

		prob = prob - 0.5;

	}

	if ( pxmatch( imgcenter_x, imgcenter_y ) / ttlpixel > 0.9 )
	{
		printf( " Found a buoy at coordinates x: %i and y: %i", imgcenter_x, imgcenter_y );
	}
	else
	{
		printf( " No buoys found" );
	}
}

/*
 int measuresize()
 {

 }
 */

int pxmatch( int image_center_x, int image_center_y )
{
	int x, y, match = 0, startx = image_center_x - 64, starty = image_center_y - 62;

	for ( x = startx; x < 128; x++ )
	{
		for ( y = starty; y < 123; y++ )
		{
			if ( cvGet2D( rlimg_, x, y ) == cvGet2D( fkimg_, x - start, y - start ) )
			{
				match++;
			}
		}
	}
	return ( match );
}

int main( int argc, char** argv )
{
	import();
}

