/*******************************************************************************
 *
 *      test_findcontour
 * 
 *      Copyright (c) 2011, noah
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
#include <stdio.h>
#include <opencv/cv.h>
#include "highgui.h"
#include <opencv/cxcore.h>
#include <common_utils/time.h>
int main( int argc, char* argv[] )
{
	double tik = time_utils::getTimeInSecs();
	CvScalar red = { 255, 0, 0 };
	CvScalar blue = { 0, 0, 255 };
	int mass;
	int mass_c;
	int mass_h;
	CvSeq* h;
	cvNamedWindow( argv[0], 1 );
	IplImage* img_8uc1 = cvLoadImage( argv[1], CV_LOAD_IMAGE_GRAYSCALE );
	IplImage* img_edge = cvCreateImage( cvGetSize( img_8uc1 ), 8, 1 );
	IplImage* img_8uc3 = cvCreateImage( cvGetSize( img_8uc1 ), 8, 3 );
	cvThreshold( img_8uc1, img_edge, 128, 255, CV_THRESH_BINARY );
	CvMemStorage* storage = cvCreateMemStorage();
	CvSeq* first_contour = NULL;
	int Nc = cvFindContours( img_edge, storage, &first_contour, sizeof(CvContour), CV_RETR_CCOMP );
	// Try all four values and see what happens

	int n = 0;
	printf( "Total Contours Detected: %d\n", Nc );
	for ( CvSeq* c = first_contour; c != NULL; c = c->h_next )
	{
		mass = 0;
		mass_c = 0;
		mass_h = 0;
		cvCvtColor( img_8uc1, img_8uc3, CV_GRAY2BGR );
		cvDrawContours( img_8uc3, c, red, blue, 0, 2, 8 );
		// Try different values of max_level, and see what happens
		printf( "Contour #%d\n", n );
		cvShowImage( argv[0], img_8uc3 );
		printf( " %d elements:\n", c->total );
		for ( int i = 0; i < c->total; ++i )
		{
			CvPoint* p = CV_GET_SEQ_ELEM( CvPoint, c, i );
			printf( "(%d,%d)\n", p->x, p->y );
			//printf("i = %d\n", i);
			/*Iterates through the values of the contour until it finds
			another index with same x value. if x > new x, it adds that value
			to the mass. This effectively numerically integrates the blob.*/
			for ( int j = i + 1; j != i; j = ( j + 1 ) % ((c->total)) )
			{
				//printf("j = %d\n", j);
				CvPoint* q = CV_GET_SEQ_ELEM( CvPoint, c, j );
				if ( q->x == p->x && p->y > q->y)
				{
					mass_c += ( p->y - q->y );
				}

			}
		}
		if ( c->v_next )
		{
			h = c->v_next;
			for ( int i = 0; i < h->total; ++i )
			{
				//printf( "i = %d, h_total = %d\n", i, h->total);
				CvPoint* p = CV_GET_SEQ_ELEM( CvPoint, h, i );
				for ( int j = i + 1; j != i; j = ( j + 1 ) % ((h->total)) )
				{
					//printf( "(i,j) = (%d,%d)\n", i, j );
					CvPoint* q = CV_GET_SEQ_ELEM( CvPoint, h, j );
					if ( q->x == p->x && p->y > q->y )
					{
						mass_h += ( p->y - q->y );
					}
				}
			}
			mass = mass_c - mass_h;
		}
		printf( "Mass of contour is %d\n", mass_c );
		printf( "Mass of hole is %d\n", mass_h );
		printf( "Total mass of blob %d is %d\n", n, mass );
		cvWaitKey( 0 );
		n++;
	}
	printf( "Finished all contours.\n" );
	cvCvtColor( img_8uc1, img_8uc3, CV_GRAY2BGR );
	cvShowImage( argv[0], img_8uc3 );
	cvWaitKey( 0 );
	cvDestroyWindow( argv[0] );
	cvReleaseImage( &img_8uc1 );
	cvReleaseImage( &img_8uc3 );
	cvReleaseImage( &img_edge );
	double tok = time_utils::getTimeInSecs();
	printf("Total run time = %f",tok-tik);
	return 0;
}
