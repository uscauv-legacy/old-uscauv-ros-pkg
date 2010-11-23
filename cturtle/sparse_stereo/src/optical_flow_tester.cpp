/*******************************************************************************
 *
 *      optical_flow_tester
 * 
 *      Copyright (c) 2010, edward
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

#include <base_image_proc/base_image_proc.h>
#include <cv_bridge/CvBridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>

class OpticalFlowTester: public BaseNode<>
{
private:
	boost::mutex l_img_mutex_, r_img_mutex_, flag_mutex_;

	sensor_msgs::ImagePtr left_image_msg_, right_image_msg_;

	ros::Subscriber info_l_sub_, info_r_sub_;
	image_transport::Subscriber image_l_sub_, image_r_sub_;
	image_transport::Publisher disparity_pub_;

	sensor_msgs::CvBridge bridge_;
	image_transport::ImageTransport it_;

	IplImage *right_image_, *left_image_;
	IplImage * last_image_;
	const static int MAX_CORNERS = 100;

	bool new_left_img_, new_right_img_, processed_left_img_, processed_right_img_;

	std::string stereo_ns, image, left_ns, right_ns;

public:
	OpticalFlowTester( ros::NodeHandle & nh ) :
		BaseNode<> ( nh ), it_( nh_priv_ ), new_left_img_( false ), new_right_img_( false ), processed_left_img_( false ),
		        processed_right_img_( false ), right_image_( NULL ), left_image_( NULL ), last_image_( NULL )
	{
		nh_priv_.param( "stereo",
		                stereo_ns,
		                std::string( "/stereo" ) );
		nh_priv_.param( "image",
		                image,
		                std::string( "image_rect_color" ) );
		nh_priv_.param( "left",
		                left_ns,
		                std::string( "left" ) );
		nh_priv_.param( "right",
		                right_ns,
		                std::string( "right" ) );

		image_l_sub_ = it_.subscribe( stereo_ns + "/" + left_ns + "/" + image,
		                              1,
		                              &OpticalFlowTester::leftImageCB,
		                              this );
		image_r_sub_ = it_.subscribe( stereo_ns + "/" + right_ns + "/" + image,
		                              1,
		                              &OpticalFlowTester::rightImageCB,
		                              this );
		disparity_pub_ = it_.advertise( stereo_ns + "/disparity",
		                                1 );
	}

	IplImage * processImages( const IplImage * ipl_img,
	                          const IplImage * last_image )
	{
		// Initialize, load two images from the file system, and
		// allocate the images and other structures we will need for
		// results.
		//

		IplImage* imgA = cvCreateImage( cvSize( ipl_img->width,
		                                        ipl_img->height ),
		                                IPL_DEPTH_8U,
		                                1 );


		//IplImage* imgB = last_image;
		IplImage* imgB = cvCreateImage( cvSize( ipl_img->width,
		                                        ipl_img->height ),
		                                IPL_DEPTH_8U,
		                                1 );
		cvCopy( last_image,
		        imgB );
		/*cvCvtColor( last_image,
		 imgB,
		 CV_BGR2GRAY );*/

		IplImage* output_image = cvCreateImage( cvSize( ipl_img->width,
		                                                ipl_img->height ),
		                                        IPL_DEPTH_8U,
		                                        3 );
		cvCopy( ipl_img,
		        output_image );

		if ( imgB )
		{
			cvCvtColor( ipl_img,
			            imgA,
			            CV_BGR2GRAY );

			CvSize img_sz = cvGetSize( imgA );
			int win_size = 10;
			// The first thing we need to do is get the features
			// we want to track.
			//
			IplImage* eig_image = cvCreateImage( img_sz,
			                                     IPL_DEPTH_32F,
			                                     1 );
			IplImage* tmp_image = cvCreateImage( img_sz,
			                                     IPL_DEPTH_32F,
			                                     1 );
			int corner_count = MAX_CORNERS;
			CvPoint2D32f* cornersA = new CvPoint2D32f[MAX_CORNERS];
			cvGoodFeaturesToTrack( imgA,
			                       eig_image,
			                       tmp_image,
			                       cornersA,
			                       &corner_count,
			                       0.01,
			                       5.0,
			                       0,
			                       3,
			                       0,
			                       0.04 );


			/*cvFindCornerSubPix( imgA,
			 cornersA,
			 corner_count,
			 cvSize( win_size,
			 win_size ),
			 cvSize( -1,
			 -1 ),
			 cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS,
			 20,
			 0.03 ) );*/
			// Call the Lucas Kanade algorithm
			//
			char features_found[MAX_CORNERS];
			float feature_errors[MAX_CORNERS];
			CvSize pyr_sz = cvSize( imgA->width + 8,
			                        imgB->height / 3 );
			IplImage* pyrA = cvCreateImage( pyr_sz,
			                                IPL_DEPTH_32F,
			                                1 );
			IplImage* pyrB = cvCreateImage( pyr_sz,
			                                IPL_DEPTH_32F,
			                                1 );
			CvPoint2D32f* cornersB = new CvPoint2D32f[MAX_CORNERS];
			cvCalcOpticalFlowPyrLK( imgA,
			                        imgB,
			                        pyrA,
			                        pyrB,
			                        cornersA,
			                        cornersB,
			                        corner_count,
			                        cvSize( imgA->width,
			                                win_size ),
			                        5,
			                        features_found,
			                        feature_errors,
			                        cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS,
			                                        20,
			                                        .3 ),
			                        0 );
			// Now make some image of what we are looking at:
			//
			for ( int i = 0; i < corner_count; i++ )
			{
				if ( features_found[i] == 0 || feature_errors[i] > 550 )
				{
					//printf( "Error is %f\n",
					//        feature_errors[i] );
					continue;
				}

				//printf( "got it/n" );
				printf( "%f %f, %f %f\n",
				        cornersA[i].x,
				        cornersA[i].y,
				        cornersB[i].x,
				        cornersB[i].y );
				CvPoint p0 = cvPoint( cvRound( cornersA[i].x ),
				                      cvRound( cornersA[i].y ) );
				CvPoint p1 = cvPoint( cvRound( cornersB[i].x ),
				                      cvRound( cornersB[i].y ) );
				cvLine( output_image,
				        p0,
				        p1,
				        CV_RGB(255,0,0),
				        2 );
			}

			cvReleaseImage( &pyrA );
			cvReleaseImage( &pyrB );
			cvReleaseImage( &eig_image );
			cvReleaseImage( &tmp_image );
			cvReleaseImage( &imgB );
		}

		if ( last_image_ ) cvReleaseImage( &last_image_ );
		last_image_ = imgA;

		return output_image;
	}

	void processImages()
	{
		//ROS_DEBUG( "Trying to process available resources..." );

		if ( new_left_img_ && !processed_left_img_ )
		{
			//ROS_DEBUG( "processing left image..." );
			boost::lock_guard<boost::mutex> limgguard( l_img_mutex_ );
			// if( left_image_ ) cvReleaseImage( &left_image_ );
			IplImage * left_image_tmp = bridge_.imgMsgToCv( left_image_msg_ );
			left_image_ = cvCreateImage( cvSize( left_image_tmp->width,
			                                     left_image_tmp->height ),
			                             IPL_DEPTH_8U,
			                             3 );
			cvCopy( left_image_tmp,
			        left_image_ );

			processed_left_img_ = true;
		}

		if ( new_right_img_ && !processed_right_img_ )
		{
			//ROS_DEBUG( "processing right image..." );
			boost::lock_guard<boost::mutex> rimgguard( r_img_mutex_ );
			// if( right_image_ ) cvReleaseImage( &right_image_ );
			IplImage * right_image_tmp = bridge_.imgMsgToCv( right_image_msg_ );
			right_image_ = cvCreateImage( cvSize( right_image_tmp->width,
			                                      right_image_tmp->height ),
			                              IPL_DEPTH_8U,
			                              3 );
			cvCopy( right_image_tmp,
			        right_image_ );

			processed_right_img_ = true;
		}

		if ( new_left_img_ && new_right_img_ && processed_left_img_ && processed_right_img_ )
		{
			//ROS_DEBUG( "Checking locks..." );
			boost::lock_guard<boost::mutex> limgguard( l_img_mutex_ );
			boost::lock_guard<boost::mutex> rimgguard( r_img_mutex_ );

			IplImage * right_image_bw = cvCreateImage( cvSize( right_image_->width,
			                                                   right_image_->height ),
			                                           IPL_DEPTH_8U,
			                                           1 );

			cvCvtColor( right_image_,
			            right_image_bw,
			            CV_BGR2GRAY );

			IplImage * output = processImages( left_image_,
			                                   right_image_bw );

			disparity_pub_.publish( bridge_.cvToImgMsg( output ) );

			cvReleaseImage( &right_image_bw );
			cvReleaseImage( &left_image_ );
			cvReleaseImage( &right_image_ );

			new_left_img_ = false;
			new_right_img_ = false;
			processed_left_img_ = false;
			processed_right_img_ = false;
		}
	}

	void leftImageCB( const sensor_msgs::ImageConstPtr& msg )
	{
		ROS_DEBUG( "got left img" );
		l_img_mutex_.lock();
		left_image_msg_ = boost::const_pointer_cast<sensor_msgs::Image>( msg );
		//img_left = msg;

		l_img_mutex_.unlock();

		boost::lock_guard<boost::mutex> guard( flag_mutex_ );
		new_left_img_ = true;

		processImages();
	}

	void rightImageCB( const sensor_msgs::ImageConstPtr& msg )
	{
		ROS_DEBUG( "got right img" );
		r_img_mutex_.lock();
		right_image_msg_ = boost::const_pointer_cast<sensor_msgs::Image>( msg );
		//img_right = msg;

		r_img_mutex_.unlock();

		boost::lock_guard<boost::mutex> guard( flag_mutex_ );
		new_right_img_ = true;

		processImages();
	}

};

int main( int argc,
          char ** argv )
{
	ros::init( argc,
	           argv,
	           "optical_flow_tester" );
	ros::NodeHandle nh;

	OpticalFlowTester optical_flow_tester( nh );
	optical_flow_tester.spin();

	return 0;
}
