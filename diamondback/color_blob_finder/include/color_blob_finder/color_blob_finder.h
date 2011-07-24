/*******************************************************************************
 *
 *      color_blob_finder
 * 
 *      Copyright (c) 2011, Edward T. Kaszubski ( ekaszubski@gmail.com )
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

#ifndef COLOR_BLOB_FINDER_H_
#define COLOR_BLOB_FINDER_H_

/* stl */
#include <string>
#include <vector>

/* ros */
#include <common_utils/opencv.h>

// message filters
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
// for CvBridge
#include <cv_bridge/CvBridge.h>
#include <cv_bridge/cv_bridge.h>

/* msgs */
#include <base_libs/ComponentImageArray.h>
#include <color_blob_finder/ColorBlobArray.h>

/* srvs */
#include <color_blob_finder/FindColorBlobs.h>

/* cfgs */
#include <color_blob_finder/ColorBlobFinderConfig.h>

/* others */
#include <base_node/base_node.h>
#include <common_utils/colors.h>
#include <color_blob_finder/contour.h>
// for mutex
#include <boost/thread.hpp>

typedef color_blob_finder::ColorBlobFinderConfig _ReconfigureType;
typedef BaseNode<_ReconfigureType> _BaseNode;
typedef unsigned int _DimType;
typedef color_blob_finder::FindColorBlobs _FindColorBlobsService;

typedef base_libs::ComponentImageArray _ImageArrayMessageType;

typedef color_blob_finder::ColorBlob _ColorBlobMessageType;

typedef message_filters::sync_policies::ApproximateTime<_ImageArrayMessageType, _ImageArrayMessageType> _SyncPolicy;

class ColorBlobFinder: public _BaseNode
{
public:
	/* subs */
	message_filters::Subscriber<_ImageArrayMessageType> source1_sub_;
	message_filters::Subscriber<_ImageArrayMessageType> source2_sub_;

	/* pubs */
	ros::Publisher color_blob_array_pub_;
	image_transport::Publisher output_image_pub_;

	/* service servers */
	ros::ServiceServer find_color_blobs_svr_;

	/* others */
	message_filters::Synchronizer<_SyncPolicy> sync_;
	IplImage * output_image_;
	image_transport::ImageTransport image_transport_;
	boost::mutex image_sources_mutex_;
	sensor_msgs::CvBridge cv_bridge_;
	std::vector<base_libs::ComponentImage> images_cache_;
	bool new_image_;

	ColorBlobFinder( const ros::NodeHandle & nh ) :
			_BaseNode( nh ), source1_sub_( nh_local_,
			                               "source1",
			                               1 ),
			                 source2_sub_( nh_local_,
			                               "source2",
			                               1 ),
			                 sync_( _SyncPolicy( 10 ),
			                        source1_sub_,
			                        source2_sub_ ),
			                 output_image_( NULL ),
			                 image_transport_( nh_local_ ),
			                 new_image_( false )
	{
		sync_.registerCallback( boost::bind( &ColorBlobFinder::imageSourcesCB,
		                                     this,
		                                     _1,
		                                     _2 ) );

		output_image_pub_ = image_transport_.advertise( "output_image",
		                                                1 );
		find_color_blobs_svr_ = nh_local_.advertiseService( "find_color_blobs", &ColorBlobFinder::findColorBlobsCB, this );

		initCfgParams();
	}

	virtual ~ColorBlobFinder()
	{
		image_sources_mutex_.unlock();
		if( output_image_ ) cvReleaseImage( &output_image_ );
	}

	bool findColorBlobsCB( _FindColorBlobsService::Request & req, _FindColorBlobsService::Response & resp )
	{
		printf( "Got service request\n" );
		if( !image_sources_mutex_.try_lock() ) return false;
		if( images_cache_.size() == 0 )
		{
			image_sources_mutex_.unlock();
			return false;
		}
		//image_sources_mutex_.lock();

		//color_blob_finder::ColorBlobArray::Ptr color_blobs( new color_blob_finder::ColorBlobArray );

		const static OutputColorRGB::_CvColorType background_color_vec = OutputColorRGB::getColorRGB( -1 );
		const static CvScalar background_color = cvScalar( background_color_vec[0], background_color_vec[1], background_color_vec[2] );
		printf( "Setting image background\n" );
		if( output_image_ ) cvSet( output_image_, background_color );//cvFillImage( output_image_, 0.0 );
		printf( "Done\n" );

		// for each image
		for ( _DimType i = 0; i < images_cache_.size(); ++i )
		{
			// look at the color ID of each image; only process it if it matches one of the colors in the request
			bool enable_current_color = false;
			for( _DimType j = 0; j < req.colors.size(); ++j )
			{
				if( images_cache_[i].id == req.colors[j] )
				{
					enable_current_color = true;
					break;
				}
			}

			if( !enable_current_color ) continue;

			/*const sensor_msgs::Image::ConstPtr current_image_msg_ptr ( &images[i].image );
			const IplImage current_image = IplImage ( cv_bridge::toCvShare( current_image_msg_ptr )->image );
			const IplImage * current_image_ptr = &current_image;*/

			printf( "Building bridge from image\n" );
			cv_bridge_.fromImage( images_cache_[i].image );
			printf( "Converting to ipl\n" );
			const IplImage * current_image_ptr = cv_bridge_.toIpl();

			std::vector<_Contour> contours;
			printf( "Getting ready to process\n" );
			processImage( current_image_ptr, contours, images_cache_[i].id );

			// for each contour
			for ( _DimType j = 0; j < contours.size(); ++j )
			{
				color_blob_finder::ColorBlob color_blob;
				color_blob.color_id = images_cache_[i].id;

				color_blob_finder::Contour contour;
				contour.header = images_cache_[i].image.header;

				contour << contours[j];

				color_blob.contour = contour;
				resp.blobs.push_back( color_blob );
			}
		}

		new_image_ = false;

		image_sources_mutex_.unlock();

		printf("Attempting to publish image");

//		output_image_pub_.publish( sensor_msgs::CvBridge::cvToImgMsg( output_image_ ) );

		printf( "Done" );

		return true;
	}

	void imageSourcesCB( const _ImageArrayMessageType::ConstPtr & image_array_msg1,
	                     const _ImageArrayMessageType::ConstPtr & image_array_msg2 )
	{
		printf( "Got images\n" );
		if( !image_sources_mutex_.try_lock() )
		{
			printf( "Couldn't get lock; dropping images\n" );
			return;
		}

		unsigned int total_images = image_array_msg1->images.size();// + image_array_msg2->images.size();

		new_image_ = total_images > 0;

		images_cache_.clear();
		images_cache_.reserve( total_images );

		for ( _DimType i = 0; i < image_array_msg1->images.size(); ++i )
		{
			images_cache_.push_back( image_array_msg1->images[i] );
		}
/*		for ( _DimType i = 0; i < image_array_msg2->images.size(); ++i )
		{
			images_cache_.push_back( image_array_msg2->images[i] );
		}*/

		printf( "Updated image cache with %d images\n", total_images );

		image_sources_mutex_.unlock();
	}

	void processImage( const IplImage * ipl_image, std::vector<_Contour> & contours, unsigned int color_id )
	{
		if ( !output_image_ ) output_image_ = cvCreateImage( cvSize( ipl_image->width,
		                                                             ipl_image->height ),
		                                                     IPL_DEPTH_8U,
		                                                     3 );

		printf( "Creating mat\n" );

		cv::Mat current_image_mat( ipl_image );

		printf( "Finding contours\n" );
		findContours( current_image_mat, contours );

		printf( "Found %zu contours\n",
		        contours.size() );

		const OutputColorRGB::_CvColorType & current_color = OutputColorRGB::getColorRGB( color_id );

		cv::Mat output_image_mat( output_image_ );
		cv::drawContours( output_image_mat,
		                  contours,
		                  -1,
		                  cvScalar( current_color[0], current_color[1], current_color[2] ),
		                  1 );
	}
};

#endif /* COLOR_BLOB_FINDER_H_ */
