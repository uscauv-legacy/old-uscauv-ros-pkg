/*******************************************************************************
 *
 *      color_segmenter
 *
 *      Copyright (c) 2010
 *
 *      Mike Gerow
 *
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

// for BaseImageProc
#include <base_image_proc/base_image_proc.h>
// for OutputColorRGB, ColorIds
#include <color_defs/colors.h>
// for FindBlobs service
#include <color_segmenter/FindBlobs.h>
// for ColorSegmenter reconfigure types
#include <color_segmenter/ColorSegmenterConfig.h>
#include <std_srvs/Empty.h>
#include <deque>
#include <vector>
#include <queue>

typedef BaseImageProc<color_segmenter::ColorSegmenterConfig, std_srvs::Empty> _BaseImageProc;

class ColorSegmenter: public _BaseImageProc
{
private:
	bool use_flood_fill_;
	IplImage * ipl;
	cv::Vec3b target_color;
public:
	ColorSegmenter( ros::NodeHandle & nh ) :
		_BaseImageProc( nh ), use_flood_fill_( false )
	{
		initCfgParams();
	}

	cv::Mat processImage( IplImage * ipl_img,
	                      _ServiceRequest & req,
	                      _ServiceResponse & resp )
	{
		//req.blob_descriptor.min_mass
		//req.blob_descriptor.color will be some number in ColorIds::*

		//color_segmenter::ColorBlob blob;
		//resp.blob_array.color_blobs.push_back( blob );

		//Use dynamic reconfigure for debug.  That'd be super fancy.
		/** use_flood_fill_ is now set by dynamic_reconfigure in ColorSegmenter::reconfigureCB; edit color_segmenter/cfg/ColorSegmenter.cfg to add more params **/

		/** use the inherited (BaseImageProcCore::publish_image_) param publish_image<bool; default:true> instead; set it in the command-line via _publish_image:=value **/
		bool debug = publish_image_;
		ipl = ipl_img;

		cv_img_ = cv::Mat( ipl_img );
		//makeImageBinary(cv_img_, req.blob_descriptor.color);
		cv::Vec3b color = OutputColorRGB::getColorVector( req.blob_descriptor.color );
		this->target_color = color;
		if ( use_flood_fill_ )
		{
			resp.blob_array.color_blobs = floodFillMethod( cv_img_,
			                                               req.blob_descriptor.min_mass );
		}
		else
		{
			resp.blob_array.color_blobs = blobFindingMethod( cv_img_,
			                                                 req.blob_descriptor.min_mass,
			                                                 color );
		}

		if ( debug )
		{
			drawSegments( cv_img_,
			              resp.blob_array.color_blobs );
		}
		for (unsigned int i = 0; i < resp.blob_array.color_blobs.size(); i++)
		{
			resp.blob_array.color_blobs[i].color = req.blob_descriptor.color;
		}
		for (unsigned int i = 0; i < resp.blob_array.color_blobs.size(); ++i)
		{
			this->buildReferenceImage(resp.blob_array.color_blobs[i]);
		}
		//
		///Changes at the end since the methods above rely on the origin being in the top left
		changeOriginToCenter(cv_img_, resp.blob_array.color_blobs);
		return cv_img_;
	}


	//void makeImageBinary(cv::Mat & cv_img_, int color_int)
	//{
	//	cv::Vec3b color;
	//	switch (color_int)
	//	{
	//	case ColorIds::red: 0
	//		color = OutputColorRGB::red;
	//		break;
	//	case ColorIds::orange: 1
	//		color = OutputColorRGB::orange;
	//		break;
	//	case ColorIds::yellow: 2
	//		color = OutputColorRGB::yellow;
	//		break;
	//	case ColorIds::green: 3
	//		color = OutputColorRGB::green;
	//		break;
	//	case ColorIds::blue: 4
	//		color = OutputColorRGB::blue;
	//		break;
	//	case ColorIds::black: 5
	//		color = OutputColorRGB::black;
	//		break;
	//	case ColorIds::white: 6
	//		color = OutputColorRGB::white;
	//		break;
	//	case ColorIds::unknown: 7
	//		color = OutputColorRGB::unknown;
	//		break;
	//	}
	//
	//DO SOMETHING FANCY HERE TO MAKE THE IMAGE TWO COLORS
	//
	//	return;
	//}

	color_segmenter::ColorBlob findBlob( cv::Mat & cv_img_,
	                                     std::vector<std::vector<bool> > & table,
	                                     int initx,
	                                     int inity,
	                                     bool include_diagonals,
	                                     cv::Vec3b & color )
	{
		color_segmenter::ColorBlob blob;
		blob.xmax = blob.ymax = 0;
		blob.xmin = cv_img_.rows - 1;
		blob.ymin = cv_img_.cols - 1;
		std::deque<cv::Point> d;
		cv::Point curpoint;
		int modx, mody;
		long totx, toty;
		int pixel_shifts = 4;

		d.push_back( cv::Point( initx,
		                        inity ) );
		blob.mass = 1;
		totx = initx;
		toty = inity;
		while ( d.size() != 0 )
		{
			curpoint = d.front();
			d.pop_front();
			if ( include_diagonals )
			{
				pixel_shifts = 8;
			}
			for ( int i = 0; i < pixel_shifts; i++ )
			{
				switch ( i )
				{
				case 0:
					modx = 1;
					mody = 0;
					break;
				case 1:
					modx = -1;
					mody = 0;
					break;
				case 2:
					modx = 0;
					mody = 1;
					break;
				case 3:
					modx = 0;
					mody = -1;
					break;
				case 4:
					modx = 1;
					mody = 1;
					break;
				case 5:
					modx = -1;
					mody = -1;
					break;
				case 6:
					modx = -1;
					mody = 1;
					break;
				case 7:
					modx = 1;
					mody = -1;
					break;
				}
				if ( curpoint.x + modx >= 0 && curpoint.y + mody >= 0 && curpoint.x + modx < cv_img_.cols && curpoint.y + mody < cv_img_.rows
				        && table[curpoint.x + modx][curpoint.y + mody] && getBinPixelValue( color,
				                                                                            curpoint.x + modx,
				                                                                            curpoint.y + mody ) )
				{
					table[curpoint.x + modx][curpoint.y + mody] = false;
					blob.mass += 1;
					totx += curpoint.x + modx;
					toty += curpoint.y + mody;
					if (curpoint.x < blob.xmin)
					    blob.xmin = curpoint.x;
					if (curpoint.y < blob.ymin)
					    blob.ymin = curpoint.y;
					if (curpoint.x > blob.xmax)
					    blob.xmax = curpoint.x;
					if (curpoint.y > blob.ymax)
					    blob.ymax = curpoint.y;
					//blob.table.data.push_back(true);
					d.push_back( cv::Point( curpoint.x + modx,
					                        curpoint.y + mody ) );
				}
			}
		}
		blob.x = static_cast<double> ( totx ) / blob.mass;
		blob.y = static_cast<double> ( toty ) / blob.mass;
		return blob;
	}

	std::vector<color_segmenter::ColorBlob> blobFindingMethod( cv::Mat & cv_img_,
	                                                           int min_mass,
	                                                           cv::Vec3b & color,
	                                                           bool include_diagonals = false )
	{
		std::vector<color_segmenter::ColorBlob> blob_vec;
		std::vector<std::vector<bool> > table;
		std::vector<bool> horizvec;
		color_segmenter::ColorBlob blob;


		//Build a 2d vector of boolean values to mark which pixels have yet to be searched
		ROS_DEBUG("The image is %d pixels by %d pixels.", cv_img_.cols, cv_img_.rows);
		for ( int i = 0; i < cv_img_.rows; i++ ) //ROWS
		{
			horizvec.push_back( true );
		}
		for ( int i = 0; i < cv_img_.cols; i++ )  //COLS
		{
			table.push_back( horizvec );
		}

		ROS_DEBUG("The test table is %d by %d elements.", table.size(), table[0].size());

		for ( int x = 0; x < cv_img_.cols; x++ )  //ROWS
		{
			for ( int y = 0; y < cv_img_.rows; y++ )  //COLS
			{
				ROS_DEBUG("Looking at pixel %d, %d", x, y);
				if ( table[x][y] )
				{
					table[x][y] = false;
					if ( getBinPixelValue( color,
					                       x,
					                       y ) == true )
					{
						blob = findBlob( cv_img_,
						                 table,
						                 x,
						                 y,
						                 include_diagonals,
						                 color );
						if ( blob.mass >= min_mass )
						{
							blob_vec.push_back( blob );
							ROS_INFO( "Found blob at (%lf, %lf) with a mass of %lf.\n",
							          blob.x,
							          blob.y,
							          blob.mass );
						}
					}
				}
			}
		}
		return blob_vec;
	}

	std::vector<color_segmenter::ColorBlob> floodFillMethod( cv::Mat & cv_img_,
	                                                         int min_mass )
	{
		//Implement cv::floodFill to
		std::vector<color_segmenter::ColorBlob> color_blobs;

		return color_blobs;
	}

	bool getBinPixelValue( cv::Vec3b & color,
	                       int x,
	                       int y )
	{
		CvScalar s;
		//ROS_INFO("Getting value at %d, %d", x, y);
		s = cvGet2D( this->ipl,
		             y,
		             x );
		if ( color[0] == s.val[0] && color[1] == s.val[1] && color[2] == s.val[2] )
		{
			return true;
		}
		return false;
	}

	void drawSegments( cv::Mat & cv_img_,
	                   std::vector<color_segmenter::ColorBlob> & color_blobs )
	{
		for ( unsigned int i = 0; i < color_blobs.size(); i++ )
		{
			cv::circle( cv_img_,
			            cv::Point( color_blobs[i].x,
			                       color_blobs[i].y ),
			            4,
			            cv::Scalar( 0,
			                        0,
			                        255 ) );
			cv::putText( cv_img_,
			             weightString( i,
			                           color_blobs[i].mass ),
			             cv::Point( color_blobs[i].x,
			                        color_blobs[i].y ),
			             0,
			             0.5,
			             cv::Scalar( 0,
			                         0,
			                         255 ) );
		}
		return;
	}

	std::string weightString( int i,
	                          double weight )
	{
		std::stringstream s;
		s << "Segment " << i << ". Mass: " << weight;
		return s.str();
	}

	void reconfigureCB( _ReconfigureType &config,
	                    uint32_t level )
	{
		// param_name_ = config.param_name;
		use_flood_fill_ = config.use_flood_fill;
	}

	void changeOriginToCenter(cv::Mat &img, std::vector<color_segmenter::ColorBlob> &blobVector)
	{
		for (unsigned int i = 0; i < blobVector.size(); i++)
		{
			blobVector[i].x -= (img.cols / 2);
			blobVector[i].y -= (img.rows / 2);
		}
	}

	void buildReferenceImage(color_segmenter::ColorBlob &blob)
	{
		IplImage *imgs[4];
		cvSetImageROI(this->ipl, cvRect(blob.xmin, blob.ymin, blob.xmax, blob.ymax));
		cv::Vec3b color = OutputColorRGB::getColorVector( blob.color );
		cvSplit(this->ipl, imgs[0], imgs[1], imgs[2], imgs[3]);
		for (int i = 0; i < this->ipl->nChannels; ++i)
		{
			cvCmpS(imgs[i], color[i], imgs[i], CV_CMP_EQ);
		}
		for (int i = 1; i < this->ipl->nChannels; ++i)
		{
			cvAnd(imgs[0], imgs[1], imgs[0]);
		}
		sensor_msgs::CvBridge::fromIpltoRosImage(imgs[0], blob.image);
		for (int i = 0; i < 4; ++i)
		{
			cvReleaseImage(&imgs[i]);
		}
	}
};

int main( int argc,
          char **argv )
{
	ros::init( argc,
	           argv,
	           "color_segmenter" );
	ROS_INFO("Starting up...");
	ros::NodeHandle nh;

	ColorSegmenter color_segmenter( nh );
	ROS_INFO("Ready to receive requests");
	color_segmenter.spin();

	return 0;
}
