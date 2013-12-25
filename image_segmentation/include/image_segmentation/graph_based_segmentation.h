/***************************************************************************
 *  include/image_segmentation/graph_based_segmentation.h
 *  --------------------
 *
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Dylan Foster (turtlecannon@gmail.com)
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
 *  * Neither the name of USC AUV nor the names of its
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


#ifndef USCAUV_IMAGESEGMENTATION_GRAPHBASEDSEGMENTATION
#define USCAUV_IMAGESEGMENTATION_GRAPHBASEDSEGMENTATION

// ROS
#include <ros/ros.h>

#include <uscauv_common/multi_reconfigure.h>

#include <image_segmentation/GraphBasedSegmentationConfig.h>
#include <image_segmentation/segment-image.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace uscauv
{

  class GraphBasedSegmentation: public MultiReconfigure
  {
  private:
    typedef image_segmentation::GraphBasedSegmentationConfig _Config;

    _Config config_;
     
  public:
    GraphBasedSegmentation()
      {}

    void init()
    {
      addReconfigureServer<_Config>("segmentation", &GraphBasedSegmentation::reconfigureCallback, this );
    }

    /** 
     * Segment an image. Most of this function is concerned with converting between cv::Mat and the primitive
     * image type used by the segmentation implementation. Very copying.
     * p
     * @param input Image to segmented (presumed to be BGR8)
     * 
     * @return Segmented image (BRG8)
     */
    cv::Mat segment( cv::Mat const & input )
      {
	
	image<rgb> converted_input(input.cols, input.rows, false);
	
	for(int idy = 0; idy < input.rows; ++idy )
	  {
	    for(int idx = 0; idx < input.cols; ++idx )
	      {
		cv::Vec3b const & px = input.at<cv::Vec3b>(idy, idx);
		converted_input.access[idy][idx].b = px[0];
		converted_input.access[idy][idx].g = px[1];
		converted_input.access[idy][idx].r = px[2];
	      }
	  }
	

	int n_components;

	image<rgb> * seg = segment_image( &converted_input, config_.sigma, config_.k, config_.min, &n_components );

	cv::Mat output(input.rows, input.cols, CV_8UC3);

	for(int idy = 0; idy < output.rows; ++idy )
	  {
	    for(int idx = 0; idx < output.cols; ++idx )
	      {
		cv::Vec3b & px = output.at<cv::Vec3b>(idy, idx);
		px[0] = seg->access[idy][idx].b;
		px[1] = seg->access[idy][idx].g;
		px[2] = seg->access[idy][idx].r;
	      }
	  }
	
	delete seg;
	
	return output;
      }

  private:
    void reconfigureCallback( _Config const & config)
    {
      config_ = config;
    }
     
  };
    
} // uscauv

#endif // USCAUV_IMAGESEGMENTATION_GRAPHBASEDSEGMENTATION
