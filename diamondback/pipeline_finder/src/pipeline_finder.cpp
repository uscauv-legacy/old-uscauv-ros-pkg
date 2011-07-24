/*******************************************************************************
 *
 *      pipeline_finder
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
 *      * Neither the name of "pipeline_finder-RelWithDebInfo@pipeline_finder" nor the names of its
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

#include <base_node/base_node.h>
#include <color_blob_finder/contour.h>
#include <pipeline_finder/FindPipelines.h>
#include <pipeline_finder/Pipeline.h>
#include <opencv/cv.h>
#include <opencv/cxcore.h>

typedef unsigned int _DimType;
typedef BaseNode<> _BaseNode;
typedef pipeline_finder::FindPipelines _FindPipelinesService;
typedef pipeline_finder::Pipeline _FindPipelinesMsgType;


class PipelineFinder: public _BaseNode
{
  private:
    ros::ServiceServer find_pipelines_svr_;

  public:
    ros::ServiceServer match_contours_svr_;

    PipelineFinder( ros::NodeHandle & nh ) :
      _BaseNode( nh )
  {
    find_pipelines_svr_ = nh_local_.advertiseService( "find_pipelines", &PipelineFinder::findPipelinesCB, this );
  }

    bool findPipelinesCB( _FindPipelinesService::Request & req,
        _FindPipelinesService::Response & resp )
    {
      ROS_INFO("Let's match some pipelines!");
      return true;
      //		std::vector<_Contour> candidate_contours( req.candidate_contours.size() );
      //
      //		// convert incoming contours
      //		for ( _DimType i = 0; i < req.candidate_contours.size(); ++i )
      //		{
      //			candidate_contours[i] << req.candidate_contours[i];
      //		}
      //
      //		std::vector<_Contour> template_contours( req.template_contours.size() );
      //
      //		for ( _DimType i = 0; i < req.template_contours.size(); ++i )
      //		{
      //			template_contours[i] << req.template_contours[i];
      //		}
      //
      //		// calculate similarity
      //		resp.matched_contours.reserve( candidate_contours.size() );
      //		for ( _DimType i = 0; i < candidate_contours.size(); ++i )
      //		{
      //			_MatchedContourMsgType matched_contour_msg;
      //			matched_contour_msg.match_qualities.reserve( template_contours.size() );
      //
      //			for ( _DimType j = 0; j < template_contours.size(); ++j )
      //			{
      //				matched_contour_msg.match_qualities.push_back( cv::matchShapes( cv::Mat( template_contours[j] ),
      //				                                                                cv::Mat( candidate_contours[i] ),
      //				                                                                CV_CONTOURS_MATCH_I2,
      //				                                                                0 ) );
      //			}
      //
      //			resp.matched_contours.push_back( matched_contour_msg );
      //		}
      //
      //		return true;
      //	}
   }
};

int main( int argc, char ** argv )
{
	ros::init( argc, argv, "contour_matcher" );
	ros::NodeHandle nh( "~" );

	PipelineFinder pipeline_finder( nh );
	pipeline_finder.spin();

	return 0;
}

