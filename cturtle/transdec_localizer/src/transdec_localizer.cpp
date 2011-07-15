/*******************************************************************************
 *
 *      transdec_localizer
 *
 *      Copyright (c) 2010
 *
 *      Randolph Voorhies
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

#include <base_image_proc/base_image_proc.h>

typedef BaseImageProc<BaseNodeTypes::_DefaultReconfigureType, std_srvs::Empty> _BaseImageProc;

class TransdecLocalizer: public _BaseImageProc
{
  public:
    // ######################################################################
    TransdecLocalizer(ros::NodeHandle & nh) : _BaseImageProc(nh)
    { initCfgParams(); }

    // ######################################################################
    IplImage* processImage( IplImage * ipl_img, _ServiceRequest & req, _ServiceResponse & resp )
    { 
      ROS_INFO("Got dis image");
      return NULL;
    }

    // ######################################################################
    void reconfigureCB( _ReconfigureType &config, uint32_t level )
    { }
};

int main( int argc, char **argv )
{
	ros::init( argc, argv, "transdec_localizer" );
	ROS_INFO("Starting up...");
	ros::NodeHandle nh("~");

	TransdecLocalizer transdec_localizer( nh );
	ROS_INFO("Ready to receive requests");
	transdec_localizer.spin();

	return 0;
}

