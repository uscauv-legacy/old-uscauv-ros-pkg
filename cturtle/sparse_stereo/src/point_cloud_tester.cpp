/*******************************************************************************
 *
 *      point_cloud_tester
 * 
 *      Copyright (c) 2010
 *
 *      Edward T. Kaszubski (ekaszubski@gmail.com)
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

#include <sensor_msgs/PointCloud.h>
#include <base_node/base_node.h>
#include <opencv/cv.h>

class PointCloudTester: public BaseNode<>
{
private:
	ros::Publisher point_cloud_pub_;
	sensor_msgs::PointCloud points_;

public:
	PointCloudTester( ros::NodeHandle & nh ) :
		BaseNode<> ( nh )
	{
		point_cloud_pub_ = nh_priv_.advertise<sensor_msgs::PointCloud> ( "points", 1 );
		srand( time( NULL ) );
		populatePoints();
	}

	void populatePoints()
	{
		size_t num_points = 10;
		double min = -1;
		double max = 1;

		points_.header.stamp = ros::Time::now();
		points_.header.frame_id = "/sparse_stereo";
		points_.channels.resize( num_points );
		points_.points.resize( num_points );

		points_.channels.resize( 1 );
		points_.channels[0].values.reserve( num_points );

		for ( size_t i = 0; i < num_points; i++ )
		{
			points_.points[i].x = min + ( max - min ) * (double) rand() / (double) RAND_MAX;
			points_.points[i].y = min + ( max - min ) * (double) rand() / (double) RAND_MAX;
			points_.points[i].z = min + ( max - min ) * (double) rand() / (double) RAND_MAX;

			cv::Vec3b bgr;

			bgr[0] = 0;
			bgr[1] = 0;
			bgr[2] = 255;

			int32_t rgb_packed = ( bgr[2] << 16 ) | ( bgr[1] << 8 ) | bgr[0];

			points_.channels[0].values.push_back(*(float*) ( &rgb_packed ));
			points_.channels[0].name = "rgb";
		}
	}

	void spinOnce()
	{

		point_cloud_pub_.publish( points_ );

		ros::Rate( 10 ).sleep();
	}
};

int main( int argc, char ** argv )
{
	ros::init( argc, argv, "point_cloud_tester" );
	ros::NodeHandle nh;

	PointCloudTester point_cloud_tester( nh );
	point_cloud_tester.spin( SpinModeId::loop_spin_once );

	return 0;
}
