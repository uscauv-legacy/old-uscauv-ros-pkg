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
#include <visualization_msgs/MarkerArray.h>
#include <opencv/cv.h>

class PointCloudTester: public BaseNode<>
{
private:
	ros::Publisher point_cloud_pub_, marker_array_pub_;
	sensor_msgs::PointCloud points_;
	visualization_msgs::MarkerArray markers_;
	int last_marker_size_;

public:
	PointCloudTester( ros::NodeHandle & nh ) :
		BaseNode<> ( nh ), last_marker_size_( 0 )
	{
		point_cloud_pub_ = nh_priv_.advertise<sensor_msgs::PointCloud> ( "points", 1 );
		marker_array_pub_ = nh_priv_.advertise<visualization_msgs::MarkerArray> ( "marker_array", 0 );

		markers_.markers.resize( 10 );

		srand( time( NULL ) );

		populatePoints();
		//populateMarkers();
	}

	void clearMarkers()
	{
		for ( size_t i = 0; i < last_marker_size_; i++ )
		{
			visualization_msgs::Marker & marker = markers_.markers[i];
			marker.header.frame_id = "/world";
			marker.header.stamp = ros::Time();
			marker.ns = "jfdsjsdf";
			marker.id = i;
			marker.action = visualization_msgs::Marker::DELETE;
		}

		marker_array_pub_.publish( markers_ );
	}

	void populateMarkers()
	{
		clearMarkers();

		last_marker_size_ = markers_.markers.size();

		for ( size_t i = 0; i < markers_.markers.size(); i++ )
		{
			double min = -1;
			double max = 1;


			//visualization_msgs::Marker & marker = markers_.markers[i];

			visualization_msgs::Marker marker;

			marker.header.frame_id = "/world";
			marker.header.stamp = ros::Time();
			marker.ns = "jfdsjsdf";
			marker.id = i;
			marker.type = visualization_msgs::Marker::ARROW;
			marker.action = visualization_msgs::Marker::ADD;

			geometry_msgs::Point old_point, new_point;
			old_point.x = 0;
			old_point.y = 0;
			old_point.z = 0;

			new_point.x = min + ( max - min ) * (double) rand() / (double) RAND_MAX;
			new_point.y = min + ( max - min ) * (double) rand() / (double) RAND_MAX;
			new_point.z = min + ( max - min ) * (double) rand() / (double) RAND_MAX;

			printf( "x %f y %f z %f\n", new_point.x, new_point.y, new_point.z );

			marker.points.push_back( old_point );
			marker.points.push_back( new_point );

			marker.scale.x = 0.05;
			marker.scale.y = 0.1;
			marker.scale.z = 0.1;

			marker.color.a = 1.0;
			marker.color.r = 0.0;
			marker.color.g = 1.0;
			marker.color.b = 0.0;

			markers_.markers[i] = marker;
		}
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

			points_.channels[0].values.push_back( *(float*) ( &rgb_packed ) );
			points_.channels[0].name = "rgb";
		}
	}

	void spinOnce()
	{

		point_cloud_pub_.publish( points_ );

		populateMarkers();
		marker_array_pub_.publish( markers_ );


		//printf( "%d\n", markers_.markers.size() );

		ros::Rate( 0.5 ).sleep();
	}
};

int main( int argc, char ** argv )
{
	ros::init( argc, argv, "point_cloud_tester" );
	ros::NodeHandle nh;

	PointCloudTester point_cloud_tester( nh );
	point_cloud_tester.spin( SpinModeId::LOOP_SPIN_ONCE );

	return 0;
}
