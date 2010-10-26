/*******************************************************************************
 *
 *      base_tf_tranceiver
 * 
 *      Copyright (c) 2010, Edward T. Kaszubski (ekaszubski@gmail.com)
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

#include <base_tf_tranceiver/base_tf_tranceiver.h>

BaseTfTranceiver::BaseTfTranceiver( ros::NodeHandle & nh, uint threads ) :
	BaseNode( nh, threads ), error_count_( 0 )
{
	tl_ = new tf::TransformListener;
	tb_ = new tf::TransformBroadcaster;
}

BaseTfTranceiver::~BaseTfTranceiver()
{
	delete tl_;
	delete tb_;
}

void BaseTfTranceiver::fetchTfFrame( tf::Transform & transform, const std::string & frame1, const std::string & frame2 )
{
	tf::StampedTransform temp;
	try
	{
		tl_->waitForTransform( frame1, frame2, ros::Time( 0 ), ros::Duration( 0.0 ) );
		tl_->lookupTransform( frame1, frame2, ros::Time( 0 ), temp );

		transform.setOrigin( temp.getOrigin() );
		transform.setRotation( temp.getRotation() );

		error_count_ = 0;
	}
	catch ( tf::TransformException ex )
	{
		if( error_count_ < 10 )
		{
			ROS_ERROR( "%s", ex.what() );
			error_count_ ++;
		}

	}
}

void BaseTfTranceiver::publishTfFrame( tf::Transform & transform, const std::string & frame1, const std::string & frame2 )
{
	tb_->sendTransform( tf::StampedTransform( transform, ros::Time::now(), frame1, frame2 ) );
}

void operator >>( const geometry_msgs::Twist & the_pose, tf::Transform & the_pose_tf )
{
	the_pose_tf.setOrigin( tf::Vector3( the_pose.linear.x, the_pose.linear.y, the_pose.linear.z ) );
	the_pose_tf.setRotation( tf::Quaternion( the_pose.angular.z, the_pose.angular.y, the_pose.angular.x ) );
}

void operator >>( const tf::Transform & the_pose_tf, geometry_msgs::Twist & the_pose )
{
	the_pose.linear.x = the_pose_tf.getOrigin().x();
	the_pose.linear.y = the_pose_tf.getOrigin().y();
	the_pose.linear.z = the_pose_tf.getOrigin().z();

	the_pose_tf.getBasis().getEulerZYX( the_pose.angular.z, the_pose.angular.y, the_pose.angular.x );
}
