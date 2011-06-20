/*******************************************************************************
 *
 *      tf_utils
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
 *      * Neither the name of "interaction-ros-pkg" nor the names of its
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

#ifndef TF_UTILS_H_
#define TF_UTILS_H_

#include <tf/transform_broadcaster.h> // for TransformBroadcaster
#include <tf/transform_listener.h> // for TransformListener
#include <geometry_msgs/Twist.h>

static void operator >>( const geometry_msgs::Twist & the_pose,
                         tf::Transform & the_pose_tf )
{
	the_pose_tf.setOrigin( tf::Vector3( the_pose.linear.x,
	                                    the_pose.linear.y,
	                                    the_pose.linear.z ) );
	the_pose_tf.setRotation( tf::Quaternion( the_pose.angular.z,
	                                         the_pose.angular.y,
	                                         the_pose.angular.x ) );
}

static void operator >>( const tf::Transform & the_pose_tf,
                         geometry_msgs::Twist & the_pose )
{
	the_pose.linear.x = the_pose_tf.getOrigin().x();
	the_pose.linear.y = the_pose_tf.getOrigin().y();
	the_pose.linear.z = the_pose_tf.getOrigin().z();

	the_pose_tf.getBasis().getEulerZYX( the_pose.angular.z,
	                                    the_pose.angular.y,
	                                    the_pose.angular.x );
}

namespace tf_utils
{
	const static tf::Quaternion ZERO_QUAT( 0,
	                                       0,
	                                       0,
	                                       1 );

	static void publishTfFrame( const tf::Transform & transform,
	                            const std::string & from,
	                            const std::string & to,
	                            ros::Time timestamp = ros::Time( 0 ) )
	{
		static tf::TransformBroadcaster * broadcaster = new tf::TransformBroadcaster;
		if ( timestamp == ros::Time( 0 ) ) timestamp = ros::Time::now();
		broadcaster->sendTransform( tf::StampedTransform( transform,
		                                                  timestamp,
		                                                  from,
		                                                  to ) );
	}

	static void fetchTfFrame( tf::Transform & transform,
	                          const std::string & from,
	                          ros::Time from_timestamp,
	                          const std::string & to,
	                          ros::Time to_timestamp,
	                          const std::string & static_frame = "/world",
	                          const double & wait_time = 0.1,
	                          const double & extrapolation_limit = 1.0 )
	{
		static tf::StampedTransform stamped_transform;
		static unsigned int error_count = 0;
		static tf::TransformListener * listener = new tf::TransformListener;
		listener->setExtrapolationLimit( ros::Duration( extrapolation_limit ) );

		try
		{
			listener->waitForTransform( from,
			                            from_timestamp,
			                            to,
			                            to_timestamp,
			                            static_frame,
			                            ros::Duration( wait_time ) );

			listener->lookupTransform( from,
			                           from_timestamp,
			                           to,
			                           to_timestamp,
			                           static_frame,
			                           stamped_transform );

			transform.setOrigin( stamped_transform.getOrigin() );
			transform.setRotation( stamped_transform.getRotation() );

			error_count = 0;
		}
		catch ( tf::TransformException & ex )
		{
			if ( error_count < 10 )
			{
				ROS_ERROR( "%s",
				           ex.what() );
				++error_count;
			}
		}
	}

	static void fetchTfFrame( tf::Transform & transform,
	                          const std::string & from,
	                          const std::string & to,
	                          ros::Time timestamp = ros::Time( 0 ),
	                          const double & wait_time = 0.1,
	                          double extrapolation_limit = 1.0,
	                          unsigned int attempts = 0 )
	{
		static tf::StampedTransform stamped_transform;
		static unsigned int error_count = 0;
		static tf::TransformListener * listener = new tf::TransformListener;
		listener->setExtrapolationLimit( ros::Duration( extrapolation_limit ) );

		try
		{
			listener->waitForTransform( from,
			                            to,
			                            timestamp,
			                            ros::Duration( wait_time ) );

			listener->lookupTransform( from,
			                           to,
			                           timestamp,
			                           stamped_transform );

			transform.setOrigin( stamped_transform.getOrigin() );
			transform.setRotation( stamped_transform.getRotation() );

			if ( attempts == 0 ) error_count = 0;
		}
		catch ( tf::TransformException ex )
		{
			if ( error_count < 10 )
			{
				ROS_ERROR( "%s",
				           ex.what() );
				++error_count;
			}
			ROS_WARN( "Attempting to look up most recent frame instead...\n" );
			if ( attempts == 0 ) fetchTfFrame( transform,
			                                   from,
			                                   to,
			                                   ros::Time( 0 ),
			                                   wait_time,
			                                   extrapolation_limit,
			                                   attempts + 1 );
		}
	}

}

#endif /* TF_UTILS_H_ */
