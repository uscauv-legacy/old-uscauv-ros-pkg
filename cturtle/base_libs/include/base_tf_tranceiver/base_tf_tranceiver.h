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

#ifndef BASE_TF_TRANCEIVER_H_
#define BASE_TF_TRANCEIVER_H_

#include <base_node/base_node.h>
#include <tf/transform_broadcaster.h> // for TransformBroadcaster
#include <tf/transform_listener.h> // for TransformListener
#include <geometry_msgs/Twist.h>

void operator >>( const geometry_msgs::Twist & the_pose, tf::Transform & the_pose_tf );
void operator >>( const tf::Transform & the_pose_tf, geometry_msgs::Twist & the_pose );

template<typename _ReconfigureType = BaseNodeTypes::_DefaultReconfigureType>
class BaseTfTranceiver: public BaseNode<_ReconfigureType>
{
protected:
	tf::TransformListener * tl_;
	tf::TransformBroadcaster * tb_;

private:
	int error_count_;

public:
	BaseTfTranceiver( ros::NodeHandle & nh, uint threads = 3 );
	~BaseTfTranceiver();

protected:
	void fetchTfFrame( tf::Transform & transform, const std::string & frame1, const std::string & frame2 );
	void publishTfFrame( tf::Transform & transform, const std::string & frame1, const std::string & frame2 );
};

template<typename _ReconfigureType>
BaseTfTranceiver<_ReconfigureType>::BaseTfTranceiver( ros::NodeHandle & nh, uint threads ) :
	BaseNode<_ReconfigureType> ( nh, threads ), error_count_( 0 )
{
	tl_ = new tf::TransformListener;
	tb_ = new tf::TransformBroadcaster;
}

template<typename _ReconfigureType>
BaseTfTranceiver<_ReconfigureType>::~BaseTfTranceiver()
{
	delete tl_;
	delete tb_;
}

template<typename _ReconfigureType>
void BaseTfTranceiver<_ReconfigureType>::fetchTfFrame( tf::Transform & transform, const std::string & frame1, const std::string & frame2 )
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
		if ( error_count_ < 10 )
		{
			ROS_ERROR( "%s", ex.what() );
			error_count_++;
		}

	}
}

template<typename _ReconfigureType>
void BaseTfTranceiver<_ReconfigureType>::publishTfFrame( tf::Transform & transform, const std::string & frame1, const std::string & frame2 )
{
	tb_->sendTransform( tf::StampedTransform( transform, ros::Time::now(), frame1, frame2 ) );
}

#endif /* BASE_TF_TRANCEIVER_H_ */
