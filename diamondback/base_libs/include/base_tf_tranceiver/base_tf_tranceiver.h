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
#include <common_utils/tf.h>

template<typename _ReconfigureType = BaseNodeTypes::_DefaultReconfigureType>
class BaseTfTranceiver: public BaseNode<_ReconfigureType>
{
protected:
	tf::TransformListener * transform_listener_;
	tf::TransformBroadcaster * transform_broadcaster_;

private:
	int error_count_;

public:
	BaseTfTranceiver( ros::NodeHandle & nh ) :
		BaseNode<_ReconfigureType> ( nh ), error_count_( 0 )
	{
		transform_listener_ = new tf::TransformListener;
		transform_broadcaster_ = new tf::TransformBroadcaster;
	}

	~BaseTfTranceiver()
	{
		delete transform_listener_;
		delete transform_broadcaster_;
	}

protected:
	void publishTfFrame( const tf::Transform & transform,
	                     const std::string & from,
	                     const std::string & to,
	                     ros::Time timestamp = ros::Time( 0 ) )
	{
		tf_utils::publishTfFrame( transform,
		                          from,
		                          to,
		                          timestamp );
	}

	void fetchTfFrame( tf::Transform & transform,
	                   const std::string & from,
	                   const std::string & to,
	                   double wait_time = 0.1,
	                   ros::Time timestamp = ros::Time( 0 ) )
	{
		tf_utils::fetchTfFrame( transform,
		                        from,
		                        to,
		                        timestamp,
		                        wait_time );
	}
};

#endif /* BASE_TF_TRANCEIVER_H_ */
