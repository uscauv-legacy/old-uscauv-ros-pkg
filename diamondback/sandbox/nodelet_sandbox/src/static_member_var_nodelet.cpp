/*******************************************************************************
 *
 *      static_member_var_nodelet
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

#include <string>
#include <stdio.h>

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <std_msgs/String.h>

namespace nodelet_sandbox
{
	class StaticMemberVarNodelet : public nodelet::Nodelet
	{
	private:
		/* static member vars */
		static std::string last_message_;

		/* subs */
		ros::Subscriber string_sub_;

		/* pubs */
		ros::Publisher string_pub_;

		/* params */
		std::string string_pub_topic_name_;

		/* others */
		ros::NodeHandle nh_;
		ros::NodeHandle nh_local_;
	public:


		StaticMemberVarNodelet() :
			nodelet::Nodelet()
		{

		}

		void onInit()
		{
			nh_ = getNodeHandle();
			nh_local_ = getPrivateNodeHandle();

			printf("init\n");

			nh_local_.param("string_pub_topic_name", string_pub_topic_name_, std::string("messages") );

			string_sub_ = nh_local_.subscribe( "message", 1, &StaticMemberVarNodelet::messageCB, this );
			string_pub_ = nh_.advertise<std_msgs::String>( string_pub_topic_name_, 1 );
		}

		void messageCB( const std_msgs::String::ConstPtr & string_msg )
		{
			printf( "%s: last message: %s\n", getName().c_str(), last_message_.c_str() );
			last_message_ = string_msg->data;
			printf( "%s: current message: %s\n", getName().c_str(), last_message_.c_str() );

			ros::Duration( 1 ).sleep();

			std_msgs::String::Ptr outgoing_msg( new std_msgs::String );
			outgoing_msg->data = last_message_;
			string_pub_.publish( outgoing_msg );
		}
	};

	std::string StaticMemberVarNodelet::last_message_ = "INIT";
}

// Register the nodelet
#include <pluginlib/class_list_macros.h>
// namespace | name | derived class path | base class path
PLUGINLIB_DECLARE_CLASS(nodelet_sandbox, StaticMemberVar, nodelet_sandbox::StaticMemberVarNodelet, nodelet::Nodelet)
