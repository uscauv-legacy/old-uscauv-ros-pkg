/***************************************************************************
 *  include/base_libs/multi_publisher.h
 *  --------------------
 * 
 *  Copyright (c) 2011, Edward T. Kaszubski ( ekaszubski@gmail.com )
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
 *  * Neither the name of seabee3-ros-pkg nor the names of its
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

#ifndef BASE_LIBS_BASE_LIBS_MULTI_PUBLISHER_H_
#define BASE_LIBS_BASE_LIBS_MULTI_PUBLISHER_H_

#include <map>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <base_libs/console.h>
#include <base_libs/container.h>
#include <vector>
#include <typeinfo>

//## Adapters ##########################################################

namespace ros
{

// ## PublisherAdapterStorage ##########################################
template<class __Publisher>
struct PublisherAdapterStorage {};

// ## PublisherAdapterStorage for ros::Publisher #######################
template<>
struct PublisherAdapterStorage<ros::Publisher> {};

// ## PublisherAdapter #################################################
template<class __Publisher, class __Message>
class PublisherAdapter {};

// ## PublisherAdapter for ros::Publisher ##############################
template<class __Message>
class PublisherAdapter<ros::Publisher, __Message>
{
public:
	typedef ros::Publisher _Publisher;
	
	static _Publisher createPublisher(
		ros::NodeHandle & nh,
		const std::string & topic,
		const unsigned int & cache_size,
		PublisherAdapterStorage<_Publisher> & storage )
	{
		return nh.advertise<__Message>(
			topic,
			cache_size );
	}
};

// ## MultiPublisher ###################################################
// *given a list of message types, create a publisher and topic for each
// *provide easy functions for publishing and accessing data 
template<class __Publisher = ros::Publisher>
class MultiPublisher
{
public:
	typedef std::string _Topic;
	typedef std::vector<_Topic> _TopicArray;
	typedef __Publisher _Publisher;
	//typedef PublisherAdapter<__Publisher> _PublisherAdapter;
	typedef PublisherAdapterStorage<__Publisher> _PublisherAdapterStorage;
	typedef std::map<_Topic, __Publisher> _PublisherMap;

protected:
	_PublisherMap publishers_;
	
public:
	// default constructor does nothing
	MultiPublisher()
	{
		//
	}
	
	template<class... __Messages>
	MultiPublisher & addPublishers( ros::NodeHandle & nh, const std::initializer_list<_Topic> & topic_names_init, _PublisherAdapterStorage storage = _PublisherAdapterStorage() )
	{
		_TopicArray topic_names( topic_names_init.size() );
		std::copy( topic_names_init.begin(), topic_names_init.end(), topic_names.begin() );
		
		PRINT_INFO( "Attempting to add %zu publishers...", topic_names.size() );
		createPublishers<Container<__Messages...> >( nh, topic_names.begin(), topic_names.end(), storage );
		
		return *this;
	}
	
	// recursively process the message types in __MessagesSubset and
	// create a publisher for each
	template<class __MessagesSubset>
	typename std::enable_if<(__MessagesSubset::num_types_ > 0 ), void>::type
	createPublishers( ros::NodeHandle & nh, typename _TopicArray::iterator current_topic, const typename _TopicArray::iterator last_topic, _PublisherAdapterStorage & storage )
	{
		typedef typename ContainerTypes<__MessagesSubset>::_Front _CurrentMessageType;
		PRINT_INFO( "Creating publisher [%s] on topic [%s/%s]", _CurrentMessageType::__s_getDataType().c_str(), nh.getNamespace().c_str(), current_topic->c_str() );
		
		publishers_[*current_topic] = PublisherAdapter<__Publisher, _CurrentMessageType>::createPublisher( nh, *current_topic, 10, storage );
		createPublishers<typename ContainerTypes<__MessagesSubset>::_Rest>( nh, ++current_topic, last_topic, storage );
	}
	
	// bottom-level in the createPublishers recursive algorithm
	// __MessagesSubset should be Container<>; if so, all topics have
	// been registered
	template<class __MessagesSubset>
	typename std::enable_if<(__MessagesSubset::num_types_ == 0 ), void>::type
	createPublishers( ros::NodeHandle & nh, typename _TopicArray::iterator current_topic, const typename _TopicArray::iterator last_topic, _PublisherAdapterStorage & storage )
	{
		PRINT_INFO( "Finished creating topics." );
	}
	
	// check to see if a topic exists in the list of publishers
	bool exists( const _Topic & topic )
	{
		return publishers_.count( topic );
	}
	
	// indexing operator; allows read-only access of publishers_
	const __Publisher & operator[]( const _Topic & topic )
	{
		if( publishers_.count( topic ) ) return publishers_[topic];
		return __Publisher();
	}
	
	// given a list of key-value pairs, recurse through the list to
	// publish the messages
	template<class __Message, class... __Args>
	void publish( const _Topic & topic, const __Message & msg, __Args&&... args )
	{
		if( exists( topic ) ) publishers_[topic].publish( msg );
		publish( args... );
	}
	
	// bottom-level of recursive publish() algorithm
	// does nothing
	void publish(){}
};

}

#endif // BASE_LIBS_BASE_LIBS_MULTI_PUBLISHER_H_
