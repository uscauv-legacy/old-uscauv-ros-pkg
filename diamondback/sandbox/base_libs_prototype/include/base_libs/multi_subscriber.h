#ifndef BASE_LIBS_BASE_LIBS_MULTI_SUBSCRIBER_H_
#define BASE_LIBS_BASE_LIBS_MULTI_SUBSCRIBER_H_

// copypasta'd from multi_publisher with types renamed
// need to take the time to do all of the std::function references later
/*#include <map>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <base_libs/container.h>
#include <vector>
#include <stdio.h>
#include <typeinfo>

namespace ros
{

// *given a list of message types, create a subscriber and topic for each
// *provide easy functions for publishing and accessing data 
class MultiSubscriber
{
public:
	typedef std::string _Topic;
	typedef std::vector<_Topic> _TopicArray;
	typedef ros::Subscriber _Subscriber;
	typedef std::map<_Topic, _Subscriber> _SubscriberMap;

protected:
	_SubscriberMap subscribers_;
	
public:
	// default constructor does nothing
	MultiSubscriber()
	{
		//
	}
	
	template<class... __Messages>
	MultiSubscriber & addSubscribers( ros::NodeHandle & nh, const std::initializer_list<_Topic> & topic_names_init )
	{
		_TopicArray topic_names( topic_names_init.size() );
		std::copy( topic_names_init.begin(), topic_names_init.end(), topic_names.begin() );
		
		printf( "Recursing through %zu subscribers...\n", topic_names.size() );
		createSubscribers<Container<__Messages...> >( nh, topic_names.begin(), topic_names.end() );
		
		return *this;
	}
	
	// recursively process the message types in __MessagesSubset and
	// create a subscriber for each
	template<class __MessagesSubset>
	typename std::enable_if<(__MessagesSubset::num_types_ > 0 ), void>::type
	createSubscribers( ros::NodeHandle & nh, typename _TopicArray::iterator current_topic, const typename _TopicArray::iterator last_topic )
	{
		typedef typename ContainerTypes<__MessagesSubset>::_Front _CurrentMessageType;
		printf( "Creating subscriber [%s] on topic %s/%s\n", _CurrentMessageType::__s_getDataType().c_str(), nh.getNamespace().c_str(), current_topic->c_str() );
		
		subscribers_[*current_topic] = nh.advertise<_CurrentMessageType>( *current_topic, 10 );
		createSubscribers<typename ContainerTypes<__MessagesSubset>::_Rest>( nh, ++current_topic, last_topic );
	}
	
	// bottom-level in the createSubscribers recursive algorithm
	// __MessagesSubset should be Container<>; if so, all topics have
	// been registered
	template<class __MessagesSubset>
	typename std::enable_if<(__MessagesSubset::num_types_ == 0 ), void>::type
	createSubscribers( ros::NodeHandle & nh, typename _TopicArray::iterator current_topic, const typename _TopicArray::iterator last_topic )
	{
		printf( "Finished creating topics.\n" );
	}
	
	// check to see if a topic exists in the list of subscribers
	bool exists( const _Topic & topic )
	{
		return subscribers_.count( topic );
	}
	
	// indexing operator; allows read-only access of subscribers_
	const _Subscriber & operator[]( const _Topic & topic )
	{
		if( subscribers_.count( topic ) ) return subscribers_[topic];
		return _Subscriber();
	}
	
	// given a list of key-value pairs, recurse through the list to
	// publish the messages
	template<class __Message, class... __Args>
	void publish( const _Topic & topic, const __Message & msg, __Args&&... args )
	{
		if( exists( topic ) ) subscribers_[topic].publish( msg );
		publish( args... );
	}
	
	// bottom-level of recursive publish() algorithm
	// does nothing
	void publish(){}
};

}*/

#endif // BASE_LIBS_BASE_LIBS_MULTI_SUBSCRIBER_H_
