#ifndef BASE_LIBS_BASE_LIBS_MULTI_SUBSCRIBER_H_
#define BASE_LIBS_BASE_LIBS_MULTI_SUBSCRIBER_H_

#include <map>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <base_libs/container.h>
#include <vector>
#include <stdio.h>
#include <typeinfo>
#include <base_libs/auto_bind.h>

namespace ros
{

// ## SubscriberAdapter ################################################
template<class __Subscriber>
class SubscriberAdapter {};

// ## SubscriberAdapter for ros::Subscriber ############################
template<>
class SubscriberAdapter<ros::Subscriber>
{
public:
	typedef ros::Subscriber _Subscriber;
	
	template<class __Message>
	static _Subscriber createSubscriber(
		ros::NodeHandle & nh,
		const std::string & topic,
		const unsigned int & cache_size,
		const std::function<void( const boost::shared_ptr<__Message const>& )> & callback )
	{
		return nh.subscribe(
			topic,
			cache_size,
			boost::function< void( const boost::shared_ptr< __Message const>& )>( callback ) );
	}
};


// ## MultiSubscriber ##################################################
// *given a list of message types, create a subscriber and topic for each
// *provide easy functions for publishing and accessing data 
template<class __Subscriber = ros::Subscriber>
class MultiSubscriber
{
public:
	typedef std::string _Topic;
	typedef std::vector<_Topic> _TopicArray;
	typedef __Subscriber _Subscriber;
	//typedef SubscriberAdapter<__Subscriber> _SubscriberAdapter;
	typedef std::map<_Topic, __Subscriber> _SubscriberMap;

protected:
	_SubscriberMap subscribers_;
	
public:
	// default constructor does nothing
	MultiSubscriber()
	{
		//
	}
	
	// here, we only support adding one callback at a time, either through a standard or member function pointer
	template<class __Message, class __Caller>
	MultiSubscriber & addSubscriber( ros::NodeHandle & nh, const _Topic & topic_name, void( __Caller::*function_ptr )( const __Message & ), __Caller * caller )
	{
		return addSubscriber( nh, topic_name, base_libs::auto_bind( function_ptr, caller ) );
	}
	
	template<class __Message>
	MultiSubscriber & addSubscriber( ros::NodeHandle & nh, const _Topic & topic_name, const std::function< void(const boost::shared_ptr< __Message const > &)> & callback )
	{
		printf( "Creating subscriber [%s] on topic %s/%s\n", __Message::__s_getDataType().c_str(), nh.getNamespace().c_str(), topic_name.c_str() );
		
		subscribers_[topic_name] = SubscriberAdapter<__Subscriber>::createSubscriber( nh, topic_name, 10, callback );
		
		return *this;
	}
	
	// check to see if a topic exists in the list of subscribers
	bool exists( const _Topic & topic )
	{
		return subscribers_.count( topic );
	}
	
	// indexing operator; allows read-only access of subscribers_
	const __Subscriber & operator[]( const _Topic & topic )
	{
		if( subscribers_.count( topic ) ) return subscribers_[topic];
		return __Subscriber();
	}
};

}

#endif // BASE_LIBS_BASE_LIBS_MULTI_SUBSCRIBER_H_
