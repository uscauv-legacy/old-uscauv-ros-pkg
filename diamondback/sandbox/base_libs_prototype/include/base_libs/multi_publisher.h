#ifndef BASE_LIBS_BASE_LIBS_MULTI_PUBLISHER_H_
#define BASE_LIBS_BASE_LIBS_MULTI_PUBLISHER_H_

#include <map>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <base_libs/container.h>
#include <vector>
#include <stdio.h>
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
		
		printf( "Recursing through %zu publishers...\n", topic_names.size() );
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
		printf( "Creating publisher [%s] on topic %s/%s\n", _CurrentMessageType::__s_getDataType().c_str(), nh.getNamespace().c_str(), current_topic->c_str() );
		
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
		printf( "Finished creating topics.\n" );
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
