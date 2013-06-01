/***************************************************************************
 *  include/uscauv_common/image_loader.h
 *  --------------------
 *
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Dylan Foster
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
 *  * Neither the name of USC AUV nor the names of its
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


#ifndef USCAUV_USCAUVCOMMON_IMAGELOADER
#define USCAUV_USCAUVCOMMON_IMAGELOADER

// ROS
#include <ros/ros.h>

/// xmlrpcpp
#include <XmlRpcValue.h>

/// cpp11
#include <unordered_map>

/// opencv2
#include <opencv2/highgui/highgui.hpp>

/// TODO: Version of this class for generic storage. Uses a loadParam overloaded for the storage type
/// and uses it to populate the map with files whose urls are found at load argument

namespace uscauv
{
  
  class ImageLoader
  {
  private:
    typedef std::unordered_map< std::string, cv::Mat > _NamedImageMap;
    typedef std::map<std::string, XmlRpc::XmlRpcValue> _NamedXmlMap;
    typedef XmlRpc::XmlRpcValue _XmlVal;
  

    _NamedImageMap image_storage_;
  
    ros::NodeHandle nh_;
  
  public:
    typedef _NamedImageMap::iterator iterator;
    typedef _NamedImageMap::const_iterator const_iterator;
    typedef _NamedImageMap::size_type size_type;
  
  public:
  ImageLoader( ros::NodeHandle nh = ros::NodeHandle()): nh_(nh){}
  ImageLoader( std::string const & ns ): nh_(ns){}

    bool loadImagesAt( std::string const & ns , int flags = CV_LOAD_IMAGE_COLOR)
    {
      std::string resolved_ns = nh_.resolveName( ns, true );
    
      ROS_INFO("Loading images [ %s ]...", resolved_ns.c_str() );
    
      _XmlVal image_urls;
      if( !nh_.getParam( resolved_ns, image_urls ) )
	{
	  ROS_WARN( "Invalid image param namespace [ %s ].", resolved_ns.c_str() );
	  return -1;
	}

      for( _NamedXmlMap::iterator url_it = image_urls.begin(); 
	   url_it != image_urls.end(); ++url_it )
	{
	  std::string const & image_name = url_it->first;
	  std::string const & url = url_it->second;

	  ROS_INFO("Loading image [ %s ] from [ %s ]...", image_name.c_str(), url.c_str() );
	
	  cv::Mat image = cv::imread( url, flags );
	
	  if ( image.data == NULL )
	    ROS_WARN("Load failed.");
	  else
	    {
	      if( image_storage_.find( image_name) != image_storage_.end() )
		ROS_WARN( "Image with name [ %s ] is already loaded. Overwriting...", 
			  image_name.c_str() );

	      image_storage_[ image_name ] = image;

	      ROS_INFO("Load success.");
	    }
	}

      ROS_INFO( "Image loading complete.");

      if( image_storage_.size() > 0 )
   	return 0;
      else
	{
	  ROS_WARN("No images were loaded.");
	  return -1;
	}
    }

    // Map-like access ###########################################
    iterator       begin()        { return image_storage_.begin();  }
    iterator       end()          { return image_storage_.end();    }
    const_iterator begin()  const { return image_storage_.begin();  }
    const_iterator end()    const { return image_storage_.end();    }
    const_iterator cbegin() const { return image_storage_.cbegin(); }
    const_iterator cend()   const { return image_storage_.cend();   }

    size_type size() const { return image_storage_.size(); }

    cv::Mat & operator[](std::string const & key){ return image_storage_[ key ]; }
    iterator find(std::string const & key ){ return image_storage_.find( key ); }
    const_iterator find(std::string const & key ) const { return image_storage_.find( key ); }
  
  };

}
#endif // USCAUV_USCAUVCOMMON_IMAGELOADER
