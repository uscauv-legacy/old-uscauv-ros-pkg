/***************************************************************************
 *  include/contour_matcher/contour_finder_node.h
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

#ifndef CONTOURMATCHER_CONTOURFINDERNODE_H_
#define CONTOURMATCHER_CONTOURFINDERNODE_H_

#include <quickdev/node.h>

// policies
#include <quickdev/action_server_policy.h>
#include <quickdev/image_proc_policy.h>

// objects
#include <quickdev/multi_subscriber.h>
#include <quickdev/multi_publisher.h>
#include <quickdev/param_reader.h>
#include <set>

// utils
#include <contour_matcher/contour.h>

// actions
#include <seabee3_actions/ConfigureAction.h>

// msgs
#include <seabee3_msgs/NamedImageArray.h>
#include <seabee3_msgs/ContourArray.h>

typedef seabee3_msgs::ContourArray _ContourArrayMsg;

typedef seabee3_msgs::NamedImageArray _NamedImageArrayMsg;

typedef seabee3_actions::ConfigureAction _ConfigureAction;

typedef quickdev::ActionServerPolicy<_ConfigureAction> _ConfigureActionServerPolicy;

// cache most recent classified image array
// start up action server; accept color-specific requests
// run cv::findContours( ... ) on specified images; publish resulting contours
// publish debug image

QUICKDEV_DECLARE_NODE( ContourFinder, _ConfigureActionServerPolicy )

QUICKDEV_DECLARE_NODE_CLASS( ContourFinder )
{
protected:
    ros::MultiSubscriber<> multi_sub_;
    ros::MultiPublisher<> multi_pub_;

    std::mutex process_images_mutex_;
    boost::shared_ptr<boost::thread> process_images_thread_ptr_;

    XmlRpc::XmlRpcValue params_;

    std::set<std::string> color_filter_;
    std::mutex color_filter_mutex_;

    _NamedImageArrayMsg::ConstPtr images_msg_ptr_;
    std::mutex images_mutex_;

    std::map<std::string, cv::Scalar> colors_map_;

    bool show_debug_images_;

    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( ContourFinder )
    {
        // note: bgr
        colors_map_["red"]    = cv::Scalar(   0,   0, 255 );
        colors_map_["orange"] = cv::Scalar(   0, 127, 255 );
        colors_map_["yellow"] = cv::Scalar(   0, 255, 255 );
        colors_map_["green"]  = cv::Scalar(   0, 255,   0 );
        colors_map_["blue"]   = cv::Scalar( 255,   0,   0 );
        colors_map_["black"]  = cv::Scalar(   0,   0,   0 );
        colors_map_["white"]  = cv::Scalar( 255, 255, 255 );

        color_filter_.insert( "green" );
        color_filter_.insert( "orange" );
        color_filter_.insert( "yellow" );
    }

    QUICKDEV_SPIN_FIRST()
    {
        QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );

        multi_sub_.addSubscriber( nh_rel, "classified_images", &ContourFinderNode::namedImageArrayCB, this );
        multi_pub_.addPublishers<_ContourArrayMsg>( nh_rel, { "contours" } );

        _ConfigureActionServerPolicy::registerExecuteCB( quickdev::auto_bind( &ContourFinderNode::configureActionExecuteCB, this ) );

        initPolicies<_ConfigureActionServerPolicy>( "action_name_param", std::string( "set_color_filter" ) );

        params_ = quickdev::ParamReader::readParam<decltype( params_ )>( nh_rel, "params" );
        show_debug_images_ = quickdev::ParamReader::getXmlRpcValue<bool>( params_, "show_debug_images", false );

        initPolicies<quickdev::policy::ALL>();

        process_images_thread_ptr_ = boost::make_shared<boost::thread>( &ContourFinderNode::processImages, this );
    }

    QUICKDEV_DECLARE_ACTION_EXECUTE_CALLBACK( configureActionExecuteCB, _ConfigureAction )
    {
        // lock color_filter mutex
        auto lock = quickdev::make_unique_lock( color_filter_mutex_ );

        auto const & settings = goal->settings;

        // update the color_filter
        for( auto setting_it = settings.cbegin(); setting_it != settings.cend(); ++setting_it )
        {
            auto const & setting = *setting_it;

            if( setting.empty() ) continue;

            // reset the color_filter
            if( setting == "-all" ) color_filter_.clear();
            // remove the given item; -<item>
            else if( setting.substr( 0, 1 ) == "-" ) color_filter_.erase( setting.substr( 1 ) );
            // add the given item; +<item>
            else if( setting.substr( 0, 1 ) == "+" ) color_filter_.insert( setting.substr( 1 ) );
            // default; add the given item <item>
            else color_filter_.insert( setting );
        }
    }

    QUICKDEV_DECLARE_MESSAGE_CALLBACK( namedImageArrayCB, _NamedImageArrayMsg )
    {
        PRINT_INFO( "Got new image array" );
        // try lock image mutex
        auto lock = quickdev::make_unique_lock( images_mutex_, std::try_to_lock );
        // drop message if lock failed
        if( !lock ) return;

        // update image array cache
        images_msg_ptr_ = msg;

        // unlock the processImages thread for one cycle
        process_images_mutex_.unlock();
    }

    void processImages()
    {
        while( QUICKDEV_GET_RUNABLE_POLICY()::running() )
        {
            // make sure mutex is locked once
            process_images_mutex_.try_lock();
            // block until new image received
            auto process_images_lock = quickdev::make_unique_lock( process_images_mutex_ );

            PRINT_INFO( "processing images" );

            // prevent updates to the color_filter while we process the images
            auto color_filter_lock = quickdev::make_unique_lock( color_filter_mutex_ );
            // prevent updates to the images while we process them
            auto images_lock = quickdev::make_unique_lock( images_mutex_ );

            auto const & images = images_msg_ptr_->images;

            _ContourArrayMsg contour_array_msg;

            cv::Mat debug_image;

            for( auto images_msg_it = images.cbegin(); images_msg_it != images.cend(); ++images_msg_it )
            {
                auto const & image_name = images_msg_it->name;
                if( !color_filter_.count( image_name ) ) continue;

                auto const & images_msg = *images_msg_it;

                auto const cv_image_ptr = quickdev::opencv_conversion::fromImageMsg( images_msg.image );
                cv::Mat const & image = cv_image_ptr->image;

                auto image_params = quickdev::ParamReader::getXmlRpcValue<XmlRpc::XmlRpcValue>( params_, image_name );
                auto const threshold_min = quickdev::ParamReader::getXmlRpcValue<int>( image_params, "threshold_min", 100 );
                auto const threshold_max = quickdev::ParamReader::getXmlRpcValue<int>( image_params, "threshold_max", 255 );
                auto const threshold_type = quickdev::ParamReader::getXmlRpcValue<int>( image_params, "threshold_type", cv::THRESH_TOZERO );

                cv::Mat normalized_image;
                cv::Mat thresholded_image;
                cv::Mat contour_image;

                image.copyTo( normalized_image );
                //cv::GaussianBlur( image, normalized_image, cv::Size( 11, 11 ), 5 );
                //cv::normalize( image, normalized_image, 0, 255, CV_MINMAX );
                //cv::adaptiveThreshold( normalized_image, thresholded_image, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 7, 0 );
                cv::threshold( image, thresholded_image, threshold_min, threshold_max, threshold_type );
                thresholded_image.copyTo( contour_image );

                std::vector<_Contour> contours;
                cv::findContours( contour_image, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE );

                for( auto contour_it = contours.cbegin(); contour_it != contours.cend(); ++contour_it )
                {
                    auto const & contour = *contour_it;

                    _ContourMsg contour_msg = unit::implicit_convert( contour );
                    contour_msg.name = image_name;

                    contour_array_msg.contours.push_back( contour_msg );
                }

                if( show_debug_images_ )
                {
                    cv::imshow( image_name + " input", image );
                    cv::imshow( image_name + " normalized", normalized_image );
                    cv::imshow( image_name + " thresholded", thresholded_image );

                    if( debug_image.empty() ) debug_image = cv::Mat( image.size(), CV_8UC3, cv::Scalar( 0, 0, 0 ) );

                    auto color_it = colors_map_.find( image_name );
                    cv::Scalar color = color_it != colors_map_.end() ? color_it->second : cv::Scalar( 255, 0, 255 );

                    for( size_t contour_idx = 0; contour_idx < contours.size(); ++contour_idx )
                    {

                        cv::drawContours( debug_image, contours, contour_idx, color );
                    }
                }
            }

            multi_pub_.publish( "contours", contour_array_msg );

            if( show_debug_images_ )
            {
                cv::imshow( "Contours", debug_image );
                cvWaitKey( 20 );
            }
        }
    }

    QUICKDEV_SPIN_ONCE()
    {
        // return to allow the invoking of ROS callbacks
    }
};

#endif // CONTOURMATCHER_CONTOURFINDERNODE_H_
