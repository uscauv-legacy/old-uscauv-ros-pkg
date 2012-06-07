/***************************************************************************
 *  include/pipe_finder/pipe_finder_node.h
 *  --------------------
 *
 *  Copyright (c) 2011, kathryn
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

#ifndef PIPEFINDER_PIPEFINDERNODE_H_
#define PIPEFINDER_PIPEFINDERNODE_H_
#define CVCLOSE_ITR 1

#include <quickdev/node.h>

// objects
#include <quickdev/multi_subscriber.h>

// utils
#include <quickdev/opencv_conversion.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

// msgs
#include <seabee3_msgs/NamedImageArray.h>

typedef seabee3_msgs::NamedImageArray _NamedImageArrayMsg;

// declare a node called PipeFinderNode
// a quickdev::RunablePolicy is automatically prepended to the list of policies our node will use
// to use more policies, simply list them here:
//
// QUICKDEV_DECLARE_NODE( PipeFinder, SomePolicy1, SomePolicy2 )
//
QUICKDEV_DECLARE_NODE( PipeFinder )

// declare a class called PipeFinderNode
//
QUICKDEV_DECLARE_NODE_CLASS( PipeFinder )
{
    ros::MultiSubscriber<> multi_sub_;
    // variable initializations can be appended to this constructor as a comma-separated list:
    //
    // QUICKDEV_DECLARE_NODE_CONSTRUCTOR( PipeFinder ), member1_( some_value ), member2_( some_other_value ){}
    //
    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( PipeFinder )
    {
        //
        cv::namedWindow( "Found_boxes", 0 );
    }

    // this function is called by quickdev::RunablePolicy after all policies are constructed but just before the main loop is started
    // all policy initialization should be done here
    //
    QUICKDEV_SPIN_FIRST()
    {
        // say we had a policy called _SomePolicy that looked for the meta-parameter "some_value1_param" of type SomeType and
        // "some_value2_param" of type SomeOtherType in its init function
        // we can create those meta-params here and then pass them to all policies using initAll():
        //
        // initAll( "some_value1_param", SomeType(), "some_value2_param", SomeOtherType() );
        //
        // or we can pass those meta-params only to _SomePolicy using its init() function:
        //
        // _SomePolicy::init( "some_value1_param", SomeType(), "some_value2_param", SomeOtherType() );
        //
        // if we don't want to initialize all policies and use their default values, we can simply call initAll() with no arguments
        // note that most initable policies won't function properly unless their init() functions are called
        // therefore, to get the default behavior from all policies, be sure to call initAll()
        //
        //initAll();

        initPolicies<quickdev::policy::ALL>();

        QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );

        multi_sub_.addSubscriber( nh_rel, "images", &PipeFinderNode::imagesCB, this );
    }

    // this opitonal function is called by quickdev::RunablePolicy at a fixed rate (defined by the ROS param _loop_rate)
    // most updateable policies should have their update( ... ) functions called within this context
    //

    /*IMAGE_PROC_PROCESS_IMAGE( image_ptr )
    {
        const float ASPECT_RATIO_BOUNDARY = 1.5;
        const float perimScale = 10;
        cv::Mat input = image_ptr->image;
        cv::Mat gray_input;
        std::vector<CvBox2D> usable_boxes;
        CvMemStorage* storage;
        CvSeq* contours;
        CvBox2D box_to_check;
        storage = cvCreateMemStorage(0);
        contours = cvCreateSeq(
                                CV_SEQ_ELTYPE_POINT,
                                sizeof( CvSeq ),
                                sizeof( CvPoint ),
                                storage
                              );
        //make 8uc1 Mat
        cvtColor(input, gray_input, CV_RGB2GRAY);

        //clean up image
        cv::Mat elem = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5));
        cv::morphologyEx(gray_input, gray_input, cv::MORPH_OPEN, elem, cv::Point(-1,-1), 3);
        //cv::morphologyEx(gray_input, gray_input, elem, cv::MORPH_OPEN, cv::Point(-1,-1), 3);

                //begin contour finder
        cvFindContours(
                        &IplImage( gray_input ),
                        storage, &contours,
                        sizeof( CvContour ),
                        CV_RETR_LIST,
                        CV_CHAIN_APPROX_SIMPLE
                       );
        //iterate through found contours
        while(contours != NULL)
        {
            float aspect_ratio = 0;
            if( contours->total >= 6 )
            {
                //make cvBox2D from potentialy usable contours, and find actual usable ones by their aspect ratios
                box_to_check = cvFitEllipse2( contours );
                if( box_to_check.size.width >= box_to_check.size.height )
                    aspect_ratio = box_to_check.size.width / box_to_check.size.height;
                else
                    aspect_ratio = box_to_check.size.height / box_to_check.size.width;

                if( aspect_ratio >= ASPECT_RATIO_BOUNDARY )
                {
                 //sort usable contours by their perimeter, discard those that are too small
                    double perimeterbound = (gray_input.rows + gray_input.cols)/perimScale;
                    double totalperimeter = box_to_check.size.width * 2 + box_to_check.size.height * 2;

                    if (totalperimeter >= perimeterbound)
                    {
                        usable_boxes.push_back( box_to_check );
                        cvEllipseBox(
                                      &IplImage( input ),
                                      box_to_check,
                                      CV_RGB( 255, 0, 0 ),
                                      3,
                                      8,
                                      0
                                     );
                     } //end perimeter check
                } //end aspect ratio check
            } //end contour total check

            contours = contours->h_next;
        }
        //landmarks::Pipe pipe( cv::Point3( x, y, z ), rotation );
        //multi_pub_.publish( "landmarks", pipe.createMsg() );
        cv::imshow( "Found_boxes", input );
        cvWaitKey( 20 );
    }*/

    QUICKDEV_DECLARE_MESSAGE_CALLBACK( imagesCB, _NamedImageArrayMsg )
    {
        PRINT_INFO( "Got images callback" );
        for( auto named_image_msg = msg->images.cbegin(); named_image_msg != msg->images.cend(); ++named_image_msg )
        {
            PRINT_INFO( "Current image message has name %s", named_image_msg->name.c_str() );
            // named_image_msg->name == "orange"
            if( named_image_msg->name == "orange" )
            {
                cv::namedWindow( named_image_msg->name, 0 );

                auto const cv_image_ptr = quickdev::opencv_conversion::fromImageMsg( named_image_msg->image );
                cv::Mat const & input = cv_image_ptr->image;
                cv::Mat debug_image( input.size(), CV_8UC3 );
                IplImage debug_image_ipl( debug_image );

                PRINT_INFO( "Got %s image %ix%i", named_image_msg->name.c_str(), input.size().width, input.size().height );

                cv::imshow( named_image_msg->name, input );

                const float ASPECT_RATIO_BOUNDARY = 1.5;
                const float perimScale = 10;
                //cv::Mat input = image_ptr->image;
                cv::Mat gray_input;
                std::vector<CvBox2D> usable_boxes;
                CvMemStorage* storage;
                CvSeq* contours;
                CvBox2D box_to_check;
                storage = cvCreateMemStorage(0);
                contours = cvCreateSeq(
                                        CV_SEQ_ELTYPE_POINT,
                                        sizeof( CvSeq ),
                                        sizeof( CvPoint ),
                                        storage
                                      );
                //make 8uc1 Mat
                //cvtColor(input, gray_input, CV_RGB2GRAY);
                threshold(input, gray_input, 127, 255, CV_THRESH_BINARY);


                cv::Mat elem = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5));
                cv::morphologyEx(gray_input, gray_input, cv::MORPH_OPEN, elem, cv::Point(-1,-1), 3);

                IplImage gray_input_ipl( gray_input );
/*
                //begin contour finder
                cvFindContours(
                                &gray_input_ipl,
                                storage, &contours,
                                sizeof( CvContour ),
                                CV_RETR_LIST,
                                CV_CHAIN_APPROX_SIMPLE
                               );

                //iterate through found contours
                while(contours != NULL)
                {
                    float aspect_ratio = 0;
                    if( contours->total >= 6 )
                    {
                        //make cvBox2D from potentialy usable contours, and find actual usable ones by their aspect ratios
                        box_to_check = cvFitEllipse2( contours );
                        if( box_to_check.size.width >= box_to_check.size.height )
                            aspect_ratio = box_to_check.size.width / box_to_check.size.height;
                        else
                            aspect_ratio = box_to_check.size.height / box_to_check.size.width;

                        if( aspect_ratio >= ASPECT_RATIO_BOUNDARY )
                        {
                         //sort usable contours by their perimeter, discard those that are too small
                            double perimeterbound = (gray_input.rows + gray_input.cols)/perimScale;
                            double totalperimeter = box_to_check.size.width * 2 + box_to_check.size.height * 2;

                            if (totalperimeter >= perimeterbound)
                            {
                                usable_boxes.push_back( box_to_check );
                                cvEllipseBox(
                                              &debug_image_ipl,
                                              box_to_check,
                                              CV_RGB( 255, 0, 0 ),
                                              3,
                                              8,
                                              0
                                             );
                             } //end perimeter check
                        } //end aspect ratio check
                    } //end contour total check

                    contours = contours->h_next;
                }
*/
                cv::imshow( "Found_boxes", gray_input );
            } //end if orange
        }

        cvWaitKey( 20 );
    }

    QUICKDEV_SPIN_ONCE()
    {
        //
    }
};

#endif // PIPEFINDER_PIPEFINDERNODE_H_
