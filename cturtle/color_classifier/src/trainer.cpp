/*******************************************************************************
 *
 *      color_classifier_trainer
 * 
 *      Copyright (c) 2010, edward
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

// for BaseImageProc
#include <base_image_proc/base_image_proc.h>
// for cfg code
#include <color_classifier/TrainerConfig.h>
#include <color_defs/colors.h>
#include <libvml/vml.h>
#include <sstream>

#include <boost/thread.hpp>
#include <boost/format.hpp>

#ifdef HAVE_GTK
#include <gtk/gtk.h>

// Platform-specific workaround for #3026: image_view doesn't close when
// closing image window. On platforms using GTK+ we connect this to the
// window's "destroy" event so that image_view exits.
static void destroy(GtkWidget *widget, gpointer data)
{
	ros::shutdown();
}
#endif

typedef color_classifier::TrainerConfig _ReconfigureType;
typedef double _InputDataType;
typedef double _OutputDataType;
typedef vml::ArtificialNeuralNetwork<_InputDataType, _OutputDataType> _ANN;

class ColorClassifierTrainer: public BaseImageProc<_ReconfigureType>
{
private:
	std::string window_name_;
	int training_color_;
	bool train_ann_;
	double min_error_;
	int max_iterations_;

	std::vector<_ANN::_InputVector> inputs_;
	std::vector<_ANN::_OutputVector> outputs_;

	vml::_Dimension dim_;
	_ANN ann_;
	cv::Mat hls_img_;

public:
	ColorClassifierTrainer( ros::NodeHandle & nh ) :
		BaseImageProc<_ReconfigureType> ( nh )
	{
		initCfgParams();

		nh_priv_.param( "min_error", min_error_, 0.05 );
		nh_priv_.param( "max_iterations", max_iterations_, 500000 );

		int num_layers;
		nh_priv_.param( "num_layers", num_layers, 0 );
		ROS_INFO( "num_layers %d", num_layers );

		if ( num_layers > 0 )
		{
			dim_.resize( num_layers );
			for ( int i = 0; i < num_layers; i++ )
			{
				std::stringstream ss;
				ss << "layer" << i << "_size";
				int value;
				nh_priv_.param( ss.str(), value, 0 );
				dim_[i] = value;
				ROS_INFO( "%s : %d", ss.str().c_str(), value );
			}

			ann_ = _ANN( dim_, 0.75 );
		}

		window_name_ = "Trainer for Color Classifier";
		const char* name = window_name_.c_str();
		cvNamedWindow( name, CV_WINDOW_AUTOSIZE );
		cvSetMouseCallback( name, &ColorClassifierTrainer::mouseCB, this );
#ifdef HAVE_GTK
		GtkWidget *widget = GTK_WIDGET( cvGetWindowHandle( name ) );
		g_signal_connect( widget, "destroy", G_CALLBACK( destroy ), NULL);
#endif
		cvStartWindowThread();
	}

	cv::Mat processImage( IplImage * ipl_img )
	{
		ROS_INFO( "Processed image" );

		if ( ipl_img ) cvShowImage( window_name_.c_str(), ipl_img );
		else ROS_ERROR( "Unable to convert %s image to bgr8", ipl_img->colorModel );

		cv_img_ = cv::Mat( ipl_img );
		cv::cvtColor( cv_img_, hls_img_, CV_BGR2HLS);
		return cv_img_;
	}

	static void mouseCB( int event, int x, int y, int flags, void* param )
	{
		if ( event != CV_EVENT_LBUTTONDOWN ) return;

		ColorClassifierTrainer *trainer = (ColorClassifierTrainer*) param;

		const cv::Mat & image_mat = trainer->hls_img_;
		const cv::Vec3b & pixel = image_mat.at<cv::Vec3b> ( cv::Point( x, y ) );

		printf( "training color %d\n", trainer->training_color_ );
		printf( "pixel at %d %d is %d %d %d\n", x, y, pixel[0], pixel[1], pixel[2] );

		_ANN::_InputVector input = trainer->ann_.getInputVector();
		input[0] = (_InputDataType) pixel[0];
		input[1] = (_InputDataType) pixel[1];
		input[2] = (_InputDataType) pixel[2];

		_ANN::_OutputVector output = trainer->ann_.getOutputVector();
		output[trainer->training_color_] = (_OutputDataType) 1.0;

		trainer->inputs_.push_back( input );
		trainer->outputs_.push_back( output );
	}

	void trainAnn()
	{
		img_sub_.shutdown();
		img_pub_.shutdown();

		ROS_INFO( "Training ANN on %d data pairs...", inputs_.size() );

		printf( "%s", ann_.toString().c_str() );
		ann_.initializeRandomWeights();
		printf( "%s", ann_.toString().c_str() );

		int iterations = ann_.train( inputs_, outputs_, min_error_, max_iterations_ );

		vml::AnnParser::saveAnnToFile( ann_, "/home/edward/workspace/seabee3-ros-pkg/libvml/docs/ann_dump.txt" );
	}

	void reconfigureCB( _ReconfigureType &config, uint32_t level )
	{
		training_color_ = config.training_color;

		if ( !train_ann_ && config.train_ann ) trainAnn();

		train_ann_ = config.train_ann;
	}
};

int main( int argc, char ** argv )
{
	ros::init( argc, argv, "color_classifier_trainer" );
	ros::NodeHandle nh;

	ColorClassifierTrainer trainer( nh );
	trainer.spin();

	return 0;
}
