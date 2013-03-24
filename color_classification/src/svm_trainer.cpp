/***************************************************************************
 *  src/svm_trainer.cpp
 *  --------------------
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

/// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv/cxcore.h>

/// Boost filesystem
#include <boost/filesystem.hpp>

#include <iostream>

namespace _FileSys = boost::filesystem3;

typedef std::vector<std::pair<cv::Mat, cv::Mat> > _ImagePairArray;

/// TODO: Generate dedicated directory for output data
/// TODO: Check size of training image and mask

const std::string keys =
  "{    h| help          |false | Print this message.                                   }"
  "{    i| input         |false | Training image directory                              }"
  "{    c| color         |false | Name of color to classify                             }"
  "{    I| iterations    |10000 | Iterations for training                               }"
  "{    o| output        |.     | Directory for output data (not supported)             }"
  "{    e| error-penalty |0.1   | SVM weighting                                         }"
  "{    C| comment       |false | Optional comment to be inserted into output YAML file }"
  ;

int main(int argc, const char ** argv)
{
  std::cout << "USC AUV SVM color classifier trainer" << std::endl;
  
  cv::CommandLineParser parser( argc, argv, keys.c_str() );

  if ( parser.get<bool>("help") || parser.get<std::string>("input") == "false" || parser.get<std::string>("color") == "false" )
    {
      std::cout << "usage: " << argv[0]  << " --input=\"training_path\" --color=\"color_name\"" << std::endl;
      parser.printParams();
      return 0;
    }
    
  const std::string image_path  = parser.get<std::string>("input");
  const std::string color_name  = parser.get<std::string>("color");
  const std::string output_path = parser.get<std::string>("output");
  const std::string comment     = parser.get<std::string>("comment");
  const double iterations       = parser.get<float>("iterations");
  const double error_penalty    = parser.get<float>("error-penalty");
  
  _ImagePairArray input_images;

  /// Used later on when we write the names of all of the images we used to file
  std::vector<std::pair<std::string, std::string> > path_str;

  /// Traverse image directory and attempt to load images ------------------------------------

  _FileSys::path image_dir( image_path );

  try
    {
      /// If the path given does not exist or is not a directory, exit
      if ( !_FileSys::exists( image_dir ) || !_FileSys::is_directory( image_dir) )
	{
	  std::cout << "Invalid training image directory. [ " << image_dir << " ]\n";
	  return 0;
	}

      /// Get a vector of all of the paths in image_dir
      std::vector<_FileSys::path> image_paths;
      std::copy( _FileSys::directory_iterator( image_dir ), _FileSys::directory_iterator(), std::back_inserter( image_paths ) );
      std::sort( image_paths.begin(), image_paths.end() );      
      
      /**
       * For each file in the directory, check if it is a regular file. If it is and
       * does not contain the string "mask", load it as the training file and its
       * name with "_mask" appended as the mask file.
       */
      for( std::vector<_FileSys::path>::iterator path_it = image_paths.begin(); path_it != image_paths.end(); ++path_it)
	{
	  std::string basename = _FileSys::basename( *path_it );

	  /// We will load mask images explicitly once their training data counterparts are loaded.
	  size_t mask_found = basename.find("mask");
	  if( mask_found != std::string::npos )
	    {
	      continue;
	    }

	  std::cout << "Checking file [ " << *path_it << " ] ... ";
	  
	  if( !_FileSys::is_regular_file( *path_it ) )
	    {
	      std::cout << "Not a regular file, skipping..." << std::endl;
	      continue;
	    }
	  
	  /// Check if the input filename with _mask appended exists
	  _FileSys::path mask_path = image_dir / _FileSys::path(_FileSys::basename(_FileSys::change_extension( *path_it, "" )) + "_mask" +  _FileSys::extension( *path_it ) );
	  if ( !_FileSys::exists( mask_path ) || !_FileSys::is_regular_file( mask_path ) )
	    {
	      std::cout << "Mask [ " << mask_path << " ] does not exist or is not a regular file, skipping..." << std::endl;
	      continue;
	    }
	  
	  /// Load the image file and mask ------------------------------------
	  std::string input_str = path_it->normalize().string(),
	    mask_str = mask_path.normalize().string();
	  
	  cv::Mat input, mask;
	  
	  input = cv::imread( input_str, CV_LOAD_IMAGE_COLOR );
	  if( input.data == NULL )
	    {
	      std::cout << "Failed to load image [ " << input_str << " ]. " << std::endl;
	      continue;
	    }
	  std::cout << "Training image loaded successfully. ";

	  mask = cv::imread( mask_str, CV_LOAD_IMAGE_GRAYSCALE );
	  if( mask.data == NULL )
	    {
	      std::cout << "Failed to load mask [ " << mask_str << " ]. " << std::endl;
	      continue;
	    }
	  std::cout << "Mask image loaded successfully." << std::endl;

	  input_images.push_back( std::make_pair( input, mask ) );
	  path_str.push_back ( std::make_pair( input_str, mask_str ) );
	}

    }
  catch (const _FileSys::filesystem_error &ex )
    {
      std::cout << ex.what() << std::endl;
      return 0;
    }

  std::cout << "Loaded [ " << input_images.size() << " ] training data sets." << std::endl;
  
  if ( input_images.size() == 0 )
    {
      std::cout << "Error: No images were loaded. Exiting..." << std::endl;
      return 0;
    }


  cv::Mat all_training, all_mask;

  unsigned int positive_mask_count = 0;
  
  /// Concatenate all of the input images into one giant vector of pixels for training
  for( _ImagePairArray::iterator input_it = input_images.begin(); input_it != input_images.end(); ++input_it )

    {
      cv::Mat input, mask;

      /// OpenCV SVM training requires floating point data
      input_it->first.convertTo( input, CV_32F );
      input = input.reshape( 1, input.size().height * input.size().width );
      all_training.push_back( input );

      mask = input_it->second;
      // input_it->second.convertTo( mask, CV_32FC1 );
      // mask = mask.reshape( 1, input.size().height * input.size().width );

      /// TODO: Convert data without explicitly using this for loop
      for(cv::MatConstIterator_<unsigned char> mask_it = mask.begin<unsigned char>(); mask_it != mask.end<unsigned char>(); ++mask_it)
	{
	  if ( *mask_it == 255 )
	    {
	      /// SVM only accepts CV_32FS, so we must explicitly add elements as floats using the trailing f.
	      all_mask.push_back( 1.0f );
	      ++positive_mask_count;
	    }
	  else 
	    {
	      all_mask.push_back( -1.0f );
	    }
	}
    }
  
  std::cout << "Training data size: " << "( " << all_training.size().height << ", " << all_training.size().width << " ), Depth: "
	    << all_training.depth() << ", Channels: " << all_training.channels() << std::endl;
  std::cout << "Mask data size: " << "( " << all_mask.size().height << ", " << all_mask.size().width << " ), Depth: "
	    << all_mask.depth() << ", Channels: " << all_mask.channels() << std::endl;

  std::cout << "Positive mask contains " << positive_mask_count << " pixels." << std::endl;

  std::cout << "Training SVM..." << std::endl;
  std::cout << "Max iterations: " << (int)iterations << ", Error penalty: " << error_penalty << std::endl;
  
    /// Initialize SVM Params ------------------------------------
  cv::SVMParams svm_params;
  
  svm_params.svm_type = cv::SVM::C_SVC;
  svm_params.C = error_penalty;
  svm_params.kernel_type = cv::SVM::LINEAR;
  /// criteria for svm training to complete
  /// TODO: figure out what these parameters are, and what their counterparts in that output yaml correspond to
  svm_params.term_crit = cv::TermCriteria( CV_TERMCRIT_ITER, (int)iterations, 1e-6f );

  time_t before_train, after_train;
  double seconds;

  time( &before_train );
  
  /// Train the SVM ------------------------------------
  cv::SVM SVM;

  /// This function expects response data to be CV_32FC1 or CV_32SC1
  SVM.train( all_training, all_mask, cv::Mat(), cv::Mat(), svm_params );

  time( &after_train );
  seconds = difftime( after_train, before_train );

  std::cout << "Training finished in [ " << seconds << " ] seconds." << std::endl;

  /// Run classification on the training images as a sanity check ------------------------------------

  std::cout << "Classifying training images..." << std::endl;

  cv::Vec3b const white( 255, 255, 255), black( 0, 0, 0 );

  std::cout << "Wrote image: ";
  
  unsigned int image_count = 1;
  for( _ImagePairArray::iterator input_it = input_images.begin(); input_it != input_images.end(); ++input_it, ++image_count )
    {
      std::stringstream image_name;
      image_name << color_name << image_count << ".png";

      cv::Mat & input = input_it->first, input_float, prediction, output;

      input.convertTo( input_float, CV_32F );
      
      /// Classify the image that we used to train
      prediction = cv::Mat( input.size(), CV_8UC3 );
  
      cv::MatIterator_<cv::Vec3b> img_it = prediction.begin<cv::Vec3b>();
      cv::MatConstIterator_<cv::Vec3f> data_it = input_float.begin<cv::Vec3f>();
  
      for(; data_it != input_float.end<cv::Vec3f>() ; ++data_it, ++img_it )
	{
	  float response = SVM.predict( cv::Mat(*data_it) );
      
	  if (response == 1.0)
	    *img_it = white;
	  else if (response == -1.0)
	    *img_it = black;
	  else
	    std::cout << "Warning: Response was: " << response << std::endl;
	}
      
      /// Stick the training image and its classified counterpart side-by-side and write to file
      cv::hconcat( input, prediction, output );
      
      cv::imwrite( image_name.str() , output );
      
      // std::cout << "[ " << image_name.str() << " ]" << (( (data_it+1) == input_float.end<cv::Vec3f>() ) ? "" : ", ");
      std::cout << "[ " << image_name.str() << " ], ";
    }

  std::cout <<std::endl;

  std::stringstream svm_path;
  svm_path << color_name << ".yaml";
  

  std::cout << "Writing svm parameters to file... [ " << svm_path.str() << " ]" << std::endl;
  
  CvFileStorage * svm_storage = cvOpenFileStorage( svm_path.str().c_str(), 0, CV_STORAGE_WRITE );

  time_t rawtime;
  struct tm * timeinfo;
  time ( &rawtime );
  timeinfo = localtime ( &rawtime );

  std::stringstream date_string;
  date_string << "Generated: " << asctime( timeinfo );

  /// Write date string
  cvWriteComment( svm_storage, date_string.str().c_str(), 0 );

  /// Write the image paths
  std::stringstream training_comment, mask_comment;
  training_comment << "Training images: ";
  mask_comment << "Mask images: ";

  for(std::vector<std::pair<std::string, std::string> >::const_iterator path_it = path_str.begin(); path_it != path_str.end(); ++path_it)
    {
      training_comment << "[ " << path_it->first << " ], ";
      mask_comment << "[ " << path_it->second << " ], ";
    }
    
  cvWriteComment( svm_storage, training_comment.str().c_str(), 0);
  cvWriteComment( svm_storage, mask_comment.str().c_str(), 0);

  /// newline
  cvWriteComment( svm_storage, "", 0);
  
  /// Write optional user comment to file
  if ( comment != "false" )
    cvWriteComment( svm_storage, comment.c_str(), 0);

  /// The second SVM.write arg sets the name of the top level node in the yaml file (i.e. green, orange, etc.)
  SVM.write(svm_storage, color_name.c_str() );

  
  cvReleaseFileStorage( &svm_storage );
  std::cout << "Write success." << std::endl;
  
  return 0;
}
