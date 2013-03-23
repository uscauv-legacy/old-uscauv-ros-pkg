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

int main(int argc, char ** argv)
{
  if (argc != 4)
    {
      std::cout << "usage: " << argv[0] << "training_dir output_name iterations" << std::endl;
      return 0;
    }

  _ImagePairArray input_images;

  _FileSys::path image_dir( argv[1] );

  try
    {
      /// If the path given does not exist or is not a directory, exit
      if ( !_FileSys::exists( image_dir ) || !_FileSys::is_directory( image_dir) )
	{
	  std::cout << "Invalid directory. [ " << image_dir << " ]\n";
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

  return 0;


  int iterations;
  std::stringstream( argv[3] ) >> iterations;
  std::cout << "Max iterations: " << iterations << std::endl; 
  
  cv::Mat data_image, mask_image, input_image;
  
  input_image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
  mask_image = cv::imread(argv[2], CV_LOAD_IMAGE_GRAYSCALE);

  input_image.convertTo(data_image, CV_32F );
  
  std::cout << "input type: " << data_image.type() << ", mask type: " << mask_image.type() << std::endl;
  

  /// TODO: make sure mask is binary
  /// TODO: make sure images have same dimensions
  /// TODO: Copy images into svm formatting efficiently using opencv builtins
  unsigned int mask_count = 0;

  std::vector<float> labels;
  std::vector<cv::Vec3f> data;

  cv::MatConstIterator_<cv::Vec3f> data_it = data_image.begin<cv::Vec3f>();
  for(cv::MatConstIterator_<unsigned char> mask_it = mask_image.begin<unsigned char>(); mask_it != mask_image.end<unsigned char>(); ++mask_it, ++data_it)
    {
      if ( *mask_it == 255 )
      // if ( *mask_it )
	{
	  labels.push_back( 1.0 );
	  ++mask_count;
	}
      else labels.push_back( -1.0 );
      
      // labels.push_back( ( (*mask_it > 128) ) ? 1 : -1);
      
      // std::cout << int(*mask_it) << ", ";
      // data.push_back( cv::saturate_cast<cv::Vec3f>(*data_it) );
      data.push_back( *data_it );
    }

  std::cout << "Positive mask contains " << mask_count << " pixels." << std::endl;

  std::cout << "labels size: " << labels.size() << ", data size: " << data.size() << std::endl;

  /// doesn't copy data
  cv::Mat maskMat( labels );

  // cv::namedWindow("mask converted", CV_WINDOW_AUTOSIZE);
  // cv::imshow("mask converted", maskMat.reshape(1, mask_image.size().height));
  // cv::waitKey(0);

  cv::Mat dataMat( data );

  /// Reshape from Nx1 & 3-Channel to Nx3 & 1-channel
  dataMat = dataMat.reshape(1, 0);
  
  std::cout << "maskMat type: " << maskMat.type() << ", dataMat type: " << dataMat.type() << std::endl;

  cv::Size mask_size = maskMat.size(), data_size = dataMat.size();

  std::cout << "Mask size: " << "( " << mask_size.height << ", " << mask_size.width << " ), Depth: "
	    << maskMat.depth() << ", Channels: " << maskMat.channels() << std::endl;
  std::cout << "Data size: " << "( " << data_size.height << ", " << data_size.width << " ), Depth: "
	    << dataMat.depth() << ", Channels: " << dataMat.channels() << std::endl;


  /// Initialize SVM Params ------------------------------------
  cv::SVMParams svm_params;
  
  svm_params.svm_type = cv::SVM::C_SVC;
  svm_params.C = 0.1;
  svm_params.kernel_type = cv::SVM::LINEAR;
  /// criteria for svm training to complete
  /// TODO: figure out what these parameters are, and what their counterparts in that output yaml correspond to
  svm_params.term_crit = cv::TermCriteria( CV_TERMCRIT_ITER, (int)iterations, 1e-6f );
  
  /// Train the SVM ------------------------------------
  cv::SVM SVM;
  /// TODO: Figure out what the two Mat() constructors are arguments for
  SVM.train( dataMat, maskMat, cv::Mat(), cv::Mat(), svm_params );

  std::cout << "Training finished." << std::endl;

  std::cout << "Classifying training image..." << std::endl;
					   
  /// Classify the image that we used to train
  cv::Mat predict_image = cv::Mat( data_image.size(), CV_8UC3 );
  
  cv::Vec3b orange( 0, 150, 250), grey( 50, 50, 50);

  cv::MatIterator_<cv::Vec3b> img_it = predict_image.begin<cv::Vec3b>();
  data_it = data_image.begin<cv::Vec3f>();
  
  for(; data_it != data_image.end<cv::Vec3f>() ; ++data_it, ++img_it )
    {
      float response = SVM.predict( cv::Mat(*data_it) );
      
      // std::cout << response << ", ";
      
      if (response == 1.0)
	*img_it = orange;
      else if (response == -1.0)
	*img_it = grey;
      else
	std::cout << "Warning: Response was: " << response << std::endl;
    }

  std::cout << "Classify finished." << std::endl;

  /// TODO: Make separate plot to draw decision regions
  /// draw support vectors
  // for(int i = 0 ; i < SVM.get_support_vector_count(); ++i)
  //   {
  //     const float * v = SVM.get_support_vector(i);
  //     cv::circle( predict_image, cv::Point( (int) v[0], (int) v[1]), 6,
  // 		  cv::Scalar(200, 200, 200), 2, 8);
  //   }

  /// Save classfied image demo ------------------------------------
  
  time_t rawtime;
  struct tm * timeinfo;

  time ( &rawtime );
  timeinfo = localtime ( &rawtime );

  std::stringstream date_string;
  std::string image_path, svm_path;

  date_string << timeinfo->tm_mon << "_" << timeinfo->tm_mday << "_" << timeinfo->tm_year
	      << timeinfo->tm_hour << "_" << timeinfo->tm_min  << "_" << timeinfo->tm_sec;
  
  image_path = date_string.str() + ".png";
  svm_path = date_string.str() + ".yaml";
    
  std::cout << "Writing test image to file: " << image_path << std::endl;
  cv::imwrite( image_path.c_str(), predict_image );
  std::cout << "Write success." << std::endl;

  /// Write training data to file ------------------------------------
  
  /// TODO: The second SVM.write arg should be set to the name of the top level node in the yaml file (i.e. green, orange, etc.)
  std::cout << "Writing svm parameters to file: " << svm_path << std::endl;
  
  CvFileStorage * svm_storage = cvOpenFileStorage(svm_path.c_str(), 0, CV_STORAGE_WRITE );
  SVM.write(svm_storage, "test_svm_name");
  cvReleaseFileStorage( &svm_storage );
  std::cout << "Write success." << std::endl;


  cv::namedWindow("Training Image", CV_WINDOW_AUTOSIZE);
  cv::namedWindow("Mask Image", CV_WINDOW_AUTOSIZE);
  cv::namedWindow("Classified Training Image", CV_WINDOW_AUTOSIZE);
  
  cv::imshow("Training Image", input_image);
  cv::imshow("Mask Image", mask_image);
  cv::imshow("Classified Training Image", predict_image);  
  
  cv::waitKey(0);

  return 0;
}
