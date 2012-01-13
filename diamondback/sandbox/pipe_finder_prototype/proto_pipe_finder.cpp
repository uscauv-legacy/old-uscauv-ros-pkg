/*
-expect/binarize image as needed
-using same contour finding logic
-Filter contours and store all usable blobs
	-consider using area instead of perimeter
-fit ellipse on all usable blobs
	-returns Box2D type (has width, height, orientation)
-calculate blobs' aspect ratio (smaller length/larger length or
	vice versa)
	-make sure aspect ratios are within certain range
	-if ratios not within range, discard
GOAL: make sure we can get boxes

To be done at a later time:
-calculate color percentages (must be >n% orange)
	-by orange, mean white or black, depending on how
	image is binarized
*/

#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
//#include <pipe_finder/pipe_finder.h>

using namespace cv;

bool validateAspectRatio( CvSeq* contour, CvBox2D &box )
{
	if( contour->total < 6 )
		return false;
	box = cvFitEllipse2( contour );
	float aspect_ratio; 
	float aspect_ratio_boundary = 1.5;
	
	if( box.size.width >= box.size.height )
		aspect_ratio = box.size.width / box.size.height;
	else
		aspect_ratio = box.size.height / box.size.width;	
		
	if( aspect_ratio >= aspect_ratio_boundary )
		return true;
	else
		return false;
}

int main( int argc, char* argv[] )
{
	if( argc != 2 )
	{
		std::cerr << "Usage: " << argv[0] << " FILENAME" << std::endl;
		return -1;
	}
cv::Mat input = imread(argv[1]);
cv::Mat gray_input;
std::vector<CvBox2D> usable_boxes;
CvMemStorage* storage;
CvSeq* contours;
CvBox2D box_to_check;
storage = cvCreateMemStorage(0);
contours = cvCreateSeq(CV_SEQ_ELTYPE_POINT, sizeof(CvSeq), sizeof(CvPoint) , storage);

cvtColor(input, gray_input, CV_RGB2GRAY);
cvFindContours( &IplImage( gray_input ), storage, &contours, sizeof( CvContour ), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE );	
while(contours != NULL)
{
	//check if the aspect ratio is within range
	if(validateAspectRatio(contours, box_to_check))
	{
		usable_boxes.push_back(box_to_check);
	}
	contours = contours->h_next;
}
std::cout << "Number of usable boxes found in image: " << usable_boxes.size() << std::endl;

//Draw the boxes to check
for( std::vector<CvBox2D>::iterator b = usable_boxes.begin(); b != usable_boxes.end(); b++)
	cvEllipseBox( &IplImage( input ), *b, CV_RGB( 255, 0, 0 ), 1, 8, 0 );
imshow("image", input);
waitKey();
}

