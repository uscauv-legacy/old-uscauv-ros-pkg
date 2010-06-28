#include <segmentimage/segmentImage2.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

#include <color_segmenter/SegmentImage.h>
#include <color_segmenter/ColorRange.h>
#include <color_segmenter/ColorHSV.h>
#include <color_segmenter/ColorBlob.h>
#include <color_segmenter/ColorBlobArray.h>

#define IMG_WIDTH  320
#define IMG_HEIGHT 240

segmentImage2 * segmenter;
ros::ServiceServer * SegmentImage_srv;

void runSegmentImage(const ImageConstPtr & image, const color_segmenter::ColorHSV & colorHSV, const float blobMassThreshold, color_segmenter::ColorBlobArray & colorBlobArray)
{
	segmenter.SIsetHue(colorHSV.H.Value, colorHSV.H.Threshold, colorHSV.H.Skew);
	segmenter.SIsetSat(colorHSV.S.Value, colorHSV.S.Threshold, colorHSV.S.Skew);
	segmenter.SIsetVal(colorHSV.V.Value, colorHSV.V.Threshold, colorHSV.V.Skew);
	/*
	 SIsetXXX(double center value, double thresh = how much data can deviate from center,
	 double skew(+- indicates side of skew, magnitude is amount to extend bounds on that side))
	 */
	//uncommented values are values that work well for footage
	//itsBuoySegmenter.SIsetHue(320, 85, 0);
	//itsBuoySegmenter.SIsetSat(55, 45, 0);
	//itsBuoySegmenter.SIsetVal(128, 128, 0);
	//^old values for red

	//new values for red
	//itsBuoySegmenter.SIsetHue(352, 22, 0);
	//itsBuoySegmenter.SIsetSat(55, 42, 0);
	//itsBuoySegmenter.SIsetVal(128, 128, 0);

	//itsPipeSegmenter.SIsetHue(150, 50, 0);
	//itsPipeSegmenter.SIsetSat(28, 18, 0);
	////itsPipeSegmenter.SIsetSat(34,24,0);
	//itsPipeSegmenter.SIsetVal(165.75, 38.25, 0);
	//^old values for yellow

	//new values for orange
	//itsPipeSegmenter.SIsetHue(22, 10, 0);
	//itsPipeSegmenter.SIsetSat(55, 30, 0);
	//itsPipeSegmenter.SIsetVal(128, 128, 0);

	//SIsetFrame(x1,y1,x2,y2,realX,realY) limits the consideration into the rectangle from (x1,y1) to (x2,y2)
	segmenter.SIsetFrame(0, 0, img.getWidth(), img.getHeight(), img.getWidth(), img.getHeight());

	//segmenters do the color segmenting here
	itsBuoySegmenter.SIsegment(image);

	//Candidates are pixels that fit within the accepted range that also fit in the previous frame (noise elimination)
	Image<bool> candidates = itsBuoySegmenter.SIreturnCandidates();

	///*The code between this ///* and the following //*/ can be commented out when not looking at output to improve performance.
	//work-around to get writeRGB to display something visible;
//	Image<PixRGB<byte> > segImgDisp; //Image to display to screen
//	segImgDisp.resize(buoyCand.getWidth(), buoyCand.getHeight());

	/*for (int i = 0; i < candidates.getWidth(); ++i)
	{
		for (int j = 0; j < candidates.getHeight(); ++j)
		{
			//LINFO("x=%d,y=%d",i,j);
			// if(buoyCand.getVal(i,j) and pipeCand.getVal(i,j)){
			//   segImgDisp.setVal(i,j,PixRGB<byte>(255,128,0));
			//}
			//if (buoyCand.getVal(i, j))
			//{
			//	segImgDisp.setVal(i, j, PixRGB<byte> (255, 0, 0));
			//}
			else if (pipeCand.getVal(i, j))
			{
				segImgDisp.setVal(i, j, PixRGB<byte> (255, 127, 0));
			}
			else
			{
				segImgDisp.setVal(i, j, PixRGB<byte> (0, 0, 255));
			}
		}
	}

	itsOfs->writeRGB(concatLooseX(img, segImgDisp, PixRGB<byte> (0, 0, 0)),
			"Color Segmenter Image", FrameInfo("Color Segementer", SRC_POS));*/

	//*/Code block can be commented out when not debugging output

	//sends out Retina Messages with the candidate images
	/*RobotSimEvents::RetinaMessagePtr msg =
			new RobotSimEvents::RetinaMessage;
	msg->img = Image2Ice(buoyCand);
	msg->cameraID = "BuoyColorSegmenter";

	this->publish("RetinaMessageTopic", msg);

	msg = new RobotSimEvents::RetinaMessage;
	msg->img = Image2Ice(pipeCand);
	msg->cameraID = "PipeColorSegmenter";

	this->publish("RetinaMessageTopic", msg);*/

	//blob properties calculated here
	segmenter.SIcalcMassCenter();

	//the following code puts the info about the red blobs (buoy) and yellow blobs(pipe) into a sorted array to send out as location messages
	int i = 0;
	vector<pair<float, pair<int, int> > > blobs( segmenter.SInumberBlobs() );
	
	sort(blobs.begin(), blobs.end());

	//for each blob, sends out a message with center location and size
	for (i = segmenter.SInumberBlobs() - 1; i >= 0; --i)
	{
		color_segmenter::ColorBlob colorBlob;
		colorBlob.X = float(blobs[i].second.first) / float(img.getWidth());
		colorBlob.Y = float(blobs[i].second.second) / float(img.getHeight());
		colorBlob.Mass = blobs[i].first;
		
		colorBlobArray.push_back(colorBlob);
	}
}

bool SegmentImageCallback (color_segmenter::SegmentImage::Request & req, color_segmenter::SegmentImage::Response & res)
{
	color_segmenter::ColorBlobArray blobArray;
	runSegmentImage(req.DesiredColor, req.BlobMassThreshold, blobArray);
	res.BlobArray = blobArray;
	return true;
}

int main (int argc, char** argv)
{
	ros::Init(argc, argv, "color_segmenter");
	ros::NodeHandle n;
	
	ros::ServiceServer SegmentImage_srv = n.advertiseService("/color_segmenter/SegmentImage", SegmentImageCallback);
	
	segmenter = new imageSegmenter2( 2 );
	
}
