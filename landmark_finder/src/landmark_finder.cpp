/*******************************************************************************
 *
 *      landmark_finder
 * 
 *      Copyright (c) 2010, Edward T. Kaszubski (ekaszubski@gmail.com)
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
 *      * Neither the name of the USC Underwater Robotics Team nor the names of its
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

#include <ros/ros.h>
#include <tf/tf.h>
#include <landmark_map/Landmark.h>
#include <localization_defs/LandmarkMsg.h>
#include <visualization_msgs/Marker.h>
#include <color_segmenter/FindBlobs.h>
#include <color_segmenter/ColorBlobArray.h>
#include <image_transport/image_transport.h>
#include <tf/transform_listener.h>
#include <opencv/cv.h>
#include <queue>

#include <landmark_finder/FindLandmarks.h>

ros::ServiceClient blob_finder_srv;
color_segmenter::FindBlobs blob_finder;
double heading_corr_scale, depth_corr_scale, distance_scale;
int runBuoyDemo;

//std::queue<localization_defs::LandmarkMsg> messageQueue;
std::queue<visualization_msgs::Marker> markerQueue;
tf::TransformListener * tl;

struct RectangleParams
{
	double rectangleParam1;
	double rectangleParam2;
};

namespace Demo
{
	landmark_finder::FindLandmarks::Request buoyDemoReq;
	landmark_finder::FindLandmarks::Response buoyDemoResp;
}

struct BlobParams
{
	BlobParams(double min = 0.0, double max = 99999.0, double heading_scale = 1.0, double depth_scale = 1.0, double dist_scale = 7000.0)
	{
		minBlobSize = min;
		maxBlobSize = max;
		heading_corr_scale = heading_scale;
		depth_corr_scale = depth_scale;
		distance_scale = dist_scale;
	}
	double minBlobSize;
	double maxBlobSize;
	double heading_corr_scale;
	double depth_corr_scale;
	double distance_scale;
};

BlobParams mBuoyBlobParams;
BlobParams mPipeBlobParams;
BlobParams mBinBlobParams;
//PingerParams mPingerParams;
BlobParams mWindowBlobParams;

//find the biggest blob within the defined limits or return false if there are none that meet the requirements
bool findBiggestBlob(const int & desiredColor, color_segmenter::ColorBlob & resp, const BlobParams & blobParams = BlobParams())
{
	ROS_INFO("calling color_segmenter to find all blobs with color %d", desiredColor);
	blob_finder.request.DesiredColor = desiredColor;
	if(blob_finder_srv.call(blob_finder))
	{
		ROS_INFO("found %d blobs; using the largest one", (int)blob_finder.response.BlobArray.ColorBlobs.size());
		color_segmenter::ColorBlob biggestBlob;
		
		//if the biggest blob is less than the min size threshold or if the smallest blob is greater than the max size threshold, no blobs will match; return false
		if(blob_finder.response.BlobArray.ColorBlobs[0].Area < blobParams.minBlobSize || blob_finder.response.BlobArray.ColorBlobs[blob_finder.response.BlobArray.ColorBlobs.size() - 1].Area > blobParams.maxBlobSize)
			return false;
		
		//at this point we know at least one blob will match
		for(unsigned int i = 0; i < blob_finder.response.BlobArray.ColorBlobs.size(); i ++ )
		{
			if(blob_finder.response.BlobArray.ColorBlobs[i].Area >= blobParams.minBlobSize && blob_finder.response.BlobArray.ColorBlobs[i].Area <= blobParams.maxBlobSize)
			{
				biggestBlob = blob_finder.response.BlobArray.ColorBlobs[0];
			}
		}
		
		resp.X = biggestBlob.X * blobParams.heading_corr_scale;
		resp.Y = biggestBlob.Y *= blobParams.depth_corr_scale;
		resp.Area = biggestBlob.Area = blobParams.distance_scale / biggestBlob.Area;
		
		return true;
	}
	ROS_INFO("didn't find any blobs");
	return false;
}

bool findBlobs(const int & desiredColor, color_segmenter::ColorBlobArray & resp, const BlobParams & blobParams = BlobParams())
{
	blob_finder.request.DesiredColor = desiredColor;
	if(blob_finder_srv.call(blob_finder))
	{
		//if the biggest blob is less than the min size threshold or if the smallest blob is greater than the max size threshold, no blobs will match; return false
		if(blob_finder.response.BlobArray.ColorBlobs[0].Area < blobParams.minBlobSize || blob_finder.response.BlobArray.ColorBlobs[blob_finder.response.BlobArray.ColorBlobs.size() - 1].Area > blobParams.maxBlobSize)
			return false;
		
		//at this point we know at least one blob will match
		for(unsigned int i = 0; i < blob_finder.response.BlobArray.ColorBlobs.size(); i ++ )
		{
			if(blob_finder.response.BlobArray.ColorBlobs[i].Area >= blobParams.minBlobSize && blob_finder.response.BlobArray.ColorBlobs[i].Area <= blobParams.maxBlobSize)
			{
				color_segmenter::ColorBlob theBlob = blob_finder.response.BlobArray.ColorBlobs[i];
				theBlob.X *= blobParams.heading_corr_scale;
				theBlob.Y *= blobParams.depth_corr_scale;
				theBlob.Area = blobParams.distance_scale / theBlob.Area;
				resp.ColorBlobs.push_back(theBlob);
			}
		}
		return true;
	}
	return false;
}

bool FindBuoysCallback(landmark_finder::FindLandmarks::Request & req, landmark_finder::FindLandmarks::Response & resp)
{
	ROS_INFO("looking for buoys...");
	bool atLeastOneBuoyFound = false;
	
	for(unsigned int i = 0; i < req.Ids.size(); i ++)
	{
		ROS_INFO("color: %d", req.Ids[i]);
		color_segmenter::ColorBlob blob;
		//FindLandmarks/Ids[] is just a list of unique identifiers; for bins it would be the img id; for buoys and windows it's the colors; for pingers it's the frequency id; for pipes it's not used
		bool lastResult = findBiggestBlob(req.Ids[i], blob, mBuoyBlobParams);
		atLeastOneBuoyFound = atLeastOneBuoyFound || lastResult;
		
		if(lastResult)
		{
			ROS_INFO("found a buoy");
			//blob.Area -> -x
			//blob.X    -> y
			//blob.Y    -> z
			
			tf::StampedTransform mapOffset;
			
			try
			{
				tl->waitForTransform("/landmark_map", "/seabee3/front_lt_camera", ros::Time(0), ros::Duration(5.0));
				tl->lookupTransform("/landmark_map", "/seabee3/front_lt_camera", ros::Time(0), mapOffset);
			}   
			catch( tf::TransformException &ex )
			{
				ROS_WARN( "unable to do transformation: [%s]", ex.what());
			}
			
			LandmarkTypes::Buoy theBuoy ( cv::Point3d( blob.Area, -blob.X, -blob.Y ), 0.0, req.Ids[i] );
			//LandmarkTypes::Buoy theBuoyAbs ( cv::Point3d( blob.Area + mapOffset.getOrigin().x(), blob.X + mapOffset.getOrigin().y(), blob.Y + mapOffset.getOrigin().z() ), 0.0, req.Ids[i] );
			
			resp.Landmarks.push_back( theBuoy.createMsg() ); //for the fuckin' win; I knew those localization_defs would come in handy
			
			theBuoy.mCenter.x += mapOffset.getOrigin().x();
			theBuoy.mCenter.y += mapOffset.getOrigin().y();
			theBuoy.mCenter.z += mapOffset.getOrigin().z();
			//we're assuming there's only one buoy of each color available at a given time, so we can use blob.Color as a uniqe ID
			markerQueue.push(theBuoy.createMarker( "/landmark_map", req.Ids[i], "_finder" ) ); //append "_finder" to the namespace: <type>_finder<id>
		}
		else
			ROS_INFO("no buoys of that color were found");
	}
	
	return atLeastOneBuoyFound;
}

bool FindPipesCallback(landmark_finder::FindLandmarks::Request & req, landmark_finder::FindLandmarks::Response & resp)
{
	return false;
}

bool FindBinsCallback(landmark_finder::FindLandmarks::Request & req, landmark_finder::FindLandmarks::Response & resp)
{
	return false;
}

bool FindPingersCallback(landmark_finder::FindLandmarks::Request & req, landmark_finder::FindLandmarks::Response & resp)
{
	return false;
}

bool FindWindowsCallback(landmark_finder::FindLandmarks::Request & req, landmark_finder::FindLandmarks::Response & resp)
{
	return false;
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "landmark_finder");
	ros::NodeHandle n("~");
	
	n.param("buoy_finder/run_demo", runBuoyDemo, 0);

	n.param("buoy_finder/blob_params/heading_corr_scale", mBuoyBlobParams.heading_corr_scale, 1.0);
	n.param("buoy_finder/blob_params/depth_corr_scale", mBuoyBlobParams.depth_corr_scale, 1.0);
	n.param("buoy_finder/blob_params/distance_scale", mBuoyBlobParams.distance_scale, 7000.0);

	n.param("pipe_finder/blob_params/heading_corr_scale", mPipeBlobParams.heading_corr_scale, 125.0);
	n.param("pipe_finder/blob_params/depth_corr_scale", mPipeBlobParams.depth_corr_scale, 50.0);
	n.param("pipe_finder/blob_params/distance_scale", mPipeBlobParams.distance_scale, 7000.0);

	n.param("bin_finder/blob_params/heading_corr_scale", mBinBlobParams.heading_corr_scale, 125.0);
	n.param("bin_finder/blob_params/depth_corr_scale", mBinBlobParams.depth_corr_scale, 50.0);
	n.param("bin_finder/blob_params/distance_scale", mBinBlobParams.distance_scale, 7000.0);
  
	//n.param("pinger_finder/heading_corr_scale", mPingerParams.heading_corr_scale, 125.0);
	//n.param("pinger_finder/depth_corr_scale", mPingerParams.depth_corr_scale, 50.0);
	//n.param("pinger_finder/distance_scale", mPingerParams.distance_scale, 7000.0);

	n.param("window_finder/blob_params/heading_corr_scale", mWindowBlobParams.heading_corr_scale, 125.0);
	n.param("window_finder/blob_params/depth_corr_scale", mWindowBlobParams.depth_corr_scale, 50.0);
	n.param("window_finder/blob_params/distance_scale", mWindowBlobParams.distance_scale, 7000.0);

	ros::ServiceServer buoy_finder_srv = n.advertiseService("FindBuoys", FindBuoysCallback);
	ros::ServiceServer pipe_finder_srv = n.advertiseService("FindPipes", FindPipesCallback);
	ros::ServiceServer bin_finder_srv = n.advertiseService("FindBins", FindBinsCallback);
	ros::ServiceServer pinger_finder_srv = n.advertiseService("FindPingers", FindPingersCallback);
	ros::ServiceServer window_finder_srv = n.advertiseService("FindWindows", FindWindowsCallback);

	blob_finder_srv = n.serviceClient<color_segmenter::FindBlobs>("/color_segmenter/FindBlobs");
	
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("landmarks", 1);
	
	tl = new tf::TransformListener;
	
	if(runBuoyDemo == 1)
	{
		Demo::buoyDemoReq.Ids.push_back(0); //red
		Demo::buoyDemoReq.Ids.push_back(4); //blue
	}
	
	while( ros::ok() )
	{
		if(runBuoyDemo == 1)
		{
			FindBuoysCallback(Demo::buoyDemoReq, Demo::buoyDemoResp);
		}
		//publish any new markers that may have been generated
		while( !markerQueue.empty() )
		{
			ROS_INFO("publishing marker...");
			marker_pub.publish( markerQueue.front() );
			markerQueue.pop();
		}
		ros::spinOnce();
		
		if(runBuoyDemo == 1)
			ros::Rate(3).sleep();
		else
			ros::Rate(10).sleep();
	}
 
	return 0;
}
