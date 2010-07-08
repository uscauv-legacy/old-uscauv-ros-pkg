// Use the image_transport classes instead.
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <tf/tf.h>
#include <opencv/cv.h>
#include <color_segmenter/SegmentImage.h>
#include <buoy_finder/BuoyPos.h>

using namespace cv;

ros::ServiceClient seg_img_srv;
double heading_corr_scale, depth_corr_scale, distance_scale;

bool buoyPosCallback(buoy_finder::BuoyPos::Request & req, buoy_finder::BuoyPos::Response & resp)
{
  color_segmenter::SegmentImage seg_img;
  seg_img.request.DesiredColor = req.DesiredColor;

  if(seg_img_srv.call(seg_img))
    {
      color_segmenter::ColorBlob biggestBlob = seg_img.response.BlobArray.ColorBlobs[0];
      float bX = biggestBlob.X;
      float bY = biggestBlob.Y;
      float bArea = biggestBlob.Area;

      resp.RelativeHeading = bX*heading_corr_scale;
      resp.RelativeDepth = bY*depth_corr_scale;
      resp.Distance = distance_scale / bArea;

      return true;
    }
  else
    return false;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "buoy_finder");
  ros::NodeHandle n;

  n.param("heading_corr_scale", heading_corr_scale, 125.0);
  n.param("depth_corr_scale", depth_corr_scale, 50.0);
  n.param("distance_scale", distance_scale, 7000.0);

  ros::ServiceServer buoy_pos_srv = n.advertiseService("buoy_pos", buoyPosCallback);

  seg_img_srv = n.serviceClient<color_segmenter::SegmentImage>("color_segmenter/segmentImage");
 
  ros::spin();
 
  return 0;
}
