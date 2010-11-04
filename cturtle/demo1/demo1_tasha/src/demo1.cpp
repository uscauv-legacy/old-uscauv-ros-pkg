#include <base_image_proc/base_image_proc.h>

class Demo1 : public BaseImageProc<base_image_proc::EmptyConfig>
{
	Demo1(ros::NodeHandle & nh) : BaseImageProc<base_image_proc::EmptyConfig>(nh)
	{
		//
	}

	virtual cv::Mat processImage(IplImage * ipl_img)
	{
		cv_img_ = cv::Mat(ipl_img);

		//fill in w modifications to image		

		return cv_img_;
	}
};

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "demo1_tasha");
	ros::NodeHandle nh;

	Demo1 demo1(nh);
	demo1.spin();

	//use rosmake to build

	return 0;
}
	
