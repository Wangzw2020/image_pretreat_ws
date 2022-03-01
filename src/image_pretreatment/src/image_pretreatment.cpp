#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include "opencv2/imgproc/detail/distortion_model.hpp"

#include "defog/dehaze.h"

#define _NODE_NAME_ "image_pretreatment"

using namespace cv;
using namespace std;

class ImagePretreatment
{
private:
	ros::Publisher pretreat_image_pub_;
	ros::Subscriber image_sub_;
	std::string image_topic_;
	bool image_ok_, image_received_;
	int mode_;
	//0 fog		1 rain		2 snow
	
	cv_bridge::CvImagePtr cv_ptr_;
	ros::Timer timer_;
public:
	ImagePretreatment();
	~ImagePretreatment();
	bool init();
	void loadimage(const sensor_msgs::ImageConstPtr& msg);
	void removefog(const ros::TimerEvent&);
	void removerain(const ros::TimerEvent&);
	void removesnow(const ros::TimerEvent&);
	
};

ImagePretreatment::ImagePretreatment()
{
	
}

ImagePretreatment::~ImagePretreatment()
{

}

bool ImagePretreatment::init()
{
	ros::NodeHandle nh, nh_private("~");
	nh_private.param<std::string>("image_topic", image_topic_, "");
	nh_private.param<int>("mode",mode_,0);
	
	cout << "mode: " << mode_ << endl;
	
	image_ok_ = false;
	image_received_ = false;
	
	pretreat_image_pub_ = nh.advertise<sensor_msgs::Image>("/image_treatment", 1);
	image_sub_ = nh.subscribe(image_topic_, 1, &ImagePretreatment::loadimage, this);
	
	if(mode_ == 0)
		timer_ = nh.createTimer(ros::Duration(0.01), &ImagePretreatment::removefog, this);
	else if(mode_ == 1)
		timer_ = nh.createTimer(ros::Duration(0.01), &ImagePretreatment::removerain, this);
	else if(mode_ == 2)
		timer_ = nh.createTimer(ros::Duration(0.01), &ImagePretreatment::removesnow, this);
	else
		ROS_ERROR("none mode is starting!");
	ROS_INFO("image_pretreatment initial ok.");
}

void ImagePretreatment::loadimage(const sensor_msgs::ImageConstPtr& msg)
{
	if(image_received_ == false)
		ROS_INFO("[%s]: getting image!",_NODE_NAME_);
	cv_bridge::CvImagePtr cv;
	cv = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	cv_ptr_ = cv;
	image_received_ = true;
	image_ok_ = true;
}

void ImagePretreatment::removefog(const ros::TimerEvent&)
{
	if (image_received_ == false)
	{
		ROS_ERROR("[%s]: no image input!",_NODE_NAME_);
		return;
	}
	else if(image_ok_ == false)
	{
		return;
	}
	else
		ROS_INFO("[%s]: image remove fog start!",_NODE_NAME_);
	static int width, height;
	width = cv_ptr_->image.cols;
	height = cv_ptr_->image.rows;
	cv::Mat image(height, width, CV_8UC3);
	image.setTo(0);
	cv_ptr_->image.copyTo(image(Rect(0, 0, width, height)));
	
	//
	const int filterRadius = 7;
	const double t0 = 0.1;
	const double omega = 0.95;
	const double eps = 10E-6;
	DeHaze deHaze(filterRadius,t0,omega,eps);
	
	cv::Mat dehaze_image;
	dehaze_image = deHaze.imageHazeRemove(image);
	//deHazeByNonLocalMethod(np, image, dehaze_image);
	
	//
	sensor_msgs::ImagePtr imageMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dehaze_image).toImageMsg();
	imageMsg->header.frame_id = std::string("pretreatment image");
	imageMsg->header.stamp = ros::Time::now();
	pretreat_image_pub_.publish(imageMsg);
	ROS_INFO("[%s]: image remove fog done!",_NODE_NAME_);
	image_ok_ = false;
}

void ImagePretreatment::removerain(const ros::TimerEvent&)
{

}

void ImagePretreatment::removesnow(const ros::TimerEvent&)
{

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, _NODE_NAME_);
	ImagePretreatment pretreatment;
	pretreatment.init();
	ros::spin();
	return 0;
}
















