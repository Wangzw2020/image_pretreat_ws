
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/detail/distortion_model.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <ros/ros.h>
#include <string>

#define _NODE_NAME_ "video_capture_node"

using namespace std;
using namespace cv;

class ImageTalker
{
private:
	ros::Publisher video_image_pub_;
	
	string video_file_path_;
	cv::Size size_;
	double fps_;
	long num_frame_;
	int flag_;

public:
	ImageTalker();
	~ImageTalker();
	
	bool init();
	void run();
};

ImageTalker::ImageTalker()
{

}

ImageTalker::~ImageTalker()
{

}

bool ImageTalker::init()
{
	ros::NodeHandle nh,nh_private("~");
	
	nh_private.param<std::string>("video_file_path", video_file_path_,"");
	ROS_INFO("[%s]: video path: [%s] ",_NODE_NAME_, video_file_path_.c_str());
	if(video_file_path_.empty())
	{
		ROS_ERROR("[%s]: please set video_file_path! ",_NODE_NAME_);
		return false;
	}
	
	video_image_pub_ = nh.advertise<sensor_msgs::Image>("/video_image", 1);
	
	return true;
}

void ImageTalker::run()
{
	cv::VideoCapture capture;
	capture.open(video_file_path_.c_str());
	
	if(!capture.isOpened())
	{
		ROS_ERROR("[%s]: cannot read this video file! ",_NODE_NAME_);
		return;
	}
	
	size_ = Size((int)capture.get(CAP_PROP_FRAME_WIDTH), (int)capture.get(CAP_PROP_FRAME_HEIGHT));
	fps_ = capture.get(CAP_PROP_FPS);
	num_frame_ = static_cast<long>(capture.get(CV_CAP_PROP_FRAME_COUNT));
	cout << "fps: " << fps_ <<'\n' << "size: " << size_ << '\n' << "num of frame: "<< num_frame_ << endl;
	
	cv::Mat image;
	ros::Rate loop_rate(fps_);
	while(ros::ok())
	{
		capture >> image;
		
		if(image.empty())
		{
			capture.release();
			capture.open(video_file_path_);
		}
		
		sensor_msgs::ImagePtr imageMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
		imageMsg->header.frame_id = std::string("video image");
		imageMsg->header.stamp = ros::Time::now();
		video_image_pub_.publish(imageMsg);
		ROS_INFO("[%s]: image publishing! ",_NODE_NAME_);
		loop_rate.sleep();
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, _NODE_NAME_);
	
	ImageTalker image_talker;
	image_talker.init();
	image_talker.run();
	
	return 0;
}
















