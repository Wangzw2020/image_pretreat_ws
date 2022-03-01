#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/detail/distortion_model.hpp>

#include <ros/ros.h>
#include <string>

#include "defog/dehaze.h"

using namespace std;
using namespace cv;

string video_file_path = "/home/wzw/Videos/haze_3/haze.mp4";
string save_path = "/home/wzw/Videos/haze_pretreat_new.mp4";

class videoPretreat
{
private:
	
	string video_file_path_, save_path_;
	cv::Size size_;
	double fps_;
	long num_frame_;
	int flag_;
	int fourcc_;

public:
	videoPretreat();
	~videoPretreat();
	
	void setVideoPath(string path); 
	void setSavePath(string path);
	void run();
};

videoPretreat::videoPretreat()
{

}

videoPretreat::~videoPretreat()
{

}

void videoPretreat::setVideoPath(string path)
{
	video_file_path_ = path;
}

void videoPretreat::setSavePath(string path)
{
	save_path_ = path;
}

void videoPretreat::run()
{
	cv::VideoCapture capture;
	capture.open(video_file_path_.c_str());
	
	if(!capture.isOpened())
	{
		cout << "open video file failed! " << endl;
		return;
	}
	
	size_ = Size((int)capture.get(CAP_PROP_FRAME_WIDTH), (int)capture.get(CAP_PROP_FRAME_HEIGHT));
	fps_ = capture.get(CAP_PROP_FPS);
	num_frame_ = static_cast<long>(capture.get(CV_CAP_PROP_FRAME_COUNT));
	fourcc_ = capture.get(CAP_PROP_FOURCC);
	cout << "fps: " << fps_ <<'\n' 
		 << "size: " << size_ << '\n' 
		 << "num of frame: "<< num_frame_ << '\n' 
		 << "fourcc: " << fourcc_ << endl;
	
	cv::VideoWriter writer;
	writer.open(save_path_, fourcc_, fps_, size_, true);
	
	cv::Mat image, dehaze_image;
	const int filterRadius = 7;
	const double t0 = 0.1;
	const double omega = 0.95;
	const double eps = 10E-6;
	DeHaze deHaze(filterRadius,t0,omega,eps);
	
	while(1)
	{
		capture >> image;
		if(image.empty())
		{
			cout << "video done! " << endl;
			break;
		}
		
		dehaze_image = deHaze.imageHazeRemove(image);

		writer.write(dehaze_image);
		
	}
	capture.release();
	writer.release();
}

int main(int argc, char** argv)
{
	videoPretreat vp;
	vp.setVideoPath(video_file_path);
	vp.setSavePath(save_path);
	vp.run();
	
	return 0;
}
















