#include "source/inc/rgb_grabber.h"

RGBGrabber::RGBGrabber(CVMatBuffer& rgb_buffer, CVMatBuffer& prop_buffer, SMP& arm)
	: rgb_buffer_(rgb_buffer)
	, prop_buffer_(prop_buffer)
	, arm_(arm)
{
	stop_ = false;
	image_width_ = 640;
	image_height_ = 480;	
	prev_tick_ = 0;
	curr_tick_ = 0;
	tick_count_ = 0;
	num_joints_ = 2;
	joint_idx_ = cv::Mat::zeros(num_joints_, 1, CV_32F);
	joint_idx_.at<float>(0, 0) = 3; joint_idx_.at<float>(1, 0) = 5; // joint_idx_.at<float>(0, 2) = 5; 	
	curr_rgb_img_ = cv::Mat::zeros(image_height_, image_width_, CV_8UC3);

	rgb_data_size_ = image_width_ * image_height_;
	rgb_data_ = new unsigned char[rgb_data_size_ * 3];

	QueryPerformanceFrequency(&frequency_);
	thread_.reset(new boost::thread(boost::bind(&RGBGrabber::ReadKinectImage, this)));
}

void RGBGrabber::RGBGrabberCallBack(const boost::shared_ptr<openni_wrapper::Image>& image)
{
	UpdateGUI();

	// update image buffer
	cv::Mat curr_grey_img;
	{
		boost::mutex::scoped_lock lock(mutex_);
		if(stop_)
			return;
	}

	image->fillRGB(image_width_, image_height_, rgb_data_);
	for(int i = 0; i < image_height_; i++)
	{
		for(int j = 0; j < image_width_; j++)
		{
			curr_rgb_img_.at<cv::Vec3b>(i, j)[0] = rgb_data_[(i * image_width_ + j) * 3 + 0]; // B
			curr_rgb_img_.at<cv::Vec3b>(i, j)[1] = rgb_data_[(i * image_width_ + j) * 3 + 1]; // G
			curr_rgb_img_.at<cv::Vec3b>(i, j)[2] = rgb_data_[(i * image_width_ + j) * 3 + 2]; // R
		}
	}
	cv::cvtColor(curr_rgb_img_, curr_grey_img, CV_BGR2GRAY);
	rgb_buffer_.PushBack(curr_grey_img);
	// update proprioception buffer
	cv::Mat curr_prop = cv::Mat::zeros(1, 2 * num_joints_, CV_32F);
	UpdateCurrProp(curr_prop);
	prop_buffer_.PushBack(curr_prop);
}

void RGBGrabber::UpdateGUI()
{
	if(prev_tick_ == 0 && curr_tick_ == 0)
	{
		boost::mutex::scoped_lock buff_lock (mutex_);
		QueryPerformanceCounter(&tick_query_);
		curr_tick_ = tick_query_.QuadPart;
		prev_tick_ = curr_tick_;		
	}
	else
	{
		boost::mutex::scoped_lock buff_lock (mutex_);
		QueryPerformanceCounter(&tick_query_);
		curr_tick_ = tick_query_.QuadPart;
		tick_count_++;	
		double time_in_seconds = (curr_tick_ - prev_tick_) * 1.0 / frequency_.QuadPart;
		if(time_in_seconds > 0.5)
		{
			boost_qt_.UpdateFrameRate(double(tick_count_) / time_in_seconds);	
			boost_qt_.UpdateTargetCount(arm_.GetTargetCount(3));
			tick_count_ = 0;
			prev_tick_ = curr_tick_;
		}
	}
}

void RGBGrabber::ReadKinectImage()
{
	// typical openni grabber routine
	pcl::Grabber* rgb_grabber = new pcl::OpenNIGrabber ();
	boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&)> f = boost::bind(&RGBGrabber::RGBGrabberCallBack, this, _1);
	rgb_grabber->registerCallback(f);
	rgb_grabber->start();
	while (true)
	{
		if(stop_)				
			break;			
		boost::this_thread::sleep (boost::posix_time::milliseconds(100));
	}
	rgb_grabber->stop ();
}

void RGBGrabber::UpdateCurrProp(cv::Mat& prop)
{
	boost::mutex::scoped_lock buff_lock(mutex_);
	for(int idx = 0; idx < num_joints_; idx++)
	{
		prop.at<float>(0, 2 * idx) = arm_.module[int(joint_idx_.at<float>(idx, 0))].position; //arm->module[7].position;
		prop.at<float>(0, 2 * idx + 1) = arm_.module[int(joint_idx_.at<float>(idx, 0))].velocity; // arm->module[7].velocity;
	}
}

void RGBGrabber::WaitForThreadTermination()
{
	// wait for the thread to terminate
	thread_->join ();      
}
