#include "source/inc/recorder.h"


Recorder::Recorder(
	PCDBuffer<pcl::PointXYZ> &pcd_buffer
	, CVMatBuffer& rgb_buffer
	, CVMatBuffer& prop_buffer
	, SMP& arm
	, cv::Mat& proprioception
	, int pcd_flag)
    : pcd_buffer_(pcd_buffer)
	, rgb_buffer_(rgb_buffer)
	, prop_buffer_(prop_buffer)
	, arm_(arm)
	, proprioception_(proprioception)
{
	stop_ = false;
	count_ = 1;
	iter_ = 0;
	max_iteration_length_ = 1e5;			
	pcd_flag_ = pcd_flag;		
	thread_.reset(new boost::thread(boost::bind (&Recorder::ReceiveData, this)));
		
}

void Recorder::RecordPCD(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, cv::Mat& curr_prop)
{		
	if(count_ % 3 == 0 && iter_ < max_iteration_length_)
	{
		std::stringstream ss;			
		ss << "pcd/" << iter_ << ".pcd";
		writer_.writeBinaryCompressed(ss.str (), *cloud);
		curr_prop.copyTo(proprioception_.rowRange(iter_, iter_ + 1));
		count_ = 1;
		iter_++;			
	}
	else if(iter_ < max_iteration_length_)
	{			
		count_++;
	}
}

void Recorder::RecordRGB(cv::Mat& curr_img, cv::Mat& curr_prop)
{		
	if(count_ % 3 == 0 && iter_ < max_iteration_length_)
	{
		std::stringstream ss;			
		ss << "images/" << iter_ << ".pgm";
		cv::imwrite(ss.str(), curr_img);			
		curr_prop.copyTo(proprioception_.rowRange(iter_, iter_ + 1));
		count_ = 1;
		iter_++;			
	}
	else if(iter_ < max_iteration_length_)
	{			
		count_++;
	}
}

void Recorder::ReceiveData()
{
	while (true)
	{
		if(stop_)				
			break;	
		if(pcd_flag_)
			RecordPCD(pcd_buffer_.GetFront (), prop_buffer_.GetFront());
		else
			RecordRGB(rgb_buffer_.GetFront (), prop_buffer_.GetFront());
	}

	if(pcd_flag_)
	{
		while(!pcd_buffer_.empty ())
			RecordPCD(pcd_buffer_.GetFront (), prop_buffer_.GetFront());
	}
	else
	{
		while(!pcd_buffer_.empty ())
			RecordRGB(rgb_buffer_.GetFront (), prop_buffer_.GetFront());
	}
}

void Recorder::WaitForThreadTermination()
{
	thread_->join ();
}
