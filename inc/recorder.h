#ifndef RECORDER_H
#define RECORDER_H

#include "source/inc/pcd_buffer.h"
#include "source/inc/cv_mat_buffer.h"
#include "source/inc/smp.h"
#include "opencv2/highgui/highgui.hpp"

class Recorder
{
private:
    void RecordPCD(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, cv::Mat& curr_prop);    
	void RecordRGB(cv::Mat& curr_img, cv::Mat& curr_prop);    
    // Consumer thread function
    void ReceiveData();
	 
public:
    Recorder( 
		  PCDBuffer<pcl::PointXYZ>& pcd_buffer
		, CVMatBuffer& rgb_buffer
		, CVMatBuffer& prop_buffer
		, SMP& arm
		, cv::Mat& proprioception
		, int pcd_flag);      
		
    void WaitForThreadTermination();

	inline void	set_stop(bool stop_flag)
	{
		boost::mutex::scoped_lock buff_lock(mutex_);
		stop_ = stop_flag;
	}

	inline bool	stop()
	{
		boost::mutex::scoped_lock buff_lock(mutex_);
		return stop_;
	}

private:
	PCDBuffer<pcl::PointXYZ> &pcd_buffer_;
	CVMatBuffer& rgb_buffer_;
	CVMatBuffer& prop_buffer_;
	boost::shared_ptr<boost::thread> thread_;
	boost::mutex mutex_;
	pcl::PCDWriter writer_;
	bool stop_;
	int count_;
	int iter_;
	int max_iteration_length_;
	int pcd_flag_; // whether record pcd or rgb...
	SMP& arm_;
	cv::Mat& proprioception_;
	// timer variables
	
};
#endif // RECORDER_H

// std::string time = boost::posix_time::to_iso_string (boost::posix_time::microsec_clock::local_time ());

// note...
// count added...
// add time and signal to GUI to show recording frame rate...