#ifndef KINECT_GRABBER_H
#define KINECT_GRABBER_H

#include "source/inc/pcd_buffer.h"
#include "source/inc/cv_mat_buffer.h"
#include "source/inc/boost_qt.h"
#include "source/inc/smp.h"
#include <Windows.h>

template <typename PointT>
class KinectGrabber
{

private:	
	void 
    KinectGrabberCallBack (const typename pcl::PointCloud<PointT>::ConstPtr& cloud)
    {
		// calculate time...
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
		// push back point cloud pointer
		pcd_buffer_.PushBack(cloud);		
		cv::Mat curr_prop = cv::Mat::zeros(1, 2 * num_joints_, CV_32F);		
		// assign value
		{
			boost::mutex::scoped_lock buff_lock(mutex_);
			for(int idx = 0; idx < num_joints_; idx++)
			{
				curr_prop.at<float>(0, 2 * idx) = arm_.module[int(joint_idx_.at<float>(idx, 0))].position; //arm->module[7].position;
				curr_prop.at<float>(0, 2 * idx + 1) = arm_.module[int(joint_idx_.at<float>(idx, 0))].velocity; // arm->module[7].velocity;
			}
		}
		prop_buffer_.PushBack(curr_prop);			
    }

    void 
    ReadKinectPCD ()
    {
		// typical openni grabber routine
		pcl::Grabber* kinect_grabber = new pcl::OpenNIGrabber ();
		boost::function<void (const typename pcl::PointCloud<PointT>::ConstPtr&)> f = boost::bind(&KinectGrabber::KinectGrabberCallBack, this, _1);
		kinect_grabber->registerCallback(f);
		kinect_grabber->start();
		while (true)
		{
			if(stop_)				
				break;			
			boost::this_thread::sleep (boost::posix_time::milliseconds(100));
		}
		kinect_grabber->stop ();
    }

public:
    KinectGrabber(PCDBuffer<PointT> &pcd_buffer, CVMatBuffer& prop_buffer, SMP& arm)
      : pcd_buffer_(pcd_buffer),
		prop_buffer_(prop_buffer),
		arm_(arm)
    {
		stop_ = false;
		prev_tick_ = 0;
		curr_tick_ = 0;
		tick_count_ = 0;
		num_joints_ = 2;
		joint_idx_ = cv::Mat::zeros(num_joints_, 1, CV_32F);
		joint_idx_.at<float>(0, 0) = 3; joint_idx_.at<float>(1, 0) = 5; // joint_idx_.at<float>(0, 2) = 5; 
		QueryPerformanceFrequency(&frequency_);
		thread_.reset(new boost::thread (boost::bind(&KinectGrabber::ReadKinectPCD, this)));
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    void
    WaitForThreadTermination()
    {
		// wait for the thread to terminate
		thread_->join ();      
    }

	inline void
	set_stop(bool stop_flag)
	{
		boost::mutex::scoped_lock buff_lock (mutex_);
		stop_ = stop_flag;
	}

	inline bool
	stop()
	{
		boost::mutex::scoped_lock buff_lock (mutex_);
		return stop_;
	}
public:
	BOOSTQT boost_qt_;

private:
    PCDBuffer<PointT> &pcd_buffer_;
	CVMatBuffer& prop_buffer_;
    boost::shared_ptr<boost::thread> thread_;
	boost::mutex mutex_;
	bool stop_;
	__int64 prev_tick_; // high performance time tick previous
	__int64 curr_tick_; // high performance time tick current   
	int tick_count_;
    LARGE_INTEGER tick_query_;
	LARGE_INTEGER frequency_;
	SMP& arm_;
	cv::Mat joint_idx_;
	int num_joints_;
	
};
#endif // KINECT_GRABBER_H
