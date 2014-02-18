#ifndef PCD_RECORDER_H
#define PCD_RECORDER_H

#include "source/inc/pcd_buffer.h"
#include "source/inc/cv_mat_buffer.h"
#include "source/inc/smp.h"

template <typename PointT>
class PCDRecorder
{
private:
    void 
    RecordPCD(const typename pcl::PointCloud<PointT>::ConstPtr& cloud, cv::Mat& curr_prop)
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
    
    // Consumer thread function
    void 
    ReceivePCDData()
    {
		while (true)
		{
			if(stop_)				
				break;			
			RecordPCD(pcd_buffer_.GetFront (), prop_buffer_.GetFront());
		}
		while(!pcd_buffer_.empty ())
			RecordPCD(pcd_buffer_.GetFront (), prop_buffer_.GetFront());
    }
	 
public:
    PCDRecorder(PCDBuffer<PointT> &pcd_buffer, CVMatBuffer& prop_buffer, SMP& arm, cv::Mat& proprioception)
      : pcd_buffer_(pcd_buffer),
		prop_buffer_(prop_buffer),
		arm_(arm),
		proprioception_(proprioception)
    {
		stop_ = false;
		count_ = 1;
		iter_ = 0;
		max_iteration_length_ = 1e5;				
		thread_.reset(new boost::thread(boost::bind (&PCDRecorder::ReceivePCDData, this)));
    }

    void
    WaitForThreadTermination()
    {
		thread_->join ();
    }

	inline void
	set_stop(bool stop_flag)
	{
		boost::mutex::scoped_lock buff_lock(mutex_);
		stop_ = stop_flag;
	}

	inline bool
	stop()
	{
		boost::mutex::scoped_lock buff_lock(mutex_);
		return stop_;
	}

private:
	PCDBuffer<PointT> &pcd_buffer_;
	CVMatBuffer& prop_buffer_;
	boost::shared_ptr<boost::thread> thread_;
	boost::mutex mutex_;
	pcl::PCDWriter writer_;
	bool stop_;
	int count_;
	int iter_;
	int max_iteration_length_;
	SMP& arm_;
	cv::Mat& proprioception_;
	// timer variables
	
};
#endif // PCD_RECORDER_H

// std::string time = boost::posix_time::to_iso_string (boost::posix_time::microsec_clock::local_time ());

// note...
// count added...
// add time and signal to GUI to show recording frame rate...