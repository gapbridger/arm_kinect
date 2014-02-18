#ifndef PCD_BUFFER_H
#define PCD_BUFFER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <boost/thread/condition.hpp>
#include <boost/circular_buffer.hpp>
#include <csignal>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h> //fps calculations


template <typename PointT>
class PCDBuffer
{
public:
	PCDBuffer() 
	{
		stop_ = false;
	}

    bool 
    PushBack(typename pcl::PointCloud<PointT>::ConstPtr cloud) // thread-save wrapper for front() method of ciruclar_buffer
	{
		bool ret_val = false;
		{
			boost::mutex::scoped_lock buff_lock(mutex_);
			if (!buffer_.full ())
				ret_val = true;
			buffer_.push_back(cloud);
		}
		buffer_empty_.notify_one ();
		return (ret_val);
	}

    typename pcl::PointCloud<PointT>::ConstPtr 
    GetFront() // thread-save wrapper for front() method of ciruclar_buffer
	{
		typename pcl::PointCloud<PointT>::ConstPtr cloud;
		{
			boost::mutex::scoped_lock buff_lock(mutex_);
			while (buffer_.empty ())
			{    
				if(stop_)
					break;
				buffer_empty_.wait(buff_lock);
			}
			cloud = buffer_.front ();
			buffer_.pop_front ();
		}
		return (cloud);
	}

    inline bool 
    full()
    {
		boost::mutex::scoped_lock buff_lock (mutex_);
		return (buffer_.full ());
    }

    inline bool
    empty()
    {
      boost::mutex::scoped_lock buff_lock (mutex_);
      return (buffer_.empty ());
    }

    inline int 
    size()
    {
		boost::mutex::scoped_lock buff_lock (mutex_);
		return (int (buffer_.size ()));
    }

    inline int 
    capacity()
    {
		return (int (buffer_.capacity ()));
    }

    inline void 
    set_capacity(int buffer_size)
    {
		boost::mutex::scoped_lock buff_lock(mutex_);
		buffer_.set_capacity(buffer_size);
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

private:
	PCDBuffer(const PCDBuffer&); // Disabled copy constructor
    PCDBuffer& operator =(const PCDBuffer&); // Disabled assignment operator

    boost::mutex mutex_;
    boost::condition_variable buffer_empty_;
    boost::circular_buffer<typename pcl::PointCloud<PointT>::ConstPtr> buffer_;
	bool stop_;
};

#endif