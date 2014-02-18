#ifndef CV_MAT_BUFFER_H
#define CV_MAT_BUFFER_H

#include "opencv2/core/core.hpp"
#include <boost/thread/condition.hpp>
#include <boost/circular_buffer.hpp>
#include <csignal>

class CVMatBuffer
{
public:
	CVMatBuffer() 
	{
		stop_ = false;
	}

	void
    PushBack(cv::Mat data) // thread-save wrapper for front() method of ciruclar_buffer
	{		
		{
			boost::mutex::scoped_lock buff_lock(mutex_);	
			buffer_.push_back(data);
		}
		buffer_empty_.notify_one ();		
	}

    cv::Mat
    GetFront() // thread-save wrapper for front() method of ciruclar_buffer
	{
		cv::Mat data;
		{
			boost::mutex::scoped_lock buff_lock(mutex_);
			while (buffer_.empty ())
			{    
				if(stop_)
					break;
				buffer_empty_.wait(buff_lock);
			}
			data = buffer_.front ();
			buffer_.pop_front ();
		}
		return (data);
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
	CVMatBuffer(const CVMatBuffer&); // Disabled copy constructor
    CVMatBuffer& operator =(const CVMatBuffer&); // Disabled assignment operator

    boost::mutex mutex_;
    boost::condition_variable buffer_empty_;
    boost::circular_buffer<cv::Mat> buffer_;
	bool stop_;
};

#endif