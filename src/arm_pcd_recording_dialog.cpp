#include "source/inc/arm_pcd_recording_dialog.h"
#include "ui_arm_pcd_recording_dialog.h"
#include "opencv2/imgproc/types_c.h"
ArmPCDRecordingDialog::ArmPCDRecordingDialog(SMP& arm, QWidget *parent, int buffer_size) :
	arm_(arm),
    QDialog(parent),    
	buffer_size_(buffer_size),
	ui(new Ui::ArmPCDRecordingDialog)
{
	// initialize necessary parameters for target list...
	num_targets_ = 625;
	num_joints_ = 2;
	recording_length_ = 100000;
	recording_iter_ = 0;
	curr_deviation_ = 1.0;
	frame_update_flag_ = false;
	// target list, including target position and velocity	
	target_list_ = cv::Mat::zeros(num_targets_, 2 * num_joints_, CV_32F);		
	// proprioception matrix
	proprioception_ = cv::Mat::zeros(recording_length_, 2 * num_joints_, CV_32F);
	smp_event_stop_ = false;
	smp_can_stop_ = false;
	// gui set up
    ui->setupUi(this);		
	// kinect recording set up
	pcd_buffer_.set_capacity(buffer_size_);	
	prop_buffer_.set_capacity(buffer_size_);
	
}

ArmPCDRecordingDialog::~ArmPCDRecordingDialog()
{
    delete ui;
}

void ArmPCDRecordingDialog::on_ini_button_clicked()
{	
	GetTargetPoints(target_list_, arm_); // get arm targets
    thread_.reset(new boost::thread(boost::bind(&ArmPCDRecordingDialog::PCDRecording, this))); // start kinect reading thread	
	arm_.canInitial(); // initialize can bus	
	thread_.reset(new boost::thread(boost::bind(&ArmPCDRecordingDialog::CanPolling, this))); // start arm polling thread
	thread_.reset(new boost::thread(boost::bind(&ArmPCDRecordingDialog::EventHandling, this))); // start arm event handling thread
	// arm acknowledgement
	for(int idx = 0; idx <= 7; idx++)		
		arm_.CMD_ACK(idx);	
	// set get state frequency, 25 HZ
	// arm_.GET_STATE(0,0.04,0x07);
	arm_.GET_STATE(3, 0.04, 0x07);
	arm_.GET_STATE(5, 0.04, 0x07);	
	// enable hold for the joints...
	// arm_.enableHold(0);
	arm_.enableHold(3);
	arm_.enableHold(5);
	// move to some initial position		
	// arm_.MOVE_POS(0, 0, 2.5, 20, 10.0);	
	arm_.MOVE_POS(3, 50, 2.5, 20, 6.0);
	arm_.MOVE_POS(5, 50, 3.5, 20, 4.0);
	// arm_.MOVE_POS(5, 50, 3.5, 20, 4.0);
}

void ArmPCDRecordingDialog::on_resume_button_clicked()
{

}

void ArmPCDRecordingDialog::on_stop_button_clicked()
{
	
	grabber_ptr_->set_stop(true);
	recorder_ptr_->set_stop(true);	
	pcd_buffer_.set_stop(true);
	// arm_.disableHold(0);
	arm_.disableHold(3);
	arm_.disableHold(5);
	// arm_.CMD_STOP(0);
	arm_.CMD_STOP(3);
	arm_.CMD_STOP(5);
	set_smp_can_stop(true);
	set_smp_event_stop(true);
	
	// must be in this order...
}

void ArmPCDRecordingDialog::on_record_button_clicked()
{
	RecordMatFloat(proprioception_, recording_length_, 2 * num_joints_, "prop.bin", 0);
}

void ArmPCDRecordingDialog::PCDRecording()
{	    
    KinectGrabber<pcl::PointXYZ> grabber(pcd_buffer_, prop_buffer_, arm_); // start grabber thread
    boost::this_thread::sleep (boost::posix_time::seconds(2)); // wait for several seconds
    PCDRecorder<pcl::PointXYZ> recorder(pcd_buffer_, prop_buffer_, arm_, proprioception_ ); // start recorder thread
	grabber_ptr_ = &grabber; // assign pointers
	recorder_ptr_ = &recorder; // assign pointers
	connect(&(grabber_ptr_->boost_qt_), SIGNAL(UpdateFrameRateSignal(double)), this, SLOT(OnUpdateFrameRate(double)), Qt::QueuedConnection);
	connect(&(grabber_ptr_->boost_qt_), SIGNAL(UpdateTargetCountSignal(int)), this, SLOT(OnUpdateTargetCount(int)), Qt::QueuedConnection);
	grabber.WaitForThreadTermination(); // wait for the grabber thread...
	recorder.WaitForThreadTermination(); // wait for the recorder thread...
}

void ArmPCDRecordingDialog::OnUpdateFrameRate(double frequency)
{	
	ui->frame_rate_value->setText(QString::number(frequency));
}

void ArmPCDRecordingDialog::OnUpdateTargetCount(int target_count)
{	
	ui->data_number_value->setText(QString::number(target_count));
}

void ArmPCDRecordingDialog::CanPolling()
{
	while(true)
	{
		arm_.canMsgPoll();
		if(smp_can_stop_)
            break;        
	}
}

void ArmPCDRecordingDialog::EventHandling()
{
	while(true)
	{
		arm_.SingleEventHandling();		
		if(smp_event_stop_)
            break;        
	}
}

void ArmPCDRecordingDialog::GetTargetPoints(cv::Mat& target_list, SMP& arm)
{
	// random distribution	
	mt19937 engine(rd_());
	uniform_real_distribution<double> uniform(-1.0 * curr_deviation_, 1.0 * curr_deviation_);	
	// matrix structures
	cv::Mat joint_low_limit = cv::Mat::zeros(num_joints_, 1, CV_32F);
	cv::Mat joint_up_limit = cv::Mat::zeros(num_joints_, 1, CV_32F);
	cv::Mat speed_limit = cv::Mat::zeros(num_joints_, 1, CV_32F);
	cv::Mat dist = cv::Mat::zeros(num_joints_, 1, CV_32F);
	cv::Mat interval = cv::Mat::zeros(num_joints_, 1, CV_32F);
	// target list, including target position and velocity			
	cv::Mat sort_target_list = cv::Mat::zeros(num_targets_, 2 * num_joints_, CV_32F);
	cv::Mat sort_prop_idx = cv::Mat::zeros(num_targets_, 1, CV_8U);
	cv::Mat prop_center = cv::Mat::zeros(num_joints_, 1, CV_32F);
	cv::Mat prop_dist = cv::Mat::zeros(num_targets_, 1, CV_32F);
	float prev_target = 0;
	float max_interval = 0;
	// joint angle limits
	// joint_low_limit.at<float>(0, 0) = -10.0; joint_up_limit.at<float>(0, 0) = 10.0;
	joint_low_limit.at<float>(0, 0) = 40.0; joint_up_limit.at<float>(0, 0) = 60.0;
	joint_low_limit.at<float>(1, 0) = 40.0; joint_up_limit.at<float>(1, 0) = 60.0;	
	// joint centers...
	for(int i = 0; i < num_joints_; i++)
		prop_center.at<float>(i, 0) = (joint_low_limit.at<float>(i, 0) + joint_up_limit.at<float>(i, 0)) / 2;
	// joint angle speed limits
	// speed_limit.at<float>(0, 0) = 2.5;
	speed_limit.at<float>(0, 0) = 5.0;
	speed_limit.at<float>(1, 0) = 5.0;	
	// get the proprioception angles
	for(int idx = 0; idx < num_targets_; idx++)
		for(int i = 0; i < num_joints_; i++)
			target_list.at<float>(idx, 2 * i) = prop_center.at<float>(i, 0) + (uniform(engine) * (joint_up_limit.at<float>(i, 0) - joint_low_limit.at<float>(i, 0)) / 2);
	
	for(int i = 0; i < num_targets_; i++){
		for(int j = 0; j < num_joints_; j++){
			prop_dist.at<float>(i, 0) += std::pow(target_list.at<float>(i, 2 * j) - prop_center.at<float>(j, 0), 2);
		}
		prop_dist.at<float>(i, 0) = std::sqrt(prop_dist.at<float>(i, 0));
	}		
	cv::sortIdx(prop_dist, sort_prop_idx, CV_SORT_EVERY_COLUMN + CV_SORT_ASCENDING);
	
	//get proper speed
	for(int idx = 0; idx < num_targets_; idx++){
		for(int i = 0; i < num_joints_; i++){
			if(idx != 0)
				prev_target = target_list.at<float>(sort_prop_idx.at<int>(idx - 1, 0), 2 * i);
			else 
				prev_target = prop_center.at<float>(i, 0);
			dist.at<float>(i, 0) = abs(target_list.at<float>(sort_prop_idx.at<int>(idx, 0), 2 * i) - prev_target);
			interval.at<float>(i, 0) = dist.at<float>(i, 0) / speed_limit.at<float>(i, 0);
		}
		
		for(int i = 0; i < num_joints_; i++)
			max_interval = interval.at<float>(i, 0) > max_interval ? interval.at<float>(i, 0) : max_interval;
		for(int i = 0; i < num_joints_; i++){
			if(idx != 0)
				prev_target = target_list.at<float>(sort_prop_idx.at<int>(idx - 1, 0), 2 * i);
			else 
				prev_target = prop_center.at<float>(i, 0);		
			target_list.at<float>(sort_prop_idx.at<int>(idx, 0), 2 * i + 1) = abs(target_list.at<float>(sort_prop_idx.at<int>(idx, 0), 2 * i) - prev_target) / max_interval;
		}
	}
	std::ofstream osp("target.txt");
	for(int i=0; i < num_targets_; i++){
		for(int j = 0; j < 2 * num_joints_; j++){
			osp << target_list.at<float>(sort_prop_idx.at<int>(i, 0), j) <<" ";
			sort_target_list.at<float>(i, j) = target_list.at<float>(sort_prop_idx.at<int>(i, 0), j);
		}
		osp<<std::endl;
	}
	osp.close();

	arm.SetTargetList(sort_target_list, num_targets_, num_joints_);
}

void ArmPCDRecordingDialog::set_smp_event_stop(bool smp_stop_flag)
{
	boost::mutex::scoped_lock buff_lock (mutex_);
	smp_event_stop_ = smp_stop_flag;
}

bool ArmPCDRecordingDialog::smp_event_stop()
{
	boost::mutex::scoped_lock buff_lock (mutex_);
	return smp_event_stop_;
}

void ArmPCDRecordingDialog::set_smp_can_stop(bool smp_stop_flag)
{
	boost::mutex::scoped_lock buff_lock (mutex_);
	smp_can_stop_ = smp_stop_flag;
}

bool ArmPCDRecordingDialog::smp_can_stop()
{
	boost::mutex::scoped_lock buff_lock (mutex_);
	return smp_can_stop_;
}

void ArmPCDRecordingDialog::RecordMatFloat(cv::Mat& src, int h, int w, std::string name, int append_flag){
	FILE* file_pt;
	if(append_flag)
		file_pt = fopen((char*)name.c_str(), "ab");
	else
		file_pt = fopen((char*)name.c_str(), "wb");
	for(int i = 0; i < h; i++){
		for(int j = 0; j < w; j++){
			fwrite(&(src.at<float>(i, j)), sizeof(float), 1, file_pt);
		}
	}
	fclose(file_pt);
}