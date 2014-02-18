#ifndef ARM_PCD_RECORDING_DIALOG_H
#define ARM_PCD_RECORDING_DIALOG_H

#include <QDialog>
#include <fstream>
#include "source/inc/pcd_buffer.h"
#include "source/inc/cv_mat_buffer.h"
#include "source/inc/kinect_grabber.h"
#include "source/inc/pcd_recorder.h"
#include "source/inc/smp.h"

namespace Ui {
class ArmPCDRecordingDialog;
}

class ArmPCDRecordingDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ArmPCDRecordingDialog(SMP& arm, QWidget *parent = 0, int buffer_size = 200);
	void PCDRecording();
	void GetTargetPoints(cv::Mat& target_list, SMP& arm);
	void CanPolling();
	void EventHandling();
	void RecordMatFloat(cv::Mat& src, int h, int w, std::string name, int append_flag);
	void set_smp_event_stop(bool smp_stop_flag);
	bool smp_event_stop();
	void set_smp_can_stop(bool smp_stop_flag);
	bool smp_can_stop();
    ~ArmPCDRecordingDialog();

private slots:
    void on_ini_button_clicked();
    void on_resume_button_clicked();
    void on_stop_button_clicked();
    void on_record_button_clicked();

public slots:
	void OnUpdateFrameRate(double);
	void OnUpdateTargetCount(int);

private:
    Ui::ArmPCDRecordingDialog *ui;
	PCDBuffer<pcl::PointXYZ> pcd_buffer_;
	CVMatBuffer prop_buffer_;
	KinectGrabber<pcl::PointXYZ>* grabber_ptr_;
	PCDRecorder<pcl::PointXYZ>* recorder_ptr_;
	boost::shared_ptr<boost::thread> thread_;
	boost::mutex mutex_;

	int buffer_size_;

	random_device rd_;
	int num_targets_;
	int num_joints_;	
	int recording_length_;
	int recording_iter_; // current recording iteration	
	float curr_deviation_;
	bool frame_update_flag_;
	bool smp_event_stop_;
	bool smp_can_stop_;
	cv::Mat target_list_;
	cv::Mat proprioception_;
	SMP arm_;
};

#endif // ARM_PCD_RECORDING_DIALOG_H
