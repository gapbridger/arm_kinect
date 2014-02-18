#include "source/inc/arm_pcd_recording_dialog.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
	SMP arm;
    ArmPCDRecordingDialog w(arm);
    w.show();

    return a.exec();
}
