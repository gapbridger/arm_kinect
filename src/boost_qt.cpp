#include "source/inc/boost_qt.h"

BOOSTQT::BOOSTQT()
{

}

void BOOSTQT::UpdateFrameRate(double frame_rate)
{
	emit UpdateFrameRateSignal(frame_rate);
}

void BOOSTQT::UpdateTargetCount(int target_count)
{
	emit UpdateTargetCountSignal(target_count);
}