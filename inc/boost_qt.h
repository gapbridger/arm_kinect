#ifndef BOOST_QT_H
#define BOOST_QT_H
#include <QThread>
#include <QMutex>

class BOOSTQT : public QObject
{
    Q_OBJECT

public:
	BOOSTQT();
	void UpdateFrameRate(double);
	void UpdateTargetCount(int);

signals:	
	void UpdateFrameRateSignal(double);
	void UpdateTargetCountSignal(int);
};

#endif