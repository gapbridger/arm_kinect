#ifndef SMP_H
#define SMP_H

#include "windows.h"
#include  <time.h>
#include "ntcan.h"
#include <iostream>
#include <fstream>
// #include <iomanip>
#include <process.h>          // for _beginthread()
#include <random>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
using namespace std;

#define MSG_SLAVE 0x0700
#define MSG_MASTER 0x0500
#define MSG_ERROR 0x0300
#define GET_ID 0x00FF
#define EMPTY_Q 32
#define UNKNOWN_MSG 90
#define OUT_OF_Q 91
#define RX_TIME_OUT 92
#define PARSE_SUCCESS 93
#define FRAGGING 94
#define	REFERENCE_FAIL 95
#define	CMD_STOP_FAIL 96
#define	ACK_FAIL 96
#define EVENT_REACHED 97
#define EVENT_NULL 98

#define CHECK_REFERENCED 0x01
#define CHECK_MOVING 0x02
#define CHECK_ERROR 0x10
#define CHECK_BRAKE 0x20
#define CHECK_MOV_END 0x40
#define CHECK_POS_REACHED 0x80


//************ COMMAND MACRO DEFINITION ***************//

#define _REFERENCE 0x92
#define _MOV_POS 0xB0
#define _MOV_VEL 0xB5
#define _CMD_STOP 0x91
#define _CMD_INFO 0x8A
#define _CMD_ACK 0x8B
#define _FRAG_ACK 0x87
#define	_FRAG_BEGIN 0x84
#define _FRAG_MIDDLE 0x85
#define _FRAG_END 0x86
#define _REFERENCE 0x92
#define _CMD_MOV_END 0x93 // implementable
#define _CMD_POS_REACHED 0x94 // implementable
#define _GET_STATE 0x95 // implementable
#define _CMD_ERROR 0x88
#define _CMD_INFO 0x8A
#define _GET_CONFIG 0x80



// more move command needed
// set command needed


//************* COMMAND MACRO DEFINITION *****************//
//#include "math.h"
//#include <list>
//#include <vector>

// Module struct
// collect information related to a module



typedef struct
{
	int32_t	id;
	uint8_t	status; 
	uint8_t	errorCode; 
	uint8_t	getStateMode;

	float	position;
	float	velocity;
	float	current;

	bool	referenced;
	bool	moving;
	bool	brake;
	bool	moveEnd;
	bool	posReached;
	bool	error;

	bool	hold;
	bool	movBack;
	bool	movForth;

	int		mEvent;

} MODULE;

class SMP{
public:
	SMP();
	~SMP();
	NTCAN_RESULT	canInitial();
	NTCAN_HANDLE	canConnect(uint32_t baud); // open CAN communication
	NTCAN_RESULT	canScanBus(NTCAN_HANDLE handle, int32_t idStart, int32_t idEnd); // scan bus
	NTCAN_RESULT	canDisconnect(); // close CAN communication
	NTCAN_RESULT	CMD_ACK(int index);
	NTCAN_RESULT	CMD_REFERENCE(int index);
	NTCAN_RESULT	CMD_STOP(int index);
	NTCAN_RESULT	CMD_EMERGENCY_STOP(int index);
	NTCAN_RESULT	MOVE_POS(int index, float pos);
	NTCAN_RESULT	MOVE_POS(int index, float pos, float vel, float acc, float cur);
	NTCAN_RESULT	MOVE_VEL(int index, float vel);
	NTCAN_RESULT	MOVE_VEL(int index, float vel, float cur);
	NTCAN_RESULT	GET_STATE(int index, float time, uint8_t mode);
	int				canMsgPoll();
	int				procNextMsg(int count, CMSG* cmsg); // parse next message in the buffer
	int				parseFragMsg(int mIndex);
	void			float2Bit(float float_num, uint8_t* bit_num);
	float			bit2Float(uint8_t* bit_num);
	void			canPolling();
	void			eventHandling();
	void SingleEventHandling();
	void			callBackReached(int index);
	void			errorHandler(int index);


	void			enableCanPoll();
	void			disableCanPoll();
	void			enableCanComm();
	void			disableCanComm();
	void			enableEvent();
	void			disableEvent();
	void			enableHold(int index);
	void			disableHold(int index);

	CRITICAL_SECTION cSec; 

	static unsigned __stdcall canPollingThreadStartUp(void * pThis){
		SMP* pSMP = (SMP*)pThis;
		pSMP->canPolling();

		return 1;
	}

	static unsigned __stdcall eventThreadStartUp(void * pThis){
		SMP* pSMP = (SMP*)pThis;
		pSMP->eventHandling();

		return 1;
	}

	MODULE			module[8];
	void GetNextTarget(mt19937& engine, uniform_real_distribution<double>& uniform, int idx);
	int GetTargetCount(int index);
	void SetTargetCount(int index, int count);
	void SetTargetList(const cv::Mat& target, int target_length, int num_joints);

private:

	NTCAN_HANDLE		handle;
	int					offset;
	uint32_t			baud;
	int32_t				idStart;
	int32_t				idEnd;
	uint8_t				*pos_buffer;
	uint8_t				*vel_buffer;
	uint8_t				*cur_buffer;
	CMSG				fragStartBuf;
	CMSG				fragMidBuf;
	CMSG				pollBuf[40]; // buffer, length 10
	CMSG				fragBuf[8][40];
	int					fragLen[8];

	float				v0;
	float				v3;
	float				v5;
	float				v7;
	
	bool				canCommActive;
	bool				canPollActive;
	bool				eventActive;
	
	
	int smp_target_length_;
	cv::Mat smp_target_list_;	
	float* target_count_;
	int smp_num_joints_;

	
};

#endif
