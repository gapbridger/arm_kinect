// SMP.cpp : Defines the entry point for the console application.
//

#include "source/inc/smp.h"

SMP::SMP(){
	handle = NULL;
	baud = NTCAN_BAUD_1000;
	idStart = 8;
	idEnd = 15;
	int j = 0;
	offset = 8;
	for (int32_t i = idStart; i <= idEnd; i++ ){
		module[j].id = i;
		module[j].current = 0;
		module[j].velocity = 0;
		module[j].position = 0;
		module[j].errorCode = 0x00;
		module[j].getStateMode = 0x00;
		module[j].referenced = true;
		module[j].moving = false;
		module[j].brake = true;
		module[j].moveEnd = true;
		module[j].posReached = true;
		module[j].error = false;
		module[j].mEvent = EVENT_NULL;
		module[j].hold = false;
		module[j].movBack = false;
		module[j].movForth = false;
		j++;
	}
	for(int i = 0; i < 8; i++)
		fragLen[i]=0;

	pos_buffer=(uint8_t*)new uint8_t[4];
	vel_buffer=(uint8_t*)new uint8_t[4];
	cur_buffer=(uint8_t*)new uint8_t[4];
	
	InitializeCriticalSection(&cSec);
	canCommActive = false;
	canPollActive = true;
	eventActive = true;

	v0 = 0.0;
	v3 = 0.0;
	smp_target_length_ = 0;
	smp_num_joints_ = 0;
	
	
}

SMP::~SMP(){
	// DeleteCriticalSection(&cSec);
}

NTCAN_HANDLE SMP::canConnect(uint32_t baud){

	// please refer to can manual for parameters' meaning

	int net = 0; 
	uint32_t mode = 0;
	int32_t txQueueSize = 40;
	int32_t rxQueueSize = 40;
	int32_t txTimeout = 100; 
	int32_t rxTimeout = 1000; 
	NTCAN_RESULT retValue; 

	retValue = canOpen(net, mode, txQueueSize, rxQueueSize, txTimeout, rxTimeout, &handle);

	if (retValue != NTCAN_SUCCESS){
		cout<<"canOpen() failed with error "<<retValue<<" !"<<endl;
		return(NULL);
	}
	cout<<"function canOpen() returned OK!"<<endl;
	
	retValue = canSetBaudrate(handle, baud);

	if (retValue != NTCAN_SUCCESS){
		cout<<"canSetBaudrate() failed with error "<<retValue<<" !"<<endl;
		canClose(handle);
		return(NULL);
	}
	cout<<"function canSetBaudrate() returned OK!"<<endl;

	return handle;
	
}

NTCAN_RESULT SMP::canScanBus(NTCAN_HANDLE handle, int32_t idStart, int32_t idEnd){
	NTCAN_RESULT retValue;
	int32_t id=0;

	for (int32_t i = idStart; i <= idEnd; i++){

		id = i|MSG_MASTER;
		retValue = canIdAdd(handle, id);

		if (retValue != NTCAN_SUCCESS){
			cout << "canIdAdd() failed with error " << retValue << " !" << endl;
			return(NTCAN_ID_NOT_ENABLED);
		}
		else{
			cout << "Module " << i << " Master on line!" << endl;
		}

		id = i|MSG_SLAVE;
		retValue = canIdAdd(handle, id);

		if (retValue != NTCAN_SUCCESS){
			cout << "canIdAdd() failed with error " << retValue << " !" << endl;
			return(NTCAN_ID_NOT_ENABLED);
		}
		else{
			cout << "Module " << i << " Slave on line!" << endl;
		}

		id = i|MSG_ERROR;
		retValue = canIdAdd(handle, id);

		if (retValue != NTCAN_SUCCESS){
			cout << "canIdAdd() failed with error " << retValue << " !" << endl;
			return(NTCAN_ID_NOT_ENABLED);
		}
		else{
			cout << "Module " << i << " Error on line!" << endl;
		}
	}

	return NTCAN_SUCCESS;
}

NTCAN_RESULT SMP::canInitial(){
	NTCAN_RESULT retValue;
	
	handle = canConnect(baud);
	if (handle == NULL){
		cout << "NTCAN net not found!" << endl;
		return(NTCAN_NET_NOT_FOUND);
	}

	retValue = canScanBus(handle, idStart, idEnd);
	if (retValue != NTCAN_SUCCESS){
		cout << "Scan Bus failed, ID not found!" << endl;
		return(NTCAN_ID_NOT_ENABLED);
	}
	
	return NTCAN_SUCCESS;
	
}

NTCAN_RESULT SMP::canDisconnect(){

	NTCAN_RESULT retValue;
	int32_t id=0;

	for (int32_t i = idStart; i <= idEnd; i++){

		id = i|MSG_MASTER;
		retValue = canIdDelete(handle, id); // delete master id

		if (retValue != NTCAN_SUCCESS){
			cout << "canIdDelete() failed with error " << retValue << " !" << endl;
			return(NTCAN_WRONG_DEVICE_STATE);
		}
		else{
			cout << "Module " << i << " Master off line!" << endl;
		}

		id = i|MSG_SLAVE;
		retValue = canIdDelete(handle, id); // delete slave id

		if (retValue != NTCAN_SUCCESS){
			cout << "canIdDelete() failed with error " << retValue << " !" << endl;
			return(NTCAN_WRONG_DEVICE_STATE);
		}
		else{
			cout << "Module " << i << " Slave off line!" << endl;
		}

		id = i|MSG_ERROR;
		retValue = canIdDelete(handle, id); // delete error id

		if (retValue != NTCAN_SUCCESS){
			cout << "canIdDelete() failed with error " << retValue << " !" << endl;
			return(NTCAN_WRONG_DEVICE_STATE);
		}
		else{
			cout << "Module " << i << " Error off line!" << endl;
		}
	}

	retValue = canClose(handle);
	if (retValue != NTCAN_SUCCESS){
		cout << "NTCAN net not closed!" << endl;
		return(NTCAN_WRONG_DEVICE_STATE);
	}
	else{
		cout << "CAN bus disconnected!" << endl;
	}

	return NTCAN_SUCCESS;
}

NTCAN_RESULT SMP::CMD_ACK(int index){

	NTCAN_RESULT retValue;
	CMSG cmsg;
	int32_t len=1;
	
	
	cmsg.len=2;
	cmsg.id=module[index].id|MSG_MASTER;
	for (int i=0; i<8; i++)
		cmsg.data[i]=0;
	cmsg.data[0]=0x01;
	cmsg.data[1]=0x8B;

	// retValue = canWrite(handle, &cmsg, &len, NULL); // acknowledgement
	retValue = canSend(handle, &cmsg, &len); // acknowledgement
	if (retValue != NTCAN_SUCCESS ){
		cout << "Failed to acknowledge " << retValue << endl;
		return NTCAN_TX_ERROR;
	}
	return retValue;
}

NTCAN_RESULT SMP::CMD_REFERENCE(int index){

	NTCAN_RESULT retValue;
	CMSG cmsg;
	int32_t len=1;
	
	retValue = CMD_ACK(index);//CMD ACK first

	cmsg.len=2;
	cmsg.id=module[index].id|MSG_MASTER;
	for (int i=0; i<8; i++)
		cmsg.data[i]=0;
	cmsg.data[0]=0x01;
	cmsg.data[1]=0x92;

	// retValue = canWrite(handle, &cmsg, &len, NULL); // reference
	retValue = canSend(handle, &cmsg, &len); // reference

	if (retValue != NTCAN_SUCCESS ){
		cout << "Failed to reference!" << endl;
		return NTCAN_TX_ERROR;
	}
	return retValue;
}

NTCAN_RESULT SMP::CMD_STOP(int index){

	NTCAN_RESULT retValue;
	CMSG cmsg;
	int32_t len=1;
	cmsg.len=2;
	cmsg.id=module[index].id|MSG_MASTER;

	for (int i=0; i<8; i++)
		cmsg.data[i]=0;
	cmsg.data[0]=0x01;
	cmsg.data[1]=0x91;
	
	// retValue = canWrite(handle, &cmsg, &len, NULL); 
	retValue = canSend(handle, &cmsg, &len); 

	if (retValue != NTCAN_SUCCESS ){
		cout << "Failed to stop!" << endl;
		return NTCAN_TX_ERROR;
	}
	return retValue;
}

NTCAN_RESULT SMP::CMD_EMERGENCY_STOP(int index){

	NTCAN_RESULT retValue;
	CMSG cmsg;
	int32_t len=1;
	
	cmsg.len=2;
	cmsg.id=module[index].id|MSG_MASTER;

	for (int i=0; i<8; i++)
		cmsg.data[i]=0;
	cmsg.data[0]=0x01;
	cmsg.data[1]=0x90;
	
	// retValue = canWrite(handle, &cmsg, &len, NULL);
	retValue = canSend(handle, &cmsg, &len);

	if (retValue != NTCAN_SUCCESS ){
		cout << "Failed to emergency stop !" << endl;
		return NTCAN_TX_ERROR;
	}
	return retValue;
}

NTCAN_RESULT SMP::MOVE_POS(int index, float pos){

	NTCAN_RESULT retValue;
	CMSG cmsg;
	int32_t len=1;
	uint8_t *param_pos=(uint8_t*)new uint8_t[4];

	float2Bit(pos, param_pos);

	cmsg.len=6;
	cmsg.id=module[index].id|MSG_MASTER;

	for (int i=0; i<8; i++)
		cmsg.data[i]=0;
	cmsg.data[0]=0x05;
	cmsg.data[1]=0xB0;
	for (int i=2;i<6;i++)
		cmsg.data[i]=param_pos[i-2];

	// retValue = canWrite(handle, &cmsg, &len, NULL);
	retValue = canSend(handle, &cmsg, &len);

	delete[] param_pos;

	return retValue;
}

NTCAN_RESULT SMP::MOVE_POS(int index, float pos ,float vel, float acc, float cur){

	NTCAN_RESULT retValue;
	CMSG* cmsg;
	cmsg=new CMSG[3];
	int32_t len=3;
	uint8_t direction=0x05;
	uint8_t *param_pos=(uint8_t*)new uint8_t[4];
	uint8_t *param_vel=(uint8_t*)new uint8_t[4];
	uint8_t *param_acc=(uint8_t*)new uint8_t[4];
	uint8_t *param_cur=(uint8_t*)new uint8_t[4];

	float2Bit(pos, param_pos);
	float2Bit(vel, param_vel);
	float2Bit(acc, param_acc);
	float2Bit(cur, param_cur);

	for (int i=0; i<8; i++){
		cmsg[0].data[i] = 0;
		cmsg[1].data[i] = 0;
		cmsg[2].data[i] = 0;
	}

	cmsg[0].id = module[index].id|MSG_MASTER;
	cmsg[1].id = module[index].id|MSG_MASTER;
	cmsg[2].id = module[index].id|MSG_MASTER;

	cmsg[0].data[0]=0x11; // length of each message data
	cmsg[1].data[0]=0x0B;
	cmsg[2].data[0]=0x05;

	cmsg[0].data[1]=0x84; // fragmentation
	cmsg[1].data[1]=0x85;
	cmsg[2].data[1]=0x86;

	cmsg[0].data[2]=0xB0; // command

	cmsg[0].len=8;
	cmsg[1].len=8;
	cmsg[2].len=7;

	for (int i=3;i<7;i++)
		cmsg[0].data[i]=param_pos[i-3];
	cmsg[0].data[7]=param_vel[0];

	for (int i=2;i<5;i++)
		cmsg[1].data[i]=param_vel[i-1];
	for (int i=5;i<8;i++)
		cmsg[1].data[i]=param_acc[i-5];

	cmsg[2].data[2]=param_acc[3];
	for(int i=3;i<7;i++)
		cmsg[2].data[i]=param_cur[i-3];
		
	//retValue = canWrite(handle, cmsg, &len, NULL);
	/*EnterCriticalSection(&cSec);
	if(pos > module[index].position){
		module[index].movForth = true;
		module[index].movBack = false;
	}
	else{
		module[index].movForth = false;
		module[index].movBack = true;
	}			
	LeaveCriticalSection(&cSec);*/
	
	retValue = canSend(handle, cmsg, &len);
	
	delete[] cmsg;
	delete[] param_pos;
	delete[] param_vel;
	delete[] param_acc;
	delete[] param_cur;

	return retValue;
}

NTCAN_RESULT SMP::MOVE_VEL(int index, float vel){

	NTCAN_RESULT retValue;
	CMSG cmsg;
	int32_t len=1;
	uint8_t *param_vel = (uint8_t*)new uint8_t[4];
	float2Bit(vel, param_vel);

	cmsg.len=6;
	cmsg.id=module[index].id|MSG_MASTER;

	for (int i=0; i<8; i++)
		cmsg.data[i]=0;
	cmsg.data[0]=0x05;
	cmsg.data[1]=0xB5;
	for (int i=2;i<6;i++)
		cmsg.data[i]=param_vel[i-2];
		
	//retValue = canWrite(handle, &cmsg, &len, NULL);
	retValue = canSend(handle, &cmsg, &len);

	delete[] param_vel;

	return retValue;
}

NTCAN_RESULT SMP::MOVE_VEL(int index, float vel ,float cur){
	NTCAN_RESULT retValue;
	CMSG* cmsg;
	cmsg=new CMSG[2];
	int32_t len=2;
	uint32_t tmp=0x05;
	uint8_t *param_vel = (uint8_t*)new uint8_t[4];
	uint8_t *param_cur = (uint8_t*)new uint8_t[4];

	float2Bit(vel,param_vel);
	float2Bit(cur,param_cur);

	for (int i=0; i<8; i++){
		cmsg[0].data[i] = 0;
		cmsg[1].data[i] = 0;
	}

	cmsg[0].id = module[index].id|MSG_MASTER;
	cmsg[1].id = module[index].id|MSG_MASTER;

	cmsg[0].data[0] = 0x09; // can message data length
	cmsg[1].data[0] = 0x03;

	cmsg[0].data[1] = 0x84; // fragmentation
	cmsg[1].data[1] = 0x86;

	cmsg[0].data[2] = 0xB5; // command
	cmsg[0].len = 8;
	cmsg[1].len = 5;

	for (int i=3;i<7;i++)
		cmsg[0].data[i] = param_vel[i-3];
	cmsg[0].data[7] = param_cur[0];

	for (int i=1;i<4;i++)
		cmsg[1].data[i+1] = param_cur[i];
		
	//retValue = canWrite(handle, cmsg, &len, NULL);
	retValue = canSend(handle, cmsg, &len);

	delete[] cmsg;
	delete[] param_vel;
	delete[] param_cur;
	return retValue;
}

NTCAN_RESULT SMP::GET_STATE(int index, float time, uint8_t mode){

	NTCAN_RESULT retValue;
	CMSG cmsg;
	int32_t len=1;
	uint8_t *param_time = (uint8_t*)new uint8_t[4];
	
	cmsg.id = module[index].id|MSG_MASTER;
	if(time == 0){
		cmsg.len = 2;
		for (int i = 0; i < 8; i++)
			cmsg.data[i] = 0;
		cmsg.data[0] = 0x01;
		cmsg.data[1] = 0x95;
	}
	else{
		cmsg.len=7;
		for (int i=0; i<8; i++)
			cmsg.data[i]=0;
		float2Bit(time, param_time);
		cmsg.data[0] = 0x05;
		cmsg.data[1] = 0x95;
		for (int i=2;i<6;i++)
			cmsg.data[i] = param_time[i-2];
		
		cmsg.data[6] = mode;
		module[index].getStateMode = mode;
	}

	//retValue = canWrite(handle, &cmsg, &len, NULL);
	retValue = canSend(handle, &cmsg, &len);
	delete[] param_time;
	return retValue;
}



void SMP::float2Bit(float float_num, uint8_t* bit_num)
{
	union { 
		uint8_t n[4]; 
		float num;
	}tmp;
	tmp.num = float_num;
	for(int i=0; i<4; i++)
		bit_num[i]=tmp.n[i];
}

float SMP::bit2Float(uint8_t* bit_num)
{
	union { 
		uint8_t n[4]; 
		float num; 
	}tmp;
	for(int i=0; i<4; i++ )
		tmp.n[i] = bit_num[i];

	return tmp.num;
}

int SMP::canMsgPoll(){
	NTCAN_RESULT retValue;
	int32_t len = 40;
	int parseRet = 0;
	
	
	int count=0;

	retValue = canTake(handle, pollBuf, &len);

	if(retValue != NTCAN_SUCCESS)
		return RX_TIME_OUT;

	if(len == 0)
		return EMPTY_Q;
	//start = clock();
	while(count < len){		
		parseRet = procNextMsg(count, pollBuf);
		count++;
	}
	//end = clock();
	//timeSum = timeSum + (float)(end - start) / (float)(CLOCKS_PER_SEC * len);
	//parseCounter++;
	return parseRet;

}

int SMP::procNextMsg(int count, CMSG* cmsg){
	int mIndex = 0;
	int parseRet = 0;
	if(cmsg[count].data[1] == _FRAG_BEGIN || cmsg[count].data[1] == _FRAG_MIDDLE || cmsg[count].data[1] == _FRAG_END){
		// if it is a fragment message, push into the buffer
		mIndex = cmsg[count].id & GET_ID - offset;
		// copy the msg into the buffer
		fragBuf[mIndex][fragLen[mIndex]].id = cmsg[count].id;
		fragBuf[mIndex][fragLen[mIndex]].len = cmsg[count].len;
		for(int i = 0; i < 8; i++)
			fragBuf[mIndex][fragLen[mIndex]].data[i] = cmsg[count].data[i];
		fragLen[mIndex]++;
		if(cmsg[count].data[1] == _FRAG_END){
			// process fragment here, potentially seperate to another process
			parseRet = parseFragMsg(mIndex);
			fragLen[mIndex] = 0;
			return parseRet;
		}
		return FRAGGING;

	}
	else{
		// non fragmented messages
		mIndex = cmsg[count].id & GET_ID - offset;
		if(cmsg[count].data[1] == _CMD_POS_REACHED){ // cmd pos reached
			// position parsed
			module[mIndex].posReached = true; // maybe we need inter thread control here
			module[mIndex].moving = false;
			pos_buffer[0] = cmsg[count].data[2];
			pos_buffer[1] = cmsg[count].data[3];
			pos_buffer[2] = cmsg[count].data[4];
			pos_buffer[3] = cmsg[count].data[5];
			module[mIndex].position = bit2Float(pos_buffer);

			if(module[mIndex].hold == true){
				EnterCriticalSection(&cSec);
				module[mIndex].mEvent = EVENT_REACHED;
				LeaveCriticalSection(&cSec);
			}
		}
		else if(cmsg[count].data[1] == _CMD_MOV_END){
			// position parsed
			module[mIndex].moveEnd = true; // maybe we need inter thread control here
			module[mIndex].moving = false;
			pos_buffer[0] = cmsg[count].data[2];
			pos_buffer[1] = cmsg[count].data[3];
			pos_buffer[2] = cmsg[count].data[4];
			pos_buffer[3] = cmsg[count].data[5];
			module[mIndex].position = bit2Float(pos_buffer);

			//EnterCriticalSection(&cSec);
			//module[mIndex].mEvent = EVENT_REACHED;
			//LeaveCriticalSection(&cSec);



		}
		else if(cmsg[count].data[1] == _MOV_POS){
			module[mIndex].brake = false;
			module[mIndex].moving = true;
		}
		else if(cmsg[count].data[1] == _MOV_VEL){
			module[mIndex].brake = false;
			module[mIndex].moving = true;
		}
		else if(cmsg[count].data[1] == _CMD_STOP){
			if(cmsg[count].data[2] == 0x4F && cmsg[count].data[3] == 0x4B){
				module[mIndex].brake = true;
				module[mIndex].moving = false;
			}
			else
				return CMD_STOP_FAIL;
		}
		else if(cmsg[count].data[1] == _CMD_ERROR){
			module[mIndex].error = true;
			module[mIndex].errorCode = cmsg[count].data[2];
			// error dealt in the event handler
		}
		else if(cmsg[count].data[1] == _CMD_INFO){
			// deal with error here
		}
		else if(cmsg[count].data[1] == _CMD_ACK){
			if(cmsg[count].data[2] == 0x4F && cmsg[count].data[3] == 0x4B){
				module[mIndex].error = false;
			}
			else
				return ACK_FAIL;
		}
		else if(cmsg[count].data[1] == _REFERENCE){
			if(cmsg[count].data[2] == 0x4F && cmsg[count].data[3] == 0x4B){
				module[mIndex].brake = false;
				module[mIndex].moving = true;
			}
			else
				return REFERENCE_FAIL;
		}
		else{
			cout<<(int)cmsg[count].data[1]<<endl;
			return UNKNOWN_MSG;
		}
		return PARSE_SUCCESS;
	}

}

int SMP::parseFragMsg(int mIndex){
	if(fragLen[mIndex]==3 && fragBuf[mIndex][0].data[2] == _GET_STATE){
		pos_buffer[0] = fragBuf[mIndex][0].data[3];
		pos_buffer[1] = fragBuf[mIndex][0].data[4];
		pos_buffer[2] = fragBuf[mIndex][0].data[5];
		pos_buffer[3] = fragBuf[mIndex][0].data[6];
		module[mIndex].position = bit2Float(pos_buffer);

		vel_buffer[0] = fragBuf[mIndex][0].data[7];
		vel_buffer[1] = fragBuf[mIndex][1].data[2];
		vel_buffer[2] = fragBuf[mIndex][1].data[3];
		vel_buffer[3] = fragBuf[mIndex][1].data[4];
		module[mIndex].velocity = bit2Float(vel_buffer);
			
		cur_buffer[0] = fragBuf[mIndex][1].data[5];
		cur_buffer[1] = fragBuf[mIndex][1].data[6];
		cur_buffer[2] = fragBuf[mIndex][1].data[7];
		cur_buffer[3] = fragBuf[mIndex][2].data[2];
		module[mIndex].current = bit2Float(cur_buffer);

		// check the states

		module[mIndex].referenced = ((fragBuf[mIndex][2].data[3]&CHECK_REFERENCED) == 0) ? false : true;
		module[mIndex].moving = ((fragBuf[mIndex][2].data[3]&CHECK_MOVING) == 0) ? false : true;
		module[mIndex].brake = ((fragBuf[mIndex][2].data[3]&CHECK_BRAKE) == 0) ? false : true;
		module[mIndex].moveEnd = ((fragBuf[mIndex][2].data[3]&CHECK_MOV_END) == 0) ? false : true;
		module[mIndex].posReached = ((fragBuf[mIndex][2].data[3]&CHECK_POS_REACHED) == 0) ? false : true;
		module[mIndex].error = ((fragBuf[mIndex][2].data[3]&CHECK_ERROR) == 0) ? false : true;
		module[mIndex].errorCode = fragBuf[mIndex][2].data[4]; 

		if(module[mIndex].moving == false && module[mIndex].brake == true){
			EnterCriticalSection(&cSec);
			module[mIndex].mEvent = EVENT_REACHED;
			LeaveCriticalSection(&cSec);
		}

		
		return PARSE_SUCCESS;
	}
	else if(fragBuf[mIndex][0].data[2] == _GET_CONFIG){
			// do nothing here, just show we know this message
		return PARSE_SUCCESS;
	}
	else{
		return UNKNOWN_MSG;
	}
}




void SMP::canPolling(){
	bool comm = false;
	bool poll = false;

	// use synchronization to check the shared variables
	EnterCriticalSection(&cSec);
	if (canCommActive == true)
		comm = true;
	else
		comm = false;
	if (canPollActive == true)
		poll = true;
	else
		poll = false;
	LeaveCriticalSection(&cSec);

	while(poll){
		while(comm){
			// functions when communicate
			canMsgPoll();
			// judge whether communicate
			// use synchronization to check shared variable
			EnterCriticalSection(&cSec);
			if (canCommActive == true)
				comm = true;
			else
				comm = false;
			LeaveCriticalSection(&cSec);		
		}
		// use synchronization to check shared variable
		EnterCriticalSection(&cSec);
		if (canPollActive == true)
			poll = true;
		else
			poll = false;
		LeaveCriticalSection(&cSec);		
	}
}

void SMP::eventHandling(){
	bool eHandle = false;
	int eventState = 0;
	EnterCriticalSection(&cSec);
	if (eventActive == true)
		eHandle = true;
	else
		eHandle = false;
	LeaveCriticalSection(&cSec);

	while(eHandle){
		for(int index = 0; index <= 7; index++){			
			EnterCriticalSection(&cSec);
			eventState = module[index].mEvent;
			LeaveCriticalSection(&cSec);

			if (eventState == EVENT_REACHED){
				callBackReached(index);
				EnterCriticalSection(&cSec);
				module[index].mEvent = EVENT_NULL;
				LeaveCriticalSection(&cSec);
			}			
		}

		EnterCriticalSection(&cSec);
		if (eventActive == true)
			eHandle = true;
		else
			eHandle = false;
		LeaveCriticalSection(&cSec);
	}
}

void SMP::SingleEventHandling(){
	
	int eventState = 0;

	for(int index = 0; index <= 7; index++){			
		EnterCriticalSection(&cSec);
		eventState = module[index].mEvent;
		LeaveCriticalSection(&cSec);

		if (eventState == EVENT_REACHED){
			callBackReached(index);
			EnterCriticalSection(&cSec);
			module[index].mEvent = EVENT_NULL;
			LeaveCriticalSection(&cSec);
		}			
	}

}

void SMP::callBackReached(int index){
	// hold	
	if(module[index].error == true){
		errorHandler(index);
	}
	/*if(index ==0){
		MOVE_POS(index, smp_target_list_.at<float>(target_count_[0], 0), smp_target_list_.at<float>(target_count_[0], 1), 20.0, 10.0); 
		target_count_[0]++;
	}*/

	if(index ==3){
		MOVE_POS(index, smp_target_list_.at<float>(target_count_[0], 0), smp_target_list_.at<float>(target_count_[0], 1), 20.0, 6.0);
		target_count_[0]++;
	}
	if(index == 5){
		MOVE_POS(index, smp_target_list_.at<float>(target_count_[1], 2), smp_target_list_.at<float>(target_count_[1], 3), 20.0, 4.0);
		target_count_[1]++;		
	}
}

void SMP::errorHandler(int index){
	uint8_t error = module[index].errorCode;
	switch(error){
		case 0x76: // ERROR_CABLE_BREAK
			CMD_ACK(index);
			if(module[index].movBack == true && index == 0)
				MOVE_POS(index, -20, v0, 10.0, 9.9);
			else if(module[index].movForth == true && index == 0)
				MOVE_POS(index, -40, v0, 10.0, 9.9);
			else if(module[index].movBack == true && index == 3)
				MOVE_POS(index, 100, v3, 20.0, 6);
			else if(module[index].movForth == true && index == 3)
				MOVE_POS(index, 0, v3, 20.0, 6);
			else{
				module[index].hold = false;
				module[index].movBack = false;
				module[index].movForth = false;	
				CMD_STOP(index);
			}


			/*
			
			*/
			break;
		case 0xD9:
			module[index].hold = false;
			module[index].movBack = false;
			module[index].movForth = false;
			CMD_STOP(index);
			break;
		case 0xDA: // ERROR_TOW
			module[index].hold = false;
			module[index].movBack = false;
			module[index].movForth = false;
			CMD_STOP(index);
			break;
		default:
			module[index].hold = false;
			module[index].movBack = false;
			module[index].movForth = false;
			CMD_STOP(index);
	}
}

void SMP::enableCanPoll(){
	EnterCriticalSection(&cSec);
	canPollActive = true;			
	LeaveCriticalSection(&cSec);
}
void SMP::disableCanPoll(){
	EnterCriticalSection(&cSec);
	canPollActive = false;
	LeaveCriticalSection(&cSec);
}
void SMP::enableCanComm(){
	EnterCriticalSection(&cSec);
	canCommActive = true;
	LeaveCriticalSection(&cSec);
}
void SMP::disableCanComm(){
	EnterCriticalSection(&cSec);
	canCommActive = false;
	LeaveCriticalSection(&cSec);
}
void SMP::enableEvent(){
	EnterCriticalSection(&cSec);
	eventActive = true;
	LeaveCriticalSection(&cSec);
}
void SMP::disableEvent(){
	EnterCriticalSection(&cSec);
	eventActive = false;
	LeaveCriticalSection(&cSec);
}

void SMP::enableHold(int index){
	module[index].hold = true;
}
void SMP::disableHold(int index){
	module[index].hold = false;
}

int SMP::GetTargetCount(int index){
	// if(index == 0)
	// 	return target_count_[0];
	if(index == 3)
		return target_count_[0];
	else if(index == 5)
		return target_count_[1];
	else
		return 0;
}

void SMP::SetTargetCount(int index, int count){
	// if(index == 0)
	// 	target_count_[0] = count;
	if(index == 3)
		target_count_[0] = count;
	else if(index == 5)
		target_count_[1] = count;	
}

void SMP::SetTargetList(const cv::Mat& target, int target_length, int num_joints){
	smp_num_joints_ = num_joints;
	smp_target_length_ = target_length;
	target_count_ = new float[smp_num_joints_];
	for(int i = 0; i < smp_num_joints_; i++)
		target_count_[i] = 0;
	smp_target_list_ = cv::Mat::zeros(smp_target_length_, 2 * smp_num_joints_, CV_32F);
	target.copyTo(smp_target_list_);
	
	std::ofstream osp("target_test.txt");
	for(int i=0; i < smp_target_length_; i++){
		for(int j = 0; j < 2 * smp_num_joints_; j++){
			osp << smp_target_list_.at<float>(i, j) <<" ";
		}
		osp<<std::endl;
	}
	osp.close();	
}

//////////////////////////// back up code ///////////////////////////////////
		
// MOVE_VEL(index, 0.0);
	
// v3 = (float)(rand() % 20001) / 10000.0 + 8.0;
// v3 should range from 4 to 12 
// v3 =6.0;
// v3 = (float)(rand() % 76000) / 10000.0 + 4.2;
// v3 = (float)(rand() % 76) / 10.0 + 4.2;
// v3 = 6.0; //5.0; //6.5;
// v0 = 4.0; //5.0;	
		
// v0 = 2.5;
// v3 = 2.5;
// v5 = 3.5;
// v7 = 5.5;
// v3 = 4.0;
// v5 = 2.0;
// v7 = 6.0;

/*	EnterCriticalSection(&cSec);
flagBack = module[index].movBack;
flagForth = module[index].movForth;
LeaveCriticalSection(&cSec); */


/*if(index == 0){
	if(module[index].movBack == true){
		MOVE_POS(index, -20.0, v0, 20.0, 10.0);

		EnterCriticalSection(&cSec);
		module[index].movBack = false;
		module[index].movForth = true;	
		LeaveCriticalSection(&cSec);
	}
	else if(module[index].movForth == true){
		MOVE_POS(index, -40.0, v0, 20.0, 10.0);

		EnterCriticalSection(&cSec);
		module[index].movBack = true;
		module[index].movForth = false;	
		LeaveCriticalSection(&cSec);
	}
}*/

/* if(index ==3){
	if(flagBack == true){
		MOVE_POS(index, 80.0, v3, 20.0, 6.0);

		EnterCriticalSection(&cSec);
		module[index].movBack = false;
		module[index].movForth = true;
		LeaveCriticalSection(&cSec);
	}
	else if(flagForth == true){
		MOVE_POS(index, 60.0, v3, 20.0, 6.0);

		EnterCriticalSection(&cSec);
		module[index].movBack = true;
		module[index].movForth = false;
		LeaveCriticalSection(&cSec);
	}
}

if(index == 5){
	if(module[index].movBack == true){
		MOVE_POS(index, 90.0, v5, 20.0, 4.0);

		EnterCriticalSection(&cSec);
		module[index].movBack = false;
		module[index].movForth = true;	
		LeaveCriticalSection(&cSec);
	}
	else if(module[index].movForth == true){
		MOVE_POS(index, 70.0, v5, 20.0, 4.0);

		EnterCriticalSection(&cSec);
		module[index].movBack = true;
		module[index].movForth = false;	
		LeaveCriticalSection(&cSec);
	}
}*/

/*if(index == 7){
	if(module[index].movBack == true){
		MOVE_POS(index, 50.0, v7, 20.0, 1.8);

		EnterCriticalSection(&cSec);
		module[index].movBack = false;
		module[index].movForth = true;	
		LeaveCriticalSection(&cSec);
	}
	else if(module[index].movForth == true){
		MOVE_POS(index, 10.0, v7, 20.0, 1.8);

		EnterCriticalSection(&cSec);
		module[index].movBack = true;
		module[index].movForth = false;	
		LeaveCriticalSection(&cSec);
	}
}*/
