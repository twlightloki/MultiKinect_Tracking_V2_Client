#ifndef CLIENT_H
#define CLIENT_H

#include "stdafx.h"
#include "KinectOperation.h"

typedef struct TimingMessage
{
	INT nIndex;
	LLONG nSendTime, nProcTime;
};


typedef struct CalibrationMessage
{
	CameraSpacePoint pSensor;
	DOUBLE fLowDepth, fHighDepth, fMidDepth;
};

typedef struct TrilaterationMessage
{
	LLONG tRecv;
	CameraSpacePoint pBody[BODY_LENGTH];
	INT nShade[BODY_LENGTH];
	DOUBLE fSpeed[BODY_LENGTH];
};

typedef struct SeverBuffer
{
	INT nType;
	CHAR bData[MAX_FRAME_LENGTH];
};

typedef struct ThreadParam
{
	SOCKET *sSever, *sClient;
	BOOL bStopCalibration,bStopThread;
	HRC *hrc;
	KinectOperation *Sensor;
	
	INT *nTimingMessageIndex;
	LLONG *nSeverTime;

};



class KinectClient
{

public:
	KinectClient(HRC *hrc, CHAR *strAddr, USHORT port);
	~KinectClient();

	void InitClient();
	//register the socket and open the port

	static INT DataSend(SOCKET *sClient, void *data, INT nLength, INT nType);
	//proc of data send


	LLONG GetSeverTime();

	BOOL ConnectState();

	BOOL CalibrationState();



private:
	HRC *Clock;
	
	BOOL bSuccInit;
	
	SOCKET sSever, sClient;

	KinectOperation *Sensor;

	ThreadParam param;

	LLONG nSeverTime;
	INT nTimingMessageIndex;
	

	WSADATA wsaData;

	USHORT nPort;
	CHAR strAddr[256];
	SOCKADDR_IN addrSever;
	//adress and port of sever


	void InitAddr(SOCKADDR_IN *addrInput, CHAR *strAddr, ADDRESS_FAMILY family, USHORT port);



	static void ThreadCalibration(ThreadParam *param);

	static void ThreadDataReceive(ThreadParam *param);

	static void StopCalibration(ThreadParam *param);

	static void TimingProc(ThreadParam *param, CHAR *buffer);

	static void ThreadTrilatertaion(ThreadParam *param);

	void TimingProc_Run(ThreadParam *param, CHAR *buffer);


};



#endif