#include "Client.h"

KinectClient::KinectClient(HRC *hrc, CHAR *strAddr, USHORT port)
{
	Clock = hrc;
	INT err = WSAStartup(MAKEWORD(2, 2), &wsaData);
	memcpy(this->strAddr, strAddr, strlen(strAddr));
	this->nPort = port;

	nTimingMessageIndex = 0;
	nSeverTime = 0;

}

KinectClient::~KinectClient()
{
	closesocket(sSever);
	closesocket(sClient);
}

void KinectClient::InitClient()
{
	InitAddr(&addrSever, strAddr, AF_INET, htons(nPort));
	sClient = socket(AF_INET, SOCK_STREAM, 0);
	INT err = connect(sClient, (SOCKADDR*)&(addrSever), sizeof(addrSever));
	if (err != 0)
	{
		bSuccInit = FALSE;
		return;
	}
	
	std::thread threadCalibration, threadDataRecv;

	Sensor = new KinectOperation(Clock);
	if (!Sensor->SuccessInit())
	{
		bSuccInit = FALSE;
		return;
	}

	param.bStopCalibration = FALSE;
	param.bStopThread = FALSE;
	param.sClient = &sClient;
	param.hrc = Clock;
	param.Sensor = Sensor;
	param.nSeverTime = &nSeverTime;
	param.nTimingMessageIndex = &nTimingMessageIndex;

	threadCalibration = std::thread(ThreadCalibration, &param);
	threadCalibration.detach();

	threadDataRecv = std::thread(ThreadDataReceive, &param);
	threadDataRecv.detach();

	bSuccInit = TRUE;
}

INT KinectClient::DataSend(SOCKET *sClient, void *data, INT nLength, INT nType)
{
	SeverBuffer bMessage;
	bMessage.nType = nType;
	memcpy(bMessage.bData, data, nLength);
	INT nSendLen = send(*sClient, (CHAR*)&bMessage, sizeof(bMessage), 0);
	return(nSendLen);
}

void KinectClient::InitAddr(SOCKADDR_IN *addrInput, CHAR *strAddr, ADDRESS_FAMILY family, USHORT port)
{
	inet_pton(AF_INET, strAddr, &(addrInput->sin_addr.S_un.S_addr));
	addrInput->sin_family = family;
	addrInput->sin_port = port;
}

void KinectClient::ThreadCalibration(ThreadParam *param)
{
	CalibrationMessage mCalibraion;
	TIMETICK tCalibrationStamp;
	while (!param->bStopCalibration)
	{
		param->hrc->TimeStampStart(&tCalibrationStamp);
		if (param->Sensor->Calibration_run(&mCalibraion.pSensor))
		{
			DataSend(param->sClient, &mCalibraion, sizeof(CalibrationMessage), CALIBRATION_MESSAGE_SEND);
		}
		param->hrc->HighResolutionSleep(KINECT_FRAME_FREQ * DEPTH_BUFFER_LENGTH * 2, &tCalibrationStamp);
	}
}

void KinectClient::ThreadDataReceive(ThreadParam *param)
{
	INT nRecvLen;
	SeverBuffer bMessage;
	while (!param->bStopThread)
	{
		nRecvLen = recv(*(param->sClient), (CHAR*)&bMessage, sizeof(bMessage), 0);
		switch (bMessage.nType)
		{
		case (CALIBRATION_STOP_MESSAGE) :
		{

			StopCalibration(param);
			break;
		}
		case (TIMING_MESSAGE) :
		{
			TimingProc(param, bMessage.bData);
			break;
		}
		case (HALT_MESSAGE) :
		{
			param->bStopThread = TRUE;
			break;

		}
		default:
		{

			break;
		}
		}
	}
}

void KinectClient::StopCalibration(ThreadParam *param)
{
	param->bStopCalibration = TRUE;
	std::thread threadTriltertaion = std::thread(ThreadTrilatertaion, param);
	threadTriltertaion.detach();
}

void KinectClient::ThreadTrilatertaion(ThreadParam *param)
{
	Joint pJoint[BODY_LENGTH];
	TIMETICK tTimeStamp;
	TrilaterationMessage mCurrentPos;
	while (!param->bStopThread)
	{
		param->hrc->TimeStampStart(&tTimeStamp);
		if (param->Sensor->GetBodyFrame(pJoint, mCurrentPos.nShade, mCurrentPos.fSpeed, &mCurrentPos.tRecv))
		{
			mCurrentPos.tRecv += *param->nSeverTime;
			for (INT i1 = 0; i1 < BODY_LENGTH; i1++)
			{
				mCurrentPos.pBody[i1].X = pJoint[i1].Position.X * 100;
				mCurrentPos.pBody[i1].Y = pJoint[i1].Position.Y * 100;
				mCurrentPos.pBody[i1].Z = pJoint[i1].Position.Z * 100;
			}
			DataSend(param->sClient, &mCurrentPos, sizeof(TrilaterationMessage), TRILATERATION_MESSAGE);
		}
		param->hrc->HighResolutionSleep(KINECT_FRAME_FREQ, &tTimeStamp);
	}

}

void KinectClient::TimingProc(ThreadParam *param, CHAR *buffer)
{
	TimingMessage *mTime = (TimingMessage*)buffer;
	if (mTime->nIndex > *param->nTimingMessageIndex)
	{
		*param->nTimingMessageIndex = mTime->nIndex;

		LLONG nClientTime = param->hrc->HighResolutionTime();

		*param->nSeverTime = mTime->nSendTime - nClientTime;
		//calc the clock difference between client and sever
		mTime->nProcTime = param->hrc->HighResolutionTime() - nClientTime;

		DataSend(param->sClient, mTime, sizeof(TimingMessage), TIMING_MESSAGE);
	}
}

BOOL KinectClient::ConnectState()
{
	return(bSuccInit);
}

BOOL KinectClient::CalibrationState()
{
	return(param.bStopCalibration);
}

LLONG KinectClient::GetSeverTime()
{
	return(nSeverTime);
}