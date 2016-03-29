#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "stdafx.h"
#include "Client.h"

struct ThreadParamController
{
	HWND *hwnd;
	HRC *hrc;
	KinectClient *client;

};

class Controller
{
public:
	Controller(LPCWSTR &sFileName, HWND *hwnd);
	~Controller();

	void StartController();

	static void PrintStateMessage(INT nLength, CHAR *cMessage, INT *nStatePos);
	//print message in state area

	static void PrintResultMessage(INT nLength, CHAR *cResult, INT *nResPos);
	//print message in result area

	static void SeverState(ThreadParamController *param);


private:
	CHAR strAddr[256];
	USHORT nPort;
	HWND *hwnd;

	HRC Clock;
	//hrc

	KinectClient *pClient;
	//client class


	ThreadParamController param;


	static void ThreadRefresh(ThreadParamController *param);



};

#endif