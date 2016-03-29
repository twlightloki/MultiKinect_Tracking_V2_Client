#include "Controller.h"



Controller::Controller(LPCWSTR &sFileName, HWND *hwnd)
{
	//HANDLE fConfig = CreateFile(sFileName, GENERIC_READ, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
	//CHAR bFileBuffer[50];
	//DWORD nRead;

	WCHAR wstrAddr[256];
	GetPrivateProfileString(TEXT("Controller"), TEXT("Address"), TEXT("127.0.0.1"), wstrAddr, 256, sFileName);
	INT nAddrLen = wcslen(wstrAddr);
	WideCharToMultiByte(CP_ACP, 0, wstrAddr, nAddrLen, strAddr, nAddrLen, NULL, NULL);

	//this->nClient = nClient;

	nPort = GetPrivateProfileInt(TEXT("Controller"), TEXT("Port"), 1992, sFileName);



	pClient = new KinectClient(&Clock, strAddr, nPort);

	this->hwnd = hwnd;

}

Controller::~Controller()
{

}


void Controller::StartController()
{
	pClient->InitClient();

	param.hwnd = hwnd;
	param.hrc = &Clock;
	param.client = pClient;
	

	std::thread threadRefresh = std::thread(ThreadRefresh, &param);
	threadRefresh.detach();
}

void Controller::ThreadRefresh(ThreadParamController *param)
{
	CameraSpacePoint pBodyCurrent[BODY_LENGTH];
	BOOL bCalc = FALSE;
	INT nConnected, nProofread;
	TIMETICK nThreadTimeStamp;
	while (TRUE)
	{
		param->hrc->TimeStampStart(&nThreadTimeStamp);

		SeverState(param);
		


		param->hrc->HighResolutionSleep(40, &nThreadTimeStamp);
		//paint

	}
}


void Controller::SeverState(ThreadParamController *param)
{
	HDC hdc;
	WCHAR wstrConnect[20], wstrCalibration[20], wstrOutput[256];
	RECT rect;
	HBRUSH hBrush;
	//init str start
	if (param->client->ConnectState())
		swprintf_s(wstrConnect, 20, TEXT("SeverTime: %dms"),param->client->GetSeverTime());
	else
		swprintf_s(wstrConnect, 20, TEXT("Disconnected"));
	if (param->client->CalibrationState())
		swprintf_s(wstrCalibration, 20, TEXT("Calibrated"));
	else
		swprintf_s(wstrCalibration, 20, TEXT("Discalibrated"));


	//init str end
	//start paint
	hdc = GetDC(*(param->hwnd));	
	SetRect(&rect, 10, 25 , 290, 50);
	hBrush = CreateSolidBrush(RGB(255, 255, 255));
	FillRect(hdc, &rect, hBrush);

	swprintf_s(wstrOutput, 256, TEXT("%s %s"), wstrConnect, wstrCalibration);
	TextOut(hdc, 20, 25, wstrOutput, wcslen(wstrOutput));

	//end paint
	ReleaseDC(*(param->hwnd), hdc);
}