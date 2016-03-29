#include "stdafx.h"|
#include "Controller.h"

#pragma comment( lib, "ws2_32.lib"  )

LRESULT CALLBACK WndProc(HWND, UINT, WPARAM, LPARAM);

void ThreadProc();

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, PSTR szCmdLine, int iCmdShow){
	static TCHAR szAppName[] = TEXT("MultiKinectTracking_Client");
	HWND hwnd;
	MSG msg;
	WNDCLASS wndclass;
	wndclass.style = CS_HREDRAW | CS_VREDRAW;
	wndclass.cbClsExtra = 0;
	wndclass.cbWndExtra = 0;
	wndclass.hInstance = hInstance;
	wndclass.hIcon = LoadIcon(NULL, IDI_APPLICATION);
	wndclass.hCursor = LoadCursor(NULL, IDC_ARROW);
	wndclass.hbrBackground = (HBRUSH)GetStockObject(WHITE_BRUSH);
	wndclass.lpszMenuName = NULL;
	wndclass.lpfnWndProc = WndProc;
	wndclass.lpszClassName = szAppName;
	if (!RegisterClass(&wndclass)){
		MessageBox(NULL, TEXT("Register ERROR!"), szAppName, MB_ICONINFORMATION);
		return(0);
	}
	hwnd = CreateWindow(szAppName,
		TEXT("MultiKinectTracking_Client"),
		WS_OVERLAPPEDWINDOW & ~WS_MAXIMIZEBOX & ~WS_THICKFRAME,
		CW_USEDEFAULT,
		CW_USEDEFAULT,
		WINDOW_WIDTH,
		WINDOW_HEIGHT,
		NULL,
		NULL,
		hInstance,
		NULL);
	ShowWindow(hwnd, iCmdShow);
	UpdateWindow(hwnd);

	//init values

	//end init

	LPCWSTR sFileName = TEXT(".\\config.inf");
	Controller controller = Controller(sFileName, &hwnd);
	controller.StartController();


	while (GetMessage(&msg, NULL, 0, 0)){
		TranslateMessage(&msg);
		DispatchMessage(&msg);
	}
	return(msg.wParam);
}

void PaintWindow(HWND hwnd)
{
	HDC hdc;
	PAINTSTRUCT ps;
	RECT rect;
	InvalidateRect(hwnd, NULL, TRUE);
	hdc = BeginPaint(hwnd, &ps);
	GetClientRect(hwnd, &rect);

	//paint start

	LPCWSTR strsubTittle1 = TEXT("Sever State");
	LPCWSTR strsubTittle2 = TEXT("Results");

	TextOut(hdc, 50, 5, strsubTittle1, wcslen(strsubTittle1));
	TextOut(hdc, 475, 5, strsubTittle2, wcslen(strsubTittle2));

	MoveToEx(hdc, 300, 0, NULL);
	LineTo(hdc, 300, WINDOW_HEIGHT);

	MoveToEx(hdc, 0, 300, NULL);
	LineTo(hdc, 300, 300);

	MoveToEx(hdc, 0, 0, NULL);
	LineTo(hdc, WINDOW_WIDTH, 0);


	//paint end

	EndPaint(hwnd, &ps);
};

/*
void RefreshData(HWND hwnd)
{
HDC hdc;
hdc = GetDC(hwnd);

//refresh lines here

ReleaseDC(hwnd, hdc);
}
*/


LRESULT CALLBACK WndProc(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	HDC hdc;
	PAINTSTRUCT ps;
	RECT rect;
	switch (message)
	{
	case WM_CREATE:
	{
		return(0);
	}
	case WM_PAINT:
	{
		PaintWindow(hwnd);
		return(0);
	}
	case SC_MAXIMIZE:
	{
		PaintWindow(hwnd);
		return(0);
	}
	case WM_TIMER:
	{

		return(0);
	}
	case WM_DESTROY:
	{
		PostQuitMessage(0);
		return(0);
	}
	}
	return DefWindowProc(hwnd, message, wParam, lParam);
}