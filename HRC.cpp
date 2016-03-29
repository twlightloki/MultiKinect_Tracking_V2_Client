#include <Windows.h>
#include "HRC.h"
#include <iostream>

HRC::HRC()
{
	QueryPerformanceFrequency(&tFrequency);
	tFrequency.QuadPart /= 1000;
	QueryPerformanceCounter(&tInit);
}

void HRC::TimeStampStart(TIMETICK *tTimeStamp)
{
	QueryPerformanceCounter(tTimeStamp);
}

void HRC::HighResolutionSleep(const LLONG lMsc, const TIMETICK *tTimeStamp)
{
	TIMETICK lBegin, lEnd;
	LLONG lTemp;
	if (tTimeStamp != NULL)
	{
		lBegin = *tTimeStamp;
	}
	else
	{
		QueryPerformanceCounter(&lBegin);
	}
	do
	{
		QueryPerformanceCounter(&lEnd);
		lTemp = (lEnd.QuadPart - lBegin.QuadPart)  / tFrequency.QuadPart;
	} while (lTemp < lMsc);
}

LLONG HRC::HighResolutionTime(LLONG lShift)
{
	TIMETICK lTime;
	QueryPerformanceCounter(&lTime);
	return((lTime.QuadPart - tInit.QuadPart) / tFrequency.QuadPart + lShift);
}
/*
int main()
{
	HRC hClock;
	hClock.HighResolutionSleep(1500, NULL);
	TIMETICK tStamp1;
	hClock.TimeStampStart(&tStamp1);
	std::cout << hClock.HighResolutionTime() << std::endl;
	for (long i = 0; i < 50; i++){
		hClock.HighResolutionSleep(1, NULL);
		//Sleep(3);
		//hClock.HighResolutionSleep(6000, &tStamp1);
		std::cout << hClock.HighResolutionTime() << std::endl;

	}
	system("pause");
	
}*/