#include <WINDOWS.h>


typedef long long LLONG;
typedef LARGE_INTEGER TIMETICK;

class HRC{

	TIMETICK tInit, tFrequency;

public:
	HRC();

	void HighResolutionSleep(const LLONG lMsc, const TIMETICK *tTimeStamp);

	LLONG HighResolutionTime(LLONG lShift = 0);

	void TimeStampStart(TIMETICK *tTimeStamp);





};