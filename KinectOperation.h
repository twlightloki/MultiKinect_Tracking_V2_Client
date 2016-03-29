#ifndef KINECTOPERATION_H
#define KINECTOPERATION_H

#include "stdafx.h"

class KinectOperation
{
public:
	KinectOperation(HRC *hrc);
	~KinectOperation();

	BOOL SuccessInit();

	BOOL Calibration_run(CameraSpacePoint *pAns);

//	BOOL Calibration_run_ver2(CameraSpacePoint *pAns, DOUBLE *fLowDepth, DOUBLE *fHighDepth, DOUBLE *fMidDepth);

	BOOL GetBodyFrame(Joint *pBuffer, INT *nShade, DOUBLE *fSpeed, LLONG *nCurrentTime);




private:
	HRC *Clock;

	BOOL bSuccInit;
	//state of the sensor
	
	CameraSpacePoint pPrevious[BODY_LENGTH];

	LLONG nPreviousFrameTime, nCurrentFrameTime;

	INT nBodySeg[BODY_SEG_AMOUNT][2];

	//use for the trust factor



	std::mutex lSensor;

	IKinectSensor *pKinectSensor;

	IBodyFrameReader *pBodyFrameReader;

	ICoordinateMapper *pCoordinateMapper;

	IDepthFrameReader *pDepthFrameReader;



	BOOL KinectGetDepthFrame(INT *nFrameHeight, INT *nFrameWidth, UINT16 **pBuffer, INT **pStateBuffer);
	//get a depth frame


	void DepthToCamera(DepthSpacePoint *pDepth, UINT16 nDepth, INT nHeight, INT nWidth, CameraSpacePoint *pSpace);
	//coordinate trans

	//main procedure for trust factor

	void TrustFactor(Joint *pCurrent, INT *nShade, DOUBLE *fSpeed);

	void ClearTrustFactor();

	DOUBLE Euclid(const CameraSpacePoint p1, const CameraSpacePoint p2);


	DOUBLE Cross(
		const DOUBLE x1, const DOUBLE y1,
		const DOUBLE x2, const DOUBLE y2
		);


	BOOL SegmentCross(
		const DOUBLE x1, const DOUBLE y1,
		const DOUBLE x2, const DOUBLE y2,
		const DOUBLE x3, const DOUBLE y3,
		const DOUBLE x4, const DOUBLE y4);


	void CrossPointScale(
		DOUBLE *fk1, DOUBLE *fk2,
		const DOUBLE x1, const DOUBLE y1,
		const DOUBLE x2, const DOUBLE y2,
		const DOUBLE x3, const DOUBLE y3,

		const DOUBLE x4, const DOUBLE y4);


	void JointShade(Joint *pCurrent, INT *nShade);

	void SpeedDetect(Joint *pCurrent, DOUBLE *fSpeed);
	// procedure for trust factor
};



#endif