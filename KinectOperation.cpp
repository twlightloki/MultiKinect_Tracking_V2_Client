#include  "KinectOperation.h"

KinectOperation::KinectOperation(HRC *hrc)
{
	HRESULT hr1, hr2;
	hr1 = GetDefaultKinectSensor(&pKinectSensor);
	if (FAILED(hr1))
	{
		bSuccInit = FALSE;
		return;
	}

	if (pKinectSensor)
	{
		IBodyFrameSource *pBodyFrameSource = NULL;
		IDepthFrameSource *pDepthFrameSource = NULL;

		hr1 = pKinectSensor->Open();

		if (SUCCEEDED(hr1))
			hr1 = pKinectSensor->get_CoordinateMapper(&pCoordinateMapper);
		if (SUCCEEDED(hr1))
		{
			hr1 = pKinectSensor->get_BodyFrameSource(&pBodyFrameSource);
			hr2 = pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);
		}

		if (SUCCEEDED(hr1) && SUCCEEDED(hr2))
		{
			hr1 = pBodyFrameSource->OpenReader(&pBodyFrameReader);
			hr2 = pDepthFrameSource->OpenReader(&pDepthFrameReader);
		}
		SafeRelease(pBodyFrameSource);
		SafeRelease(pDepthFrameSource);
	}
	bSuccInit = pKinectSensor&&SUCCEEDED(hr1) || SUCCEEDED(hr2);

	// Initialize the Sensor (depth and body scan)

	Clock = hrc;


	nBodySeg[0][0] = JointType_Head;
	nBodySeg[0][1] = JointType_Neck;
	nBodySeg[1][0] = JointType_Neck;
	nBodySeg[1][1] = JointType_SpineShoulder;
	nBodySeg[2][0] = JointType_SpineShoulder;
	nBodySeg[2][1] = JointType_SpineMid;
	nBodySeg[3][0] = JointType_SpineMid;
	nBodySeg[3][1] = JointType_SpineBase;
	nBodySeg[4][0] = JointType_SpineShoulder;
	nBodySeg[4][1] = JointType_ShoulderRight;
	nBodySeg[5][0] = JointType_SpineShoulder;
	nBodySeg[5][1] = JointType_ShoulderLeft;
	nBodySeg[6][0] = JointType_SpineBase;
	nBodySeg[6][1] = JointType_HipRight;
	nBodySeg[7][0] = JointType_SpineBase;
	nBodySeg[7][1] = JointType_HipLeft;
	nBodySeg[8][0] = JointType_ShoulderRight;
	nBodySeg[8][1] = JointType_ElbowRight;
	nBodySeg[9][0] = JointType_ElbowRight;
	nBodySeg[9][1] = JointType_WristRight;
	nBodySeg[10][0] = JointType_WristRight;
	nBodySeg[10][1] = JointType_HandRight;
	nBodySeg[11][0] = JointType_HandRight;
	nBodySeg[11][1] = JointType_HandTipRight;
	nBodySeg[12][0] = JointType_WristRight;
	nBodySeg[12][1] = JointType_ThumbRight;
	nBodySeg[13][0] = JointType_ShoulderLeft;
	nBodySeg[13][1] = JointType_ElbowLeft;
	nBodySeg[14][0] = JointType_ElbowLeft;
	nBodySeg[14][1] = JointType_WristLeft;
	nBodySeg[15][0] = JointType_WristLeft;
	nBodySeg[15][1] = JointType_HandLeft;
	nBodySeg[16][0] = JointType_HandLeft;
	nBodySeg[16][1] = JointType_HandTipLeft;
	nBodySeg[17][0] = JointType_WristLeft;
	nBodySeg[17][1] = JointType_ThumbLeft;
	nBodySeg[18][0] = JointType_HipRight;
	nBodySeg[18][1] = JointType_KneeRight;
	nBodySeg[19][0] = JointType_KneeRight;
	nBodySeg[19][1] = JointType_AnkleRight;
	nBodySeg[20][0] = JointType_AnkleRight;
	nBodySeg[20][1] = JointType_FootRight;
	nBodySeg[21][0] = JointType_HipLeft;
	nBodySeg[21][1] = JointType_KneeLeft;
	nBodySeg[22][0] = JointType_KneeLeft;
	nBodySeg[22][1] = JointType_AnkleLeft;
	nBodySeg[23][0] = JointType_AnkleLeft;
	nBodySeg[23][1] = JointType_FootLeft;

	ClearTrustFactor();
}

KinectOperation::~KinectOperation()
{
	SafeRelease(pDepthFrameReader);
	if (pKinectSensor)
	{
		pKinectSensor->Close();
	}
	SafeRelease(pKinectSensor);

	//release the sensor
}

BOOL KinectOperation::SuccessInit()
{
	return(bSuccInit);
}

BOOL KinectOperation::Calibration_run(CameraSpacePoint *pAns)
{
	TIMETICK tProofStamp;
	INT nFrameHeight, nFrameWidth;
	lSensor.lock();
	UINT16 *pDepthFrameBuffer[DEPTH_BUFFER_LENGTH];
	INT *pDepthFrameState[DEPTH_BUFFER_LENGTH];
	for (INT i1 = 0; i1 < DEPTH_BUFFER_LENGTH; i1++)
	{
		pDepthFrameBuffer[i1] = NULL;
		Clock->TimeStampStart(&tProofStamp);
		if (!KinectGetDepthFrame(&nFrameHeight, &nFrameWidth, &pDepthFrameBuffer[i1], &pDepthFrameState[i1]))
		{
			for (INT i2=0;i2<=i1;i2++)
				if (pDepthFrameBuffer[i2]!=NULL)
					delete(pDepthFrameBuffer[i2]);
			lSensor.unlock();
			return(FALSE);
		}
		Clock->HighResolutionSleep(KINECT_FRAME_FREQ, &tProofStamp);
	}

	//multi frames used in each calibration procedure

	lSensor.unlock();

	UINT16 *pDepthBufferSum = new UINT16[nFrameHeight*nFrameWidth];
	INT *pDepthStateSum = new INT[nFrameHeight*nFrameWidth];
	memset(pDepthBufferSum, 0, sizeof(UINT16)*nFrameHeight*nFrameWidth);
	memset(pDepthStateSum, 0, sizeof(INT)*nFrameHeight*nFrameWidth);


	for (INT i1 = nFrameHeight*MARK_RANGE; i1 < nFrameHeight*(DOUBLE)(1 - MARK_RANGE); i1++)
		for (INT i2 = nFrameWidth*MARK_RANGE; i2 < nFrameWidth*(DOUBLE)(1 - MARK_RANGE); i2++)
		{
			INT nStatic = 0;
			for (INT i3 = 0; i3 < DEPTH_BUFFER_LENGTH; i3++)
			{
				nStatic += *(pDepthFrameState[i3] + i1*nFrameWidth + i2);
				*(pDepthBufferSum + i1*nFrameWidth + i2) += *(pDepthFrameBuffer[i3] + i1*nFrameWidth + i2)* *(pDepthFrameState[i3] + i1*nFrameWidth + i2);
			}
			if (nStatic>DEPTH_BUFFER_LENGTH*DEPTH_CHECK_THRESHOLD)
				*(pDepthStateSum + i1*nFrameWidth + i2) = 1;
			else
				*(pDepthStateSum + i1*nFrameWidth + i2) = 0;
			if (nStatic!=0)
				*(pDepthBufferSum + i1*nFrameWidth + i2) /= nStatic;
		}
	
	//merge frames

	INT nTopX, nTopYLeft,nTopYRight;
	for (nTopX = nFrameHeight*MARK_RANGE; nTopX < nFrameHeight*(1.0 - MARK_RANGE); nTopX++)
	{
		BOOL bFindTop = FALSE;
		for (nTopYLeft = nFrameWidth*MARK_RANGE; nTopYLeft < nFrameWidth*(1.0 - MARK_RANGE); nTopYLeft++)
			if (*(pDepthStateSum + nTopX*nFrameWidth + nTopYLeft) == 1 && *(pDepthStateSum + (nTopX - 1)*nFrameWidth + nTopYLeft) == 0)
			{
				bFindTop = TRUE;
				break;
			}
		if (bFindTop)
			break;
	}

	for (nTopYRight = nFrameWidth*(1.0 - MARK_RANGE); nTopYRight > nTopYLeft; nTopYRight--)
		if (*(pDepthStateSum + nTopX*nFrameWidth + nTopYRight) == 1 && *(pDepthStateSum + (nTopX - 1)*nFrameWidth + nTopYRight) == 0) break;

	//find the mark

	DepthSpacePoint dpAnswer;
	dpAnswer.X = nTopX;
	dpAnswer.Y = (nTopYLeft + nTopYRight) / 2;
	memset(pAns, 0, sizeof(float)*3);
	
	printf("(%.1f %.1f %.1f)\n", dpAnswer.X, dpAnswer.Y, (DOUBLE)*(pDepthBufferSum + nTopX*nFrameWidth + (INT)dpAnswer.Y));

	DepthToCamera(&dpAnswer, *(pDepthBufferSum + nTopX*nFrameWidth + (INT)dpAnswer.Y), nFrameHeight, nFrameWidth, pAns);
	
	/*pCoordinateMapper->MapDepthPointToCameraSpace(dpAnswer, *(pDepthBufferSum + nTopX*nFrameWidth + (INT)dpAnswer.Y), pAns);
	pAns->X *= 100;
	pAns->Y *= 100;
	pAns->Z *= 100;
	*/
	for (INT i1 = 0; i1 < DEPTH_BUFFER_LENGTH;i1++)
		if (pDepthFrameBuffer[i1] != NULL)
		{
			delete(pDepthFrameBuffer[i1]);
		}
	delete(pDepthBufferSum);
	delete(pDepthStateSum);
	return(TRUE);
}

BOOL KinectOperation::KinectGetDepthFrame(INT *nFrameHeight, INT *nFrameWidth, UINT16 **pBuffer, INT **pStateBuffer)
{
	IDepthFrame *pDepthFrame = NULL;
	HRESULT hr1, hr2;
	hr1 = pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);
	if (SUCCEEDED(hr1))
	{
		UINT nBufferSize = 0;
		UINT16 *pTmpBuffer;
		IFrameDescription *pFrameDescription = NULL;
		hr1 = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &pTmpBuffer);
		hr2 = pDepthFrame->get_FrameDescription(&pFrameDescription);
		if (SUCCEEDED(hr1) && SUCCEEDED(hr2))
		{
			pFrameDescription->get_Height(nFrameHeight);
			pFrameDescription->get_Width(nFrameWidth);

			*pBuffer = new UINT16[*nFrameHeight* *nFrameWidth];
			for (INT i1 = 0; i1 < *nFrameHeight* *nFrameWidth; i1++)
				*(*pBuffer + i1) = *(pTmpBuffer + i1);
		}
	}

	//store the depth data into buffer

	if (!(SUCCEEDED(hr1) && SUCCEEDED(hr2)))
		return(FALSE);
	SafeRelease(pDepthFrame);

	INT nMarkPos; 

	INT nMarkSt = 0, nMarkEn = 0, nMeanDepth = 0, bInMark = 0, nCurrentMarkThresHold = MARK_THRESHOLD;


	while (bInMark<2 && nCurrentMarkThresHold>0)
	{
		nCurrentMarkThresHold -= MARK_THRESHOLD_DEC;
		nMarkPos = *nFrameHeight*MARK_POSITION;
		while (bInMark < 2 && nMarkPos < *nFrameHeight)
		{
			nMarkPos += MARK_POS_STEP;
			/*
			for (INT i1 = 0; i1 < *nFrameWidth; i1++)
			{
				printf("%d ", *(*pBuffer + nMarkPos * *nFrameWidth + i1) / 10);
			}
			printf("\n");
			*/
			UINT16 *nLastDep = *pBuffer + nMarkPos * *nFrameWidth + (INT)(*nFrameWidth*MARK_RANGE);
			INT nStInc = 1;
			while (*nLastDep == 0)
			{
				nLastDep++;
				nStInc++;
			}

			for (nMarkSt = *nFrameWidth * MARK_RANGE + nStInc; nMarkSt < *nFrameWidth * (1.0 - MARK_RANGE); nMarkSt++)
			{
				UINT16 *nCurrent = (*pBuffer + nMarkPos * *nFrameWidth + nMarkSt);

				if (*nCurrent != 0)
				{
					if (*nCurrent + nCurrentMarkThresHold < *nLastDep)
					{
						bInMark++;
						break;
					}
					*nLastDep = *nCurrent;
				}
			}

			nLastDep = *pBuffer + nMarkPos * *nFrameWidth + (INT)(*nFrameWidth*(1.0 - MARK_RANGE));
			nStInc = 1;
			while (*nLastDep == 0)
			{
				nLastDep--;
				nStInc++;
			}

			for (nMarkEn = *nFrameWidth*(1.0 - MARK_RANGE) - nStInc; nMarkEn > nMarkSt; nMarkEn--)
			{
				UINT16 *nCurrent = (*pBuffer + nMarkPos * *nFrameWidth + nMarkEn);

				if (*nCurrent != 0)
				{
					if (*nCurrent + nCurrentMarkThresHold < *nLastDep)
					{
						bInMark++;
						break;
					}
					nLastDep = nCurrent;

				}
			}
		}
	}


	//find the distance between the sensor and the mark

	if (bInMark < 2||nMarkSt>nMarkEn)
		return(FALSE);
	//printf("[%d %d] with Threshold %d and Line %d\n",nMarkSt, nMarkEn,nCurrentMarkThresHold,nMarkPos);

	
	for (INT i1 = nMarkSt; i1 <= nMarkEn; i1++)
		nMeanDepth += *(*pBuffer + nMarkPos * *nFrameWidth + i1);
	nMeanDepth = nMeanDepth / (nMarkEn - nMarkSt + 1) + MARK_RADIUS;
	
	//nMeanDepth = nCurrentMarkThresHold - MARK_THRESHOLD_DEC;

	*pStateBuffer = new INT[*nFrameHeight* *nFrameWidth];
	for (INT i1 = 0; i1 < *nFrameHeight; i1++)
		for (INT i2 = 0; i2 < *nFrameWidth; i2++)
		{
			if (*(*pBuffer + i1* *nFrameWidth + i2) == 0 || *(*pBuffer + i1* *nFrameWidth + i2)>nMeanDepth)
				*(*pStateBuffer + i1* *nFrameWidth + i2) = 0;
			else
				*(*pStateBuffer + i1* *nFrameWidth + i2) = 1;
		}

	//binaryzation
	return(TRUE);
}




void KinectOperation::DepthToCamera(DepthSpacePoint *pDepth, UINT16 nDepth, INT nHeight, INT nWidth, CameraSpacePoint *pSpace)
{
	pSpace->Z = (DOUBLE)nDepth/10;
	pSpace->X = (DOUBLE)nDepth*tan((0.5 - pDepth->X / nHeight)*DEPTHVERTICAL_RADIANS) / 10;
	pSpace->Y = (DOUBLE)nDepth*tan((0.5 - pDepth->Y / nWidth)*DEPTHHORIZONTAL_RADIANS)/10;
}




BOOL KinectOperation::GetBodyFrame(Joint *pBuffer, INT *nShade, DOUBLE *fSpeed, LLONG *nCurrentTime)
{
	IBodyFrame* pBodyFrame = NULL;

	HRESULT hr = pBodyFrameReader->AcquireLatestFrame(&pBodyFrame);

	BOOL bSucc = FALSE;

	if (SUCCEEDED(hr))
	{

		IBody* pBodies[BODY_COUNT] = { 0 };

		if (SUCCEEDED(hr))
		{
			hr = pBodyFrame->GetAndRefreshBodyData(_countof(pBodies), pBodies);
		}

		if (SUCCEEDED(hr))
		{
			BOOLEAN bTracked = FALSE;
			INT iSucc;
			for (iSucc = 0; iSucc < BODY_COUNT; iSucc++)
			{
				hr = pBodies[iSucc]->get_IsTracked(&bTracked);
				if (SUCCEEDED(hr) && bTracked)
					break; 
			}
			if (SUCCEEDED(hr) && bTracked)
			{
				pBodies[iSucc]->GetJoints(JointType_Count, pBuffer);
				nCurrentFrameTime = Clock->HighResolutionTime();
				TrustFactor(pBuffer, nShade, fSpeed);
				*nCurrentTime = nCurrentFrameTime;
				bSucc = TRUE;
			}
			else
				ClearTrustFactor();
		}

		for (int i = 0; i < _countof(pBodies); ++i)
		{
			SafeRelease(pBodies[i]);
		}
	}

	SafeRelease(pBodyFrame);

	return(bSucc);
}

void KinectOperation::TrustFactor(Joint *pCurrent, INT *nShade, DOUBLE *fSpeed)
{
	memset(nShade, 0, sizeof(INT)*BODY_LENGTH);
	memset(fSpeed, 0, sizeof(DOUBLE)*BODY_LENGTH);
	if (nPreviousFrameTime!=-1)
	{
		JointShade(pCurrent, nShade);
		SpeedDetect(pCurrent, fSpeed);
	}



	nPreviousFrameTime = nCurrentFrameTime;
	for (INT i1 = 0; i1 < BODY_LENGTH; i1++)
	{
		pPrevious[i1] = (pCurrent + i1)->Position;
	}
	//refresh
}

void KinectOperation::ClearTrustFactor()
{
	nPreviousFrameTime = -1;
	//to set the value to -1 means the trust factor process restart
}

DOUBLE KinectOperation::Euclid(const CameraSpacePoint p1, const CameraSpacePoint p2)
{
	return(sqrt(pow(p1.X-p2.X,2)+
		pow(p1.Y-p2.Y,2)+
		pow(p1.Z-p2.Z,2)));

}

void KinectOperation::JointShade(Joint *pCurrent, INT *nShade)
{
	for (INT i1 = 0; i1 < BODY_SEG_AMOUNT; i1++)
	{
		for (INT i2 = i1 + 1; i2 < BODY_SEG_AMOUNT; i2++)
		{
			if (SegmentCross(
				(pCurrent + nBodySeg[i1][0])->Position.X, (pCurrent + nBodySeg[i1][0])->Position.Y,
				(pCurrent + nBodySeg[i1][1])->Position.X, (pCurrent + nBodySeg[i1][1])->Position.Y,
				(pCurrent + nBodySeg[i2][0])->Position.X, (pCurrent + nBodySeg[i2][0])->Position.Y,
				(pCurrent + nBodySeg[i2][1])->Position.X, (pCurrent + nBodySeg[i2][1])->Position.Y)
				)
			{
				DOUBLE fk1, fk2;
				CrossPointScale(
					&fk1, &fk2,
					(pCurrent + nBodySeg[i1][0])->Position.X, (pCurrent + nBodySeg[i1][0])->Position.Y,
					(pCurrent + nBodySeg[i1][1])->Position.X, (pCurrent + nBodySeg[i1][1])->Position.Y,
					(pCurrent + nBodySeg[i2][0])->Position.X, (pCurrent + nBodySeg[i2][0])->Position.Y,
					(pCurrent + nBodySeg[i2][1])->Position.X, (pCurrent + nBodySeg[i2][1])->Position.Y);

				DOUBLE fDep1 = ((pCurrent + nBodySeg[i1][1])->Position.Z - (pCurrent + nBodySeg[i1][0])->Position.Z)*fk1 + (pCurrent + nBodySeg[i1][0])->Position.Z;
				DOUBLE fDep2 = ((pCurrent + nBodySeg[i2][1])->Position.Z - (pCurrent + nBodySeg[i2][0])->Position.Z)*fk1 + (pCurrent + nBodySeg[i2][0])->Position.Z;
				if (fDep1 > fDep2)
				{
					*(nShade + nBodySeg[i1][0]) += 1;
					*(nShade + nBodySeg[i1][1]) += 1;
				}
				else
				{
					*(nShade + nBodySeg[i2][0]) += 1;
					*(nShade + nBodySeg[i2][1]) += 1;

				}
			}
		}
	}
}


void KinectOperation::SpeedDetect(Joint *pCurrent, DOUBLE *fSpeed)
{
	for (INT i1 = 0; i1 < BODY_LENGTH; i1++)
	{
		*(fSpeed + i1) = Euclid((pCurrent + i1)->Position, pPrevious[i1]) / (DOUBLE)(nCurrentFrameTime - nPreviousFrameTime) * 100000;
	}
}

DOUBLE KinectOperation::Cross(
	const DOUBLE x1, const DOUBLE y1,
	const DOUBLE x2, const DOUBLE y2
	)
{
	return(x1*y2 - x2*y1);
}

BOOL KinectOperation::SegmentCross(
	const DOUBLE x1, const DOUBLE y1, 
	const DOUBLE x2, const DOUBLE y2, 
	const DOUBLE x3, const DOUBLE y3,
	const DOUBLE x4, const DOUBLE y4)
{
	if (min(x1, x2) <= max(x3, x4) &&
		min(x3, x4) <= max(x1, x2) &&
		min(y1, y2) <= max(y3, y4) &&
		min(y3, y4) <= max(y1, y2))
	{
		if (Cross(x1 - x3, y1 - y3, x4 - x3, y4 - y3)*Cross(x2 - x3, y2 - y3, x4 - x3, y4 - y3) < 0 &&
			Cross(x3 - x1, y3 - y1, x2 - x1, y2 - y1)*Cross(x4 - x1, y4 - y1, x2 - x1, y2 - y1) < 0)
		{
			return(TRUE);
		}
	}
	return(FALSE);
}

void KinectOperation::CrossPointScale(
	DOUBLE *fk1, DOUBLE *fk2,
	const DOUBLE x1, const DOUBLE y1,
	const DOUBLE x2, const DOUBLE y2,
	const DOUBLE x3, const DOUBLE y3,
	const DOUBLE x4, const DOUBLE y4)
{
	DOUBLE fDx1, fDx2, fDy1, fDy2, fSx, fSy;
	fDx1 = x2 - x1;
	fDx2 = x4 - x3;
	fDy1 = y2 - y1;
	fDy2 = y4 - y3;
	fSx = x1 - x3;
	fSy = y1 - y3;
	*fk1 = (fSy*fDx2 - fSx*fDy2) / (fDx1*fDy2 - fDx2*fDy1);
	*fk2 = (fSx*fDy1 - fSy*fDx1) / (fDx2*fDy1 - fDx1*fDy2);
}




int main()
{
	HRC newhrc;
	KinectOperation ko = KinectOperation(&newhrc);
	while (TRUE)
	{
		CameraSpacePoint ans;
		if (ko.Calibration_run(&ans))
			printf("<%.2f %.2f %.2f> \n", ans.X, ans.Y, ans.Z);
		else
			printf("failed\n");
		CHAR tmp;
		scanf_s("%c", &tmp);
	}
}

/*

BOOL KinectOperation::Calibration_run_ver2(CameraSpacePoint *pAns, DOUBLE *fLowDepth, DOUBLE *fHighDepth, DOUBLE *fMidDepth)
{
	TIMETICK tProofStamp;
	INT nFrameHeight, nFrameWidth;
	lSensor.lock();
	UINT16 *pDepthFrameBuffer[DEPTH_BUFFER_LENGTH];
	INT *pDepthFrameState[DEPTH_BUFFER_LENGTH];
	for (INT i1 = 0; i1 < DEPTH_BUFFER_LENGTH; i1++)
	{
		Clock->TimeStampStart(&tProofStamp);
		if (!KinectGetDepthFrame(&nFrameHeight, &nFrameWidth, &pDepthFrameBuffer[i1], &pDepthFrameState[i1]))
		{
			delete(pDepthFrameBuffer);
			lSensor.unlock();
			return(FALSE);
		}
		Clock->HighResolutionSleep(KINECT_FRAME_FREQ, &tProofStamp);
	}

	//multi frames used in each calibration procedure

	lSensor.unlock();

	UINT16 *pDepthBufferSum = new UINT16[nFrameHeight*nFrameWidth];
	INT *pDepthStateSum = new INT[nFrameHeight*nFrameWidth];
	memset(pDepthBufferSum, 0, sizeof(UINT16)*nFrameHeight*nFrameWidth);
	memset(pDepthStateSum, 0, sizeof(INT)*nFrameHeight*nFrameWidth);


	for (INT i1 = nFrameHeight*MARK_RANGE; i1 < nFrameHeight*(DOUBLE)(1 - MARK_RANGE); i1++)
		for (INT i2 = nFrameWidth*MARK_RANGE; i2 < nFrameWidth*(DOUBLE)(1 - MARK_RANGE); i2++)
		{
			INT nStatic = 0;
			for (INT i3 = 0; i3 < DEPTH_BUFFER_LENGTH; i3++)
			{
				nStatic += *(pDepthFrameState[i3] + i1*nFrameWidth + i2);
				*(pDepthBufferSum + i1*nFrameWidth + i2) += *(pDepthFrameBuffer[i3] + i1*nFrameWidth + i2)* *(pDepthFrameState[i3] + i1*nFrameWidth + i2);
			}
			if (nStatic>DEPTH_BUFFER_LENGTH*DEPTH_CHECK_THRESHOLD)
				*(pDepthStateSum + i1*nFrameWidth + i2) = 1;
			else
				*(pDepthStateSum + i1*nFrameWidth + i2) = 0;
			if (nStatic != 0)
				*(pDepthBufferSum + i1*nFrameWidth + i2) /= nStatic;
		}

	//merge frames

	INT nTopX, nTopYLeft, nTopYRight;
	for (nTopX = nFrameHeight*MARK_RANGE; nTopX < nFrameHeight*(1.0 - MARK_RANGE); nTopX++)
	{
		BOOL bFindTop = FALSE;
		for (nTopYLeft = nFrameWidth*MARK_RANGE; nTopYLeft < nFrameWidth*(1.0 - MARK_RANGE); nTopYLeft++)
			if (*(pDepthStateSum + nTopX*nFrameWidth + nTopYLeft) == 1 && *(pDepthStateSum + (nTopX - 1)*nFrameWidth + nTopYLeft) == 0)
			{
				bFindTop = TRUE;
				break;
			}
		if (bFindTop)
			break;
	}

	for (nTopYRight = nFrameWidth*(1.0 - MARK_RANGE); nTopYRight > nTopYLeft; nTopYRight--)
		if (*(pDepthStateSum + nTopX*nFrameWidth + nTopYRight) == 1 && *(pDepthStateSum + (nTopX - 1)*nFrameWidth + nTopYRight) == 0) break;

	//find the mark

	DepthSpacePoint dpAnswer;
	dpAnswer.X = nTopX;
	dpAnswer.Y = (nTopYLeft + nTopYRight) / 2;
	memset(pAns, 0, sizeof(CameraSpacePoint));

	printf("(%.1f %.1f %.1f)\n", dpAnswer.X, dpAnswer.Y, (DOUBLE)*(pDepthBufferSum + nTopX*nFrameWidth + (INT)dpAnswer.Y));

	DepthToCamera(&dpAnswer, *(pDepthBufferSum + nTopX*nFrameWidth + (INT)dpAnswer.Y), nFrameHeight, nFrameWidth, pAns);

	pCoordinateMapper->MapDepthPointToCameraSpace(dpAnswer, *(pDepthBufferSum + nTopX*nFrameWidth + (INT)dpAnswer.Y), pAns);
	pAns->X *= 100;
	pAns->Y *= 100;
	pAns->Z *= 100;
	

	*fLowDepth = MARK_THRESHOLD;
	*fHighDepth = 0;
	INT nDepthBufferLen = 0;
	UINT16 *nDepthBuffer = new UINT16[nFrameHeight*nFrameWidth];
	for (INT i1 = nFrameHeight * MARK_RANGE; i1 < nFrameHeight * (1.0 - MARK_RANGE); i1++)
	{
		for (INT i2 = nFrameWidth * MARK_RANGE; i2 < nFrameWidth * (1.0 - MARK_RANGE); i2++)
		{
			if (*(pDepthStateSum + i1*nFrameWidth + i2) == 1)
			{
				nDepthBuffer[nDepthBufferLen++] = *(pDepthStateSum + i1*nFrameWidth + i2);

				if ((DOUBLE)*(pDepthBufferSum + i1*nFrameWidth + i2) / 10 > *fLowDepth)
				{
					*fLowDepth = (DOUBLE)*(pDepthBufferSum + i1*nFrameWidth + i2) / 10;
				}
				if ((DOUBLE)*(pDepthBufferSum + i1*nFrameWidth + i2) / 10 < *fHighDepth)
				{
					*fHighDepth = (DOUBLE)*(pDepthBufferSum + i1*nFrameWidth + i2) / 10;
				}
			}
		}
	}

	std::qsort(nDepthBuffer, nDepthBufferLen, sizeof(UINT16), ICompare);
	*fMidDepth = (DOUBLE)nDepthBuffer[nDepthBufferLen / 2] / 10;

	delete(nDepthBuffer);
	delete(pDepthFrameBuffer);
	delete(pDepthBufferSum);
	delete(pDepthStateSum);
	return(TRUE);
}

static INT ICompare(const void *i1, const void *i2)
{
	return(*(UINT16*)i1 > *(UINT16*)i2);
}
*/

/*
int main()
{
	HRC newhrc;
	KinectOperation ko = KinectOperation(&newhrc);
	DOUBLE x1, x2, x3, x4, y1, y2, y3, y4;
	scanf_s("%lf %lf %lf %lf %lf %lf %lf %lf", &x1, &y1, &x2, &y2, &x3, &y3, &x4, &y4);
	if (ko.SegmentCross(x1, y1, x2, y2, x3, y3, x4, y4))
	{
		DOUBLE k1, k2;
		ko.CrossPointScale(&k1, &k2, x1, y1, x2, y2, x3, y3, x4, y4);
		printf("%.1f %.1f\n", k1, k2);
	}
	else
		printf("none\n");
}*/