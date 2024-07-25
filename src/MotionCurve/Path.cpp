// Path.cpp: implementation of the Path class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "Path.h"
#include <string.h>
#include <stdio.h>

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

Path::Path()
{
	pInternalPathMD = NULL;
	pKeyFrame = NULL;
	Duration = 0.0f;
	FrameTime = 0.0f;
	numKeyFrames = 0;
}

Path::~Path()
{
	if(pKeyFrame != NULL)
		delete[] pKeyFrame;

	if(pInternalPathMD != NULL)
		delete pInternalPathMD;
}

KeyFrame * Path::ImportMotionPath(const char *filename, unsigned int *p_numFrames)
{
	FILE *stream;

	stream = fopen(filename, "r");
	if(stream != NULL)
	{
		char line[4096];
		unsigned int i, var;

		fgets(line, 4096, stream);
		if(strcmp(line, "METPATHFILE1.0") == 0)
			return NULL;
	
		fgets(line, 4096, stream);
		sscanf(line, "Frame Time: %f", &FrameTime);		

		fgets(line, 4096, stream);
		sscanf(line, "Duration: %f", &Duration);

		while(!feof(stream))
		{
			fgets(line, 4096, stream);
			numKeyFrames++;
		}
		numKeyFrames--;

		*p_numFrames = numKeyFrames;

		if(numKeyFrames < 1)
			return NULL;

		fseek(stream, 0, 0);
		fgets(line, 4096, stream);
		fgets(line, 4096, stream);
		fgets(line, 4096, stream);

		pKeyFrame = new KeyFrame[numKeyFrames];

		for(i = 0; i < numKeyFrames; i++)
		{
			fgets(line, 4096, stream);
			sscanf(line, "%d %f %f %f %f", &var, &pKeyFrame[i].Val.x, &pKeyFrame[i].Val.y, &pKeyFrame[i].Val.z, &pKeyFrame[i].t);
		}

		if(!InitializeInternalPathMD())
			return NULL;
	}
	else
		return NULL;

	return pKeyFrame;
}

bool Path::ExportBVHMotionPath(const char *filename)
{
	if(pKeyFrame != NULL)
	{
		FILE *stream;

		stream = fopen(filename, "w");
		if(stream != NULL)
		{
			MotionMarker *pMarker = NULL;
			unsigned int i;

			fprintf(stream, "METPATHFILE1.0\n");
			fprintf(stream, "FrameTime: %f\n", FrameTime);
			fprintf(stream, "Duration: %f\n", Duration);
			
			for(i = 0; i < numKeyFrames; i++)
				fprintf(stream, "%d %f %f %f %f\n", i + 1, pKeyFrame[i].Val.x, pKeyFrame[i].Val.y, pKeyFrame[i].Val.z, pKeyFrame[i].t);
		}
		fclose(stream);
	}
	else
		return false;

	return true;
}

bool Path::ImportMotionPathFromBuffer(MotionData *pMD)
{
	if(pMD == NULL || pMD->numFrames == 0 || pMD->numMarkers == 0)
		return false;

	unsigned int i;
	MotionMarker *pMarker;

	FrameTime = pMD->FrameTime;
	Duration = (float)(pMD->numFrames - 1) * FrameTime;
	numKeyFrames = pMD->numFrames;

	pKeyFrame = new KeyFrame[numKeyFrames];

	// Find root marker.
	for(i = 0; i < pMD->numMarkers; i++)
	{
		pMarker = pMD->GetMarker(i);
			
		if(pMarker == NULL)
			return false;

		if(pMarker->parent == NULL)
			break;
	}

	for(i = 0; i < pMD->numFrames; i++)
	{
		pKeyFrame[i].Val.x = pMarker->m_cbsc[0].pps.pp[i].GetAmpVal();
		pKeyFrame[i].Val.y = pMarker->m_cbsc[1].pps.pp[i].GetAmpVal();
		pKeyFrame[i].Val.z = pMarker->m_cbsc[2].pps.pp[i].GetAmpVal();

		pKeyFrame[i].t = pMarker->m_cbsc[0].pps.pp[i].GetTimeVal();
	}

	if(!InitializeInternalPathMD())
		return false;

	return true;
}

MotionData * Path::ExportMotionPathToBuffer(void)
{
	if(pKeyFrame == NULL || pInternalPathMD == NULL)
		return NULL;

	//unsigned int i;
	MotionData *pMD = new MotionData;

	/*
	pMD->m_MotionDataType = BVH_MOTION_TYPE;
	pMD->numMarkers = 1;
	pMD->numFrames = numKeyFrames;
	pMD->numPPs = numKeyFrames;

	pMD->FrameTime = FrameTime;
	pMD->TimeLength = Duration;

	pMD->pMarkerHead = new MotionMarker;
	pMD->pMarkerHead->Initialize(3, numKeyFrames, "MotionPath", BVH_MARKER);
	
	for(i = 0; i < pMD->numFrames; i++)
	{
		pMD->pMarkerHead->m_cbsc[0].pps.pp[i].SetAmpVal(pKeyFrame[i].Val.x);
		pMD->pMarkerHead->m_cbsc[0].pps.pp[i].SetTimeVal(pKeyFrame[i].t);

		pMD->pMarkerHead->m_cbsc[1].pps.pp[i].SetAmpVal(pKeyFrame[i].Val.y);
		pMD->pMarkerHead->m_cbsc[1].pps.pp[i].SetTimeVal(pKeyFrame[i].t);

		pMD->pMarkerHead->m_cbsc[2].pps.pp[i].SetAmpVal(pKeyFrame[i].Val.z);
		pMD->pMarkerHead->m_cbsc[2].pps.pp[i].SetTimeVal(pKeyFrame[i].t);
	}
*/
	*pMD = *pInternalPathMD;

	return pMD;
}

bool Path::InitializeInternalPathMD()
{
	if(pKeyFrame == NULL)
		return false;

	unsigned int i;
	pInternalPathMD = new MotionData;

	pInternalPathMD->m_MotionDataType = BVH_MOTION_TYPE;
	pInternalPathMD->numMarkers = 1;
	pInternalPathMD->numFrames = numKeyFrames;
	pInternalPathMD->numPPs = numKeyFrames;

	pInternalPathMD->FrameTime = FrameTime;
	pInternalPathMD->TimeLength = Duration;

	pInternalPathMD->pMarkerHead = new MotionMarker;
	pInternalPathMD->pMarkerHead->Initialize(3, numKeyFrames, "MotionPath", BVH_MARKER);
	
	for(i = 0; i < pInternalPathMD->numFrames; i++)
	{
		pInternalPathMD->pMarkerHead->m_cbsc[0].pps.pp[i].SetAmpVal(pKeyFrame[i].Val.x);
		pInternalPathMD->pMarkerHead->m_cbsc[0].pps.pp[i].SetTimeVal(pKeyFrame[i].t);

		pInternalPathMD->pMarkerHead->m_cbsc[1].pps.pp[i].SetAmpVal(pKeyFrame[i].Val.y);
		pInternalPathMD->pMarkerHead->m_cbsc[1].pps.pp[i].SetTimeVal(pKeyFrame[i].t);

		pInternalPathMD->pMarkerHead->m_cbsc[2].pps.pp[i].SetAmpVal(pKeyFrame[i].Val.z);
		pInternalPathMD->pMarkerHead->m_cbsc[2].pps.pp[i].SetTimeVal(pKeyFrame[i].t);
	}

	pInternalPathMD->knotstep = 1;
	if(!pInternalPathMD->FitAllMarkersToCBSC())
		return false;

	return true;
}
