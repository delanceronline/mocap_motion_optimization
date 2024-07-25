// MotionMarker.cpp: implementation of the MotionMarker class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "MotionMarker.h"
#include "string.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

MotionMarker::MotionMarker()
{
	CurveName = NULL;
	prev = NULL;
	next = NULL;
	parent = NULL;
	m_cbsc = NULL;

	numCPs = 0;
	numDOFs = 0;

	nID = 0;

	MarkerType = UNKNOWN_MARKER;
	IsEndSite = false;

	TranMat.ResetDimension(4, 4);
	LocalRM.ResetDimension(4, 4);
}

MotionMarker::~MotionMarker()
{
	if(CurveName != NULL)
	{
		delete[] CurveName;
		CurveName = NULL;
	}

	if(m_cbsc != NULL)
	{
		delete[] m_cbsc;
		m_cbsc = NULL;
	}
}

bool MotionMarker::Initialize(unsigned int numDOFs_in, unsigned int numFrames_in, char *name_in, unsigned int type)
{
	if(m_cbsc != NULL)
		delete[] m_cbsc;

	numDOFs = numDOFs_in;
	numFrames = numFrames_in;
	MarkerType = type;
	CurveName = new DOFName[numDOFs];
	m_cbsc = new CubicBSplineCurve[numDOFs];

	for(unsigned int i = 0; i < numDOFs; i++)
		m_cbsc[i].pps.Reset(numFrames);

	strcpy(name, name_in);

	return true;
}

bool MotionMarker::SetDOFName(unsigned int nDOF, char *name_in)
{
	strcpy(CurveName[nDOF].name, name_in);

	return true;
}

MotionMarker& MotionMarker::operator =(const MotionMarker &MK_in)
{
	if(this == &MK_in)
		return *this;

	prev = NULL;
	next = NULL;
	parent = NULL;

	numFrames = MK_in.numFrames;
	numCPs = MK_in.numCPs;
	numDOFs = MK_in.numDOFs;
	nID = MK_in.nID;

	MarkerType = MK_in.MarkerType;
	IsEndSite = MK_in.IsEndSite;

	offset = MK_in.offset;
	CurrentPosition = MK_in.CurrentPosition;
	IsEndSite = MK_in.IsEndSite;
	strcpy(name, MK_in.name);

	if(CurveName != NULL)
	{
		delete[] CurveName;
		CurveName = NULL;
	}

	if(m_cbsc != NULL)
	{
		delete[] m_cbsc;
		m_cbsc = NULL;
	}

	TranMat.ResetDimension(4, 4);	

	if(numDOFs != 0)
	{
		m_cbsc = new CubicBSplineCurve[numDOFs];
		CurveName = new DOFName[numDOFs];

		for(unsigned int i = 0; i < numDOFs; i++)
		{
			m_cbsc[i] = MK_in.m_cbsc[i];
			strcpy(CurveName[i].name, MK_in.CurveName[i].name);
		}
	}

	return *this;
}

bool MotionMarker::ResetAllCurves(unsigned int _numFrames)
{
	numFrames = _numFrames;

	for(unsigned int i = 0; i < numDOFs; i++)
		m_cbsc[i].pps.Reset(numFrames);

	return true;
}
