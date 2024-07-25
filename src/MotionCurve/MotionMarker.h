// MotionMarker.h: interface for the MotionMarker class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_MOTIONMARKER_H__9A52DE58_9121_482A_900D_8F467A1A3D3C__INCLUDED_)
#define AFX_MOTIONMARKER_H__9A52DE58_9121_482A_900D_8F467A1A3D3C__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "PathPointSet.h"
#include "ControlPointSet.h"
#include "DOFName.h"
#include "Vector3D.h"
#include "Matrix.h"
#include "CubicBSplineCurve.h"

#define UNKNOWN_MARKER		17000
#define TRC_MARKER			17001
#define BVH_MARKER			17002

class MotionMarker  
{
public:
	bool ResetAllCurves(unsigned int _numFrames);
	MotionMarker& operator =(const MotionMarker &MK_in);
	bool SetDOFName(unsigned nDOF, char *name_in);
	bool Initialize(unsigned int numDOFs_in, unsigned int numFrames_in, char *name_in, unsigned int type);
	MotionMarker();
	virtual ~MotionMarker();

	//Pointer to name of this marker.
	char name[256];
	//Number of frames of this marker.
	unsigned int numFrames;
	//Number of DOFs for this marker.
	unsigned int numDOFs;
	//Type of this marker.
	unsigned int MarkerType;

	DOFName *CurveName;

	MotionMarker *prev, *next, *parent;
	Vector3D offset;
	Vector3D CurrentPosition;

	Vector3D Velocity, Acceleration;

	Matrix TranMat, LocalRM;
	unsigned int nID;
	unsigned int numCPs;
	bool IsEndSite;

	CubicBSplineCurve *m_cbsc;
};

#endif // !defined(AFX_MOTIONMARKER_H__9A52DE58_9121_482A_900D_8F467A1A3D3C__INCLUDED_)
