// Cylinder.h: interface for the Cylinder class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_CYLINDER_H__F19E7695_45A6_41EE_8E2C_C8A636FC2458__INCLUDED_)
#define AFX_CYLINDER_H__F19E7695_45A6_41EE_8E2C_C8A636FC2458__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "MotionCurve\Vector3D.h"

class Cylinder  
{
public:
	bool SaveCylinder(char *filename);
	bool LoadCylinder(char *filename);
	Vector3D GetWristAbsPos(unsigned int nFrame);
	Vector3D GetElbowAbsPos(unsigned int nFrame);
	Cylinder();

	Vector3D r1, r2;
	Vector3D *pVecWrist, *pVecElbow;
	float *pWristRadius, *pWristHeight, *pElbowRadius, *pElbowHeight;
	unsigned int numPoints;

	virtual ~Cylinder();

};

#endif // !defined(AFX_CYLINDER_H__F19E7695_45A6_41EE_8E2C_C8A636FC2458__INCLUDED_)
