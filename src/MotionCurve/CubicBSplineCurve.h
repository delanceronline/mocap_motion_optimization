// CubicBSplines.h: interface for the CubicBSplines class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_CUBICBSPLINECURVE_H__986ECA0A_F4A0_4984_AFE3_176700D13903__INCLUDED_)
#define AFX_CUBICBSPLINECURVE_H__986ECA0A_F4A0_4984_AFE3_176700D13903__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "Matrix.h"
#include "PathPointSet.h"
#include "ControlPointSet.h"
#include "ControlPoint.h"	// Added by ClassView
#include "PathPoint.h"	// Added by ClassView

class CubicBSplineCurve
{
public:
	bool UnFitCurve(void);
	unsigned int numPPs, numCPs;
	float GetMinAmp(void);
	float GetMaxAmp(void);
	bool FindMaxAndMinPPAmp(void);

	//Coefficient matrix.
	//Matrix *coef_mat, *Inverse_mat;
	Matrix *pInverse_mat;
	
	//Get interpolated value of curve at given time.
	//Tell this object to calculate / update all control points by using all path points.
	bool CalculateCPFromPP(unsigned int numCPs, unsigned int knotstep);
	//Get specified path point already in this object.
	//Get specified control point already in this object.

	CubicBSplineCurve& operator =(const CubicBSplineCurve &CBSC_in);

	CubicBSplineCurve();
	virtual ~CubicBSplineCurve();

	PathPointSet pps;
	ControlPointSet cps;

private:
	//Initialize constant matrices in this object.
	//Member function to build coefficient matrix.



protected:
	unsigned int numFitPointTime;

};

#endif // !defined(AFX_CUBICBSPLINECURVE_H__986ECA0A_F4A0_4984_AFE3_176700D13903__INCLUDED_)
