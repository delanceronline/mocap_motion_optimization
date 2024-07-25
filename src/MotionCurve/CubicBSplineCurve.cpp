// ubicBSplines.cpp: implementation of the CubicBSplines class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "CubicBSplineCurve.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CubicBSplineCurve::CubicBSplineCurve()
{
	pInverse_mat = NULL;
	
	numFitPointTime = 0;
	numPPs = 0;
	numCPs = 0;
}

CubicBSplineCurve::~CubicBSplineCurve()
{
}

bool CubicBSplineCurve::CalculateCPFromPP(unsigned int _numCPs, unsigned int knotstep)
{
	if(pInverse_mat == NULL)
		return false;

	unsigned int i, j = 1;

	// Subtract two addition control points for boundary condition.
	numCPs = _numCPs;
	numFitPointTime = numCPs - 2;

	//Two column matrices for amplitude and time of path point set.
	Matrix ppVal_mat(numFitPointTime, 1);
	//Two column matrices for amplitude and time of control point set.
	Matrix cpVal_mat(numFitPointTime, 1);

	//Set column matrices of path point set.
	ppVal_mat.SetVal(0, 0, 6.0f * pps.pp[0].GetAmpVal());
	for(i = 1; i < pps.numPoints - 1; i += knotstep)
	{
		ppVal_mat.SetVal(j, 0, pps.pp[i].GetAmpVal());
		j++;
	}
	ppVal_mat.SetVal(numFitPointTime - 1, 0, 6.0f * pps.pp[pps.numPoints - 1].GetAmpVal());

	//Calculate control points using inverse.
	(*pInverse_mat).Product(&ppVal_mat, &cpVal_mat);

	//Put amplitude and time to control point set from two column matrices of control point set.
	cps.Reset(numCPs);
	cps.cp[0].SetAmpVal(pps.pp[0].GetAmpVal());
	for(i = 1; i < cps.numPoints - 1; i++)
		cps.cp[i].SetAmpVal(cpVal_mat.GetVal(i - 1, 0));
	cps.cp[numCPs - 1].SetAmpVal(pps.pp[pps.numPoints - 1].GetAmpVal());

	return true;
}

bool CubicBSplineCurve::FindMaxAndMinPPAmp()
{
	unsigned int i;
	float prev;

	// Max.
	for(i = 0; i < pps.numPoints; i++)
	{
		if(i == 0)
		{
			prev = pps.pp[i].GetAmpVal();
			pps.maxAmp = pps.pp[i].GetAmpVal();
		}
		else
		{
			if(pps.pp[i].GetAmpVal() > pps.maxAmp)
				pps.maxAmp = pps.pp[i].GetAmpVal();

			prev = pps.pp[i].GetAmpVal();
		}
	}

	// Min.
	for(i = 0; i < pps.numPoints; i++)
	{
		if(i == 0)
		{
			prev = pps.pp[i].GetAmpVal();
			pps.minAmp = pps.pp[i].GetAmpVal();
		}
		else
		{
			if(pps.pp[i].GetAmpVal() < pps.minAmp)
				pps.minAmp = pps.pp[i].GetAmpVal();

			prev = pps.pp[i].GetAmpVal();
		}
	}

	return true;
}

float CubicBSplineCurve::GetMaxAmp()
{
	return pps.maxAmp;
}

float CubicBSplineCurve::GetMinAmp()
{
	return pps.minAmp;
}

CubicBSplineCurve& CubicBSplineCurve::operator =(const CubicBSplineCurve &CBSC_in)
{
	if(this == &CBSC_in)
		return *this;

	pInverse_mat = CBSC_in.pInverse_mat;
	
	numFitPointTime = CBSC_in.numFitPointTime;
	numPPs = CBSC_in.numPPs;
	numCPs = CBSC_in.numCPs;

	pps = CBSC_in.pps;
	cps = CBSC_in.cps;

	return *this;
}

bool CubicBSplineCurve::UnFitCurve()
{
	cps.Reset(0);
	numFitPointTime = 0;
	pInverse_mat = NULL;
	
	return true;
}
