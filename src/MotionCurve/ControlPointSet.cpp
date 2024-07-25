// ControlPointSet.cpp: implementation of the ControlPointSet class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "ControlPointSet.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

ControlPointSet::ControlPointSet()
{
	cp = NULL;
	numPoints = 0;
}

ControlPointSet::ControlPointSet(unsigned int N)
{
	cp = new ControlPoint[N];
	numPoints = N;
}

ControlPointSet::~ControlPointSet()
{
	if(cp != NULL)
	{
		delete[] cp;
		cp = NULL;
	}
}


ControlPointSet& ControlPointSet::operator =(const ControlPointSet& cps_in)
{
	if(this == &cps_in)
		return *this;

	if(cp != NULL)
	{
		delete[] cp;
		cp = NULL;
	}

	numPoints = cps_in.numPoints;

	if(numPoints > 0)
	{
		cp = new ControlPoint[cps_in.numPoints];

		for(unsigned int i = 0; i < numPoints; i++)
			cp[i] = cps_in.cp[i];
	}

	return *this;
}

bool ControlPointSet::Reset(unsigned int numPoints_in)
{
	if(cp != NULL)
	{
		delete[] cp;
		cp = NULL;
	}

	numPoints = numPoints_in;
	
	if(numPoints != 0)
		cp = new ControlPoint[numPoints];

	return true;	
}
