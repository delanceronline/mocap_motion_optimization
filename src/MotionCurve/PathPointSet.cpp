// PathPointSet.cpp: implementation of the PathPointSet class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "PathPointSet.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

PathPointSet::PathPointSet()
{
	pp = NULL;
	numPoints = 0;
}

PathPointSet::PathPointSet(unsigned int N)
{
	pp = new PathPoint[N];
	numPoints = N;
}

PathPointSet::~PathPointSet()
{
	if(pp != NULL)
	{
		delete[] pp;
		pp = NULL;
	}
}

PathPointSet& PathPointSet::operator =(const PathPointSet& pps_in)
{
	if(this == &pps_in)
		return *this;

	if(pp != NULL)
	{
		delete[] pp;
		pp = NULL;
	}

	numPoints = pps_in.numPoints;

	if(numPoints > 0)
	{
		pp = new PathPoint[pps_in.numPoints];

		for(unsigned int i = 0; i < numPoints; i++)
			pp[i] = pps_in.pp[i];
	}

	return *this;
}

bool PathPointSet::Reset(unsigned int numPoints_in)
{
	if(pp != NULL)
	{
		delete[] pp;
		pp = NULL;
	}

	numPoints = numPoints_in;

	if(numPoints != 0)
		pp = new PathPoint[numPoints];

	return true;
}
