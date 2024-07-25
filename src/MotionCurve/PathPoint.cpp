// PathPoint.cpp: implementation of the PathPoint class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "PathPoint.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

PathPoint::PathPoint()
{
	x = 0.0f;
	t = 0.0f;
}

PathPoint::~PathPoint()
{

}

void PathPoint::operator =(const PathPoint& pp_in)
{
	x = pp_in.x;
	t = pp_in.t;
}

bool PathPoint::SetTimeVal(float timeVal)
{
	t = timeVal;

	return true;
}

float PathPoint::GetAmpVal()
{
	return x;
}

float PathPoint::GetTimeVal()
{
	return t;
}

bool PathPoint::SetAmpVal(float ampVal)
{
	x = ampVal;

	return true;
}
