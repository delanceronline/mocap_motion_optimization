// ControlPoint.cpp: implementation of the ControlPoint class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "ControlPoint.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

ControlPoint::ControlPoint()
{
	x = 0.0f;
	t = 0.0f;
}

ControlPoint::~ControlPoint()
{
}

void ControlPoint::operator =(const ControlPoint& cp_in)
{
	x = cp_in.x;
	t = cp_in.t;
}

bool ControlPoint::SetTimeVal(float timeVal)
{
	t = timeVal;

	return true;
}

float ControlPoint::GetAmpVal()
{
	return x;
}

float ControlPoint::GetTimeVal()
{
	return t;
}

bool ControlPoint::SetAmpVal(float ampVal)
{
	x = ampVal;

	return true;
}
