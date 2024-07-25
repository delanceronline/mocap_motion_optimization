// PathPoint.h: interface for the PathPoint class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_PATHPOINT_H__181A7B40_6675_4E6E_80B6_23DBF03382D1__INCLUDED_)
#define AFX_PATHPOINT_H__181A7B40_6675_4E6E_80B6_23DBF03382D1__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "Point2D.h"

class PathPoint : Point2D
{
public:
	bool SetAmpVal(float ampVal);
	float GetTimeVal(void);
	float GetAmpVal(void);
	bool SetTimeVal(float timeVal);
	//Two values of path point.
	//float x, t;

	void operator=(const PathPoint& pp_in);

	PathPoint();
	virtual ~PathPoint();

};

#endif // !defined(AFX_PATHPOINT_H__181A7B40_6675_4E6E_80B6_23DBF03382D1__INCLUDED_)
