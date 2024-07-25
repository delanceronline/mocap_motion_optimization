// ControlPoint.h: interface for the ControlPoint class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_CONTROLPOINT_H__C136F8B0_2978_4DEE_8881_5797A60B4C49__INCLUDED_)
#define AFX_CONTROLPOINT_H__C136F8B0_2978_4DEE_8881_5797A60B4C49__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "Point2D.h"

class ControlPoint : Point2D
{
public:
	bool SetAmpVal(float ampVal);
	float GetTimeVal(void);
	float GetAmpVal(void);
	bool SetTimeVal(float timeVal);
	//Two values of control point.
	//float x, t;

	void operator=(const ControlPoint& cp_in);

	ControlPoint();
	virtual ~ControlPoint();

};

#endif // !defined(AFX_CONTROLPOINT_H__C136F8B0_2978_4DEE_8881_5797A60B4C49__INCLUDED_)
