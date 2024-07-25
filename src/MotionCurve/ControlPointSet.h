// ControlPointSet.h: interface for the ControlPointSet class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_CONTROLPOINTSET_H__9CF39B32_F257_4175_B82B_983D97832BA2__INCLUDED_)
#define AFX_CONTROLPOINTSET_H__9CF39B32_F257_4175_B82B_983D97832BA2__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "ControlPoint.h"

class ControlPointSet  
{
public:
	//Number of control points in this point set.
	unsigned int numPoints;
	//Pointer to control point array.
	ControlPoint *cp;

	//Reset this point set.
	bool Reset(unsigned int numPoints_in);
	//Operator for assigning.
	ControlPointSet& operator=(const ControlPointSet& cps_in);

	ControlPointSet();
	//Constructor to define control point set with N control points.
	ControlPointSet(unsigned int N);
	//Destroyer to release resources of this class.
	virtual ~ControlPointSet();

};

#endif // !defined(AFX_CONTROLPOINTSET_H__9CF39B32_F257_4175_B82B_983D97832BA2__INCLUDED_)
