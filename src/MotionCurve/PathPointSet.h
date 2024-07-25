// PathPointSet.h: interface for the PathPointSet class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_PATHPOINTSET_H__70297489_C082_44F3_B01E_EB8E99A8DA88__INCLUDED_)
#define AFX_PATHPOINTSET_H__70297489_C082_44F3_B01E_EB8E99A8DA88__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "PathPoint.h"

class PathPointSet  
{
public:
	//Number of path points in this point set.
	unsigned int numPoints;
	//Pointer to path point array.
	PathPoint *pp;

	float maxAmp, minAmp;

	//Reset this point set.
	bool Reset(unsigned int numPoints_in);
	//Operator for assigning.
	PathPointSet& operator =(const PathPointSet& pps_in);

	PathPointSet();
	//Constructor to define path point set with N path points.
	PathPointSet(unsigned int N);
	//Destroyer to release resources of this class.
	virtual ~PathPointSet();

};

#endif // !defined(AFX_PATHPOINTSET_H__70297489_C082_44F3_B01E_EB8E99A8DA88__INCLUDED_)
