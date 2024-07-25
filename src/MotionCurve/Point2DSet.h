// Point2DSet.h: interface for the Point2DSet class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_POINT2DSET_H__D6ED7968_6B25_48F7_AB86_7CB730099185__INCLUDED_)
#define AFX_POINT2DSET_H__D6ED7968_6B25_48F7_AB86_7CB730099185__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "Point2D.h"

class Point2DSet  
{
public:
	//Number of 2D data points in this point set.
	unsigned int numPoints;
	//Pointer to 2D point array.
	Point2D *point;
	
	//Constructor to define point set with N sets of 2D points.
	Point2DSet(unsigned int N);
	//Destroyer to release resources of this class.
	virtual ~Point2DSet();
};

#endif // !defined(AFX_POINT2DSET_H__D6ED7968_6B25_48F7_AB86_7CB730099185__INCLUDED_)
