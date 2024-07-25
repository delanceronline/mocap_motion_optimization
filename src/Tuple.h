// Tuple.h: interface for the Tuple class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_TUPLE_H__D4D1D53A_A234_4EAC_A499_52595F8A32EE__INCLUDED_)
#define AFX_TUPLE_H__D4D1D53A_A234_4EAC_A499_52595F8A32EE__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "MotionCurve\Vector3D.h"

class Tuple  
{
public:
	Tuple();

	float alpha1, beta1, gamma1, alpha2, beta2, gamma2;
	Vector3D ElbowPosition, ElbowVelocity, ElbowAcceleration;
	Vector3D WristPosition, WristVelocity, WristAcceleration;

	Tuple& operator=(const Tuple& tuple_in);

	virtual ~Tuple();

};

#endif // !defined(AFX_TUPLE_H__D4D1D53A_A234_4EAC_A499_52595F8A32EE__INCLUDED_)
