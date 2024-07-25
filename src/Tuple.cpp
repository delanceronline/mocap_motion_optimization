// Tuple.cpp: implementation of the Tuple class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "Tuple.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

Tuple::Tuple()
{
	alpha1 = 0.0f;
	alpha2 = 0.0f;
	beta1 = 0.0f;
	beta2 = 0.0f;
	gamma1 = 0.0f;
	gamma2 = 0.0f;
}

Tuple::~Tuple()
{

}

Tuple& Tuple::operator =(const Tuple& tuple_in)
{
	if(&tuple_in == this)
		return *this;

	alpha1 = tuple_in.alpha1;
	alpha2 = tuple_in.alpha2;
	beta1 = tuple_in.beta1;
	beta2 = tuple_in.beta2;
	gamma1 = tuple_in.gamma1;
	gamma2 = tuple_in.gamma2;

	ElbowPosition = tuple_in.ElbowPosition;
	ElbowVelocity = tuple_in.ElbowVelocity;
	ElbowAcceleration = tuple_in.ElbowAcceleration;

	WristPosition = tuple_in.WristPosition;
	WristVelocity = tuple_in.WristVelocity;
	WristAcceleration = tuple_in.WristAcceleration;

	return *this;
}