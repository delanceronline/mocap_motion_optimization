#include "stdafx.h"
#include <math.h>

bool IsEqual(float var1, float var2);
//bool IsGreater(float var1, float var2);

bool IsEqual(float var1, float var2)
{
	//Precison up to 10 of power -5.
	if((float)fabs((float)(var1 - var2)) < 1e-5)
		return true;
	else
		return false;
}
