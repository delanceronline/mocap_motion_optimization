// Weight.h: interface for the Weight class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_WEIGHT_H__AFDF8CA2_D777_4CD7_BB9D_0C1D3C601FC0__INCLUDED_)
#define AFX_WEIGHT_H__AFDF8CA2_D777_4CD7_BB9D_0C1D3C601FC0__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

class Weight  
{
public:
	Weight();

	float Wv1, Wv2, Wa1, Wa2, Wj1, Wj2, Wj3;

	virtual ~Weight();

};

#endif // !defined(AFX_WEIGHT_H__AFDF8CA2_D777_4CD7_BB9D_0C1D3C601FC0__INCLUDED_)
