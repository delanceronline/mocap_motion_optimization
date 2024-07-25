// MarkerSeq.h: interface for the MarkerSeq class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_MARKERSEQ_H__F4ECD341_34AD_4336_A01A_E5DBE908481A__INCLUDED_)
#define AFX_MARKERSEQ_H__F4ECD341_34AD_4336_A01A_E5DBE908481A__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "MotionMarker.h"

class MarkerSeq  
{
public:
	MotionMarker * pMK;
	MarkerSeq();
	virtual ~MarkerSeq();

};

#endif // !defined(AFX_MARKERSEQ_H__F4ECD341_34AD_4336_A01A_E5DBE908481A__INCLUDED_)
