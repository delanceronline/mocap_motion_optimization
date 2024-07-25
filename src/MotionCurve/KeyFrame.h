// KeyFrame.h: interface for the KeyFrame class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_KEYFRAME_H__0D9560ED_C4F4_419E_8D35_9114AA051005__INCLUDED_)
#define AFX_KEYFRAME_H__0D9560ED_C4F4_419E_8D35_9114AA051005__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "Vector3D.h"

class KeyFrame  
{
public:
	KeyFrame();
	
	Vector3D Val;
	float t;

	virtual ~KeyFrame();

};

#endif // !defined(AFX_KEYFRAME_H__0D9560ED_C4F4_419E_8D35_9114AA051005__INCLUDED_)
