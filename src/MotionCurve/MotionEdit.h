// MotionEdit.h: interface for the MotionEdit class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_MOTIONEDIT_H__F3FC738C_14E6_4DCA_ABB8_9F730CEFEF58__INCLUDED_)
#define AFX_MOTIONEDIT_H__F3FC738C_14E6_4DCA_ABB8_9F730CEFEF58__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "MotionData.h"
#include "Path.h"
#include "Vector3D.h"

struct MotionEditInfo
{
	unsigned int M1_StartFrame, M1_EndFrame;
	unsigned int M2_StartFrame, M2_EndFrame;
	unsigned int numFrames, nFrame;
	Vector3D *pVector3D;
	float t;
	MotionData *pMD1, *pMD2, *pOutputMD;
};

class MotionEdit  
{
public:
	bool GetPathTangent(Path *pPath, unsigned int nFrame, Vector3D *pTangent);
	float DegToRad(float deg);
	bool GetUpperBodyOrientation(MotionEditInfo *pMEInfo);
	bool TimeWarping(MotionEditInfo *pMEInfo);
	bool ExtractByPP(MotionEditInfo *pMEInfo);
	bool ExtractByCP(MotionEditInfo *pMEInfo);
	bool Concatenation_NoBlend(MotionEditInfo *pMEInfo);
	MotionEdit();
	virtual ~MotionEdit();

};

#endif // !defined(AFX_MOTIONEDIT_H__F3FC738C_14E6_4DCA_ABB8_9F730CEFEF58__INCLUDED_)
