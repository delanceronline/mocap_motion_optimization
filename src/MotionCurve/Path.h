// Path.h: interface for the Path class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_PATH_H__B4711D85_6FFB_433F_8258_5C73B665CBAF__INCLUDED_)
#define AFX_PATH_H__B4711D85_6FFB_433F_8258_5C73B665CBAF__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "KeyFrame.h"
#include "MotionData.h"

class Path  
{
public:
	MotionData * pInternalPathMD;
	MotionData * ExportMotionPathToBuffer(void);
	bool ImportMotionPathFromBuffer(MotionData *pMD);
	float Duration;
	float FrameTime;
	KeyFrame * pKeyFrame;
	unsigned int numKeyFrames;
	bool ExportBVHMotionPath(const char *filename);
	KeyFrame * ImportMotionPath(const char *filename, unsigned int *p_numFrames);
	Path();
	virtual ~Path();

private:
	bool InitializeInternalPathMD(void);
};

#endif // !defined(AFX_PATH_H__B4711D85_6FFB_433F_8258_5C73B665CBAF__INCLUDED_)
