// MotionData.h: interface for the MotionData class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_MOTIONDATA_H__9D5278B1_874B_4506_BC38_D1864173986F__INCLUDED_)
#define AFX_MOTIONDATA_H__9D5278B1_874B_4506_BC38_D1864173986F__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#define UNKNOWN_MOTION_TYPE			16000
#define TRC_MOTION_TYPE				16001
#define BVH_MOTION_TYPE				16002

#include "MotionMarker.h"
#include "PathPointSet.h"
#include "PathPoint.h"	// Added by ClassView
#include "ControlPoint.h"	// Added by ClassView
#include "KeyFrame.h"
#include "MarkerSeq.h"

struct FitPointTimeArray
{
	float time;
	unsigned int nPathPoint;
};

class MotionData
{
public:
	bool UpdateVelAcc(unsigned int nFrame);
	bool UpdateMarkerTM(MotionMarker *pMK, unsigned int nFrame);
	MotionMarker * GetMarkerByName(const char *MarkerName);
	float GetPathPointAmp(MotionMarker *pMK, unsigned int nFrame, unsigned int nDOF);
	bool UnFitAllCurves(void);
	bool LinkToParent(const MotionData *pMD1, MotionData *pMD2);
	bool ReleaseInverseMatrix(void);
	bool BuildInverseMatrix(void);
	bool ReleaseCoefMatrix(void);
	bool FitAllMarkersToCBSC(void);
	float InterpolatedCurveValue(MotionMarker *pMK, unsigned int nDOF, float time);
	bool BuildConstantMatrix(void);
	bool InsertMarker(MotionMarker *pMK);
	KeyFrame *ImportMotionPath(const char *filename, unsigned int *p_numFrames);
	MotionData& operator =(const MotionData& MD_in);

	unsigned int knotstep;
	void ResetFitPointTimeArray(void);
	FitPointTimeArray * pFitPointTime;
	unsigned int numPPs, numCPs;
	bool SaveBVHMotionPath(const char *filename);
	bool RenewMaxAndMinPPAmp(unsigned int nMarker, unsigned int nDOF);
	ControlPoint GetControlPoint(unsigned int nMarker, unsigned int nFrame, unsigned int nDOF);
	unsigned int GetMarkerIndex(const char *MarkerName);
	bool m_AreMarkersFitted;
	bool SaveBVHFile(const char *filename, float _TimeLength, float _FrameTime);
	bool SaveBVHFile(const char *filename);
	bool BuildCoefMatrix(unsigned int N);
	unsigned int nMotionData;
	MotionData * pNext;
	bool IsMarkerEndSite(unsigned int nMarker);
	unsigned int GetNumDOFs(void);
	char * GetDOFName(unsigned int nMarker, unsigned int nDOF);
	MotionMarker *GetMarker(unsigned int nMarker);
	float GetMinAmp(unsigned int nMarker, unsigned int nDOF);
	float GetMaxAmp(unsigned int nMarker, unsigned int nDOF);
	PathPoint GetPathPoint(unsigned int nMarker, unsigned int nFrame, unsigned  int  nDOF);
	bool LoadBVHFile(const char *filename);
	void SetMotionFileType(unsigned int type);
	char * GetMarkerName(unsigned int index);
	//Number of markers of this trc buffer.
	unsigned int numMarkers;
	//Number of frames of this trc buffer.
	unsigned int numFrames;

	unsigned int m_MotionDataType;
	unsigned int m_maxDOFs;
	unsigned int m_nMSeq;

	MotionMarker *pMarkerHead;
	Matrix *m_pCoefMat;
	Matrix *m_pInvMat;
	Matrix m_ConstMat1, m_ConstMat2, m_ConstMat3, m_ConstMat4, m_ConstMat5;
	MarkerSeq *m_pMSeq;

	unsigned int numFitPointTime;

	float DataRate;
	float CameraRate;
	float FrameTime;
	char Units[16];
	
	float TimeLength;

	//Load from TRC file.
	bool LoadTRCFile(const char *filename);
	//Save buffer to TRC file.
	bool SaveTRCFile(const char *filename);

	unsigned int FileType();

	MotionData();
	virtual ~MotionData();

protected:
	bool SaveBVHMarker(MotionMarker *pParentMK, int level, FILE *stream);

private:
	bool PrintTab(int numTabs, FILE *stream);
	bool LoadBVHMarker(MotionMarker *parent, MotionMarker *prev, FILE *stream, bool *IsNoMoreChild);
};

#endif // !defined(AFX_MOTIONDATA_H__9D5278B1_874B_4506_BC38_D1864173986F__INCLUDED_)
