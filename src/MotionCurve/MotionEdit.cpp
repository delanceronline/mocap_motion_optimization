// MotionEdit.cpp: implementation of the MotionEdit class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "MotionEdit.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

MotionEdit::MotionEdit()
{

}

MotionEdit::~MotionEdit()
{

}

bool MotionEdit::Concatenation_NoBlend(MotionEditInfo *pMEInfo)
{

	return true;
}

// Not yet complete.
bool MotionEdit::ExtractByCP(MotionEditInfo *pMEInfo)
{
	if(pMEInfo->pMD1 == NULL || !pMEInfo->pMD1->m_AreMarkersFitted || pMEInfo->pMD1->pFitPointTime == NULL)
		return false;

	if(pMEInfo->M1_EndFrame - pMEInfo->M1_StartFrame < 3)
		return false;

	MotionData *pSourceMD = pMEInfo->pMD1;
	MotionData *pTargetMD = new MotionData;

	unsigned int i = 0, nStart, nEnd, numCPs;
	unsigned int nTargetStart = pMEInfo->M1_StartFrame - 1, nTargetEnd = pMEInfo->M1_EndFrame - 1;
	bool flag1 = false, flag2 = false;

	while(nTargetStart > pSourceMD->pFitPointTime[i].nPathPoint)
		i++;
	nStart = i;

	while(nTargetEnd > pSourceMD->pFitPointTime[i].nPathPoint)
		i++;
	nEnd = i - 1;

	numCPs = nEnd - nStart + 1;

	numCPs += 2;
	if(pSourceMD->pFitPointTime[nStart].nPathPoint == nTargetStart)
	{
		numCPs--;
		flag1 = true;
	}

	if(pSourceMD->pFitPointTime[nEnd].nPathPoint == nTargetEnd)
	{
		numCPs--;
		flag2 = true;
	}

	numCPs += 2;
	for(i = 0; i < pSourceMD->numFitPointTime; i++)
	{
		if(nTargetStart + 1 == pSourceMD->pFitPointTime[i].nPathPoint)
		{
			numCPs--;
			break;
		}
	}
	for(i = 0; i < pSourceMD->numFitPointTime; i++)
	{
		if(nTargetEnd - 1 == pSourceMD->pFitPointTime[i].nPathPoint)
		{
			numCPs--;
			break;
		}
	}

	pTargetMD->numFitPointTime = numCPs - 2;
	pTargetMD->pFitPointTime = new FitPointTimeArray[pTargetMD->numFitPointTime];

	for(i = 0; i < pTargetMD->numFitPointTime; i++)
	{
		if(flag1 || flag2)
		{
			if(i == 0)
			{
				pTargetMD->pFitPointTime[0].time = pSourceMD->GetPathPointAmp(pSourceMD->pMarkerHead, nTargetStart, 0);
				pTargetMD->pFitPointTime[0].nPathPoint = 0;
			}
			
			if(i == pTargetMD->numFitPointTime - 1)
			{
				pTargetMD->pFitPointTime[i].time = pSourceMD->GetPathPointAmp(pSourceMD->pMarkerHead, nTargetEnd, 0);
				pTargetMD->pFitPointTime[i].nPathPoint = nTargetEnd - nTargetStart + 1;
			}
		}
		else
		{
			pTargetMD->pFitPointTime[i].time = pSourceMD->pFitPointTime[i].time;
			pTargetMD->pFitPointTime[i].nPathPoint = pSourceMD->pFitPointTime[i + nStart].nPathPoint - nTargetStart;
		}
	}

	pMEInfo->pOutputMD = pTargetMD;

	return true;
}

bool MotionEdit::ExtractByPP(MotionEditInfo *pMEInfo)
{
	if(pMEInfo->pMD1 == NULL || !pMEInfo->pMD1->m_AreMarkersFitted || pMEInfo->pMD1->pFitPointTime == NULL)
		return false;

	if(pMEInfo->M1_EndFrame - pMEInfo->M1_StartFrame < 3)
		return false;

	MotionData *pSourceMD = pMEInfo->pMD1;
	MotionData *pTargetMD = new MotionData;
	pMEInfo->pOutputMD = pTargetMD;

	MotionMarker *pMK, *pTargetMK;

	unsigned int i = 0, j, k;
	unsigned int nTargetStart = pMEInfo->M1_StartFrame - 1, nTargetEnd = pMEInfo->M1_EndFrame - 1;

	*pTargetMD = *pSourceMD;
	pTargetMD->UnFitAllCurves();

	
	pTargetMD->numFrames = nTargetEnd - nTargetStart + 1;
	pTargetMD->numPPs = nTargetEnd - nTargetStart + 1;

	// Reset existing curves.
	for(pMK = pTargetMD->pMarkerHead; pMK != NULL; pMK = pMK->next)
		pMK->ResetAllCurves(pTargetMD->numFrames);

	// Copy range of curve points.
	for(pMK = pSourceMD->pMarkerHead; pMK != NULL; pMK = pMK->next)
	{
		pTargetMK = pTargetMD->GetMarker(pMK->nID);

		k = 0;
		for(i = nTargetStart; i <= nTargetEnd; i++)
		{
			for(j = 0; j < pMK->numDOFs; j++)
				pTargetMK->m_cbsc[j].pps.pp[k] = pMK->m_cbsc[j].pps.pp[i];

			k++;
		}
	}

	pTargetMD->FitAllMarkersToCBSC();

	return true;
}

bool MotionEdit::TimeWarping(MotionEditInfo *pMEInfo)
{
	if(pMEInfo->M1_StartFrame >= pMEInfo->M1_EndFrame || pMEInfo->pMD1 == NULL)
		return false;

	float time, val, timestep, starttime, delta;
	MotionData *pSourceMD = pMEInfo->pMD1;
	MotionData *pTargetMD = new MotionData;
	MotionMarker *pMK, *pTargetMK;
	unsigned int k, j, nTargetStart, nTargetEnd;

	nTargetStart = pMEInfo->M1_StartFrame - 1;
	nTargetEnd = pMEInfo->M1_EndFrame - 1;

	starttime = (float)pMEInfo->M1_StartFrame * pSourceMD->FrameTime;
	timestep = ((float)pMEInfo->M1_EndFrame * pSourceMD->FrameTime - starttime) / (float)pMEInfo->numFrames;

	*pTargetMD = *pMEInfo->pMD1;
	pTargetMD->UnFitAllCurves();

	pTargetMD->numFrames = pMEInfo->numFrames;
	pTargetMD->numPPs = pMEInfo->numFrames;

	// Reset existing curves.
	for(pMK = pTargetMD->pMarkerHead; pMK != NULL; pMK = pMK->next)
		pMK->ResetAllCurves(pTargetMD->numFrames);

	// Interpolate range of curve points.
	for(pMK = pSourceMD->pMarkerHead; pMK != NULL; pMK = pMK->next)
	{
		pTargetMK = pTargetMD->GetMarker(pMK->nID);

		delta = 0.0f;
		time = starttime;
		for(k = 0; k < pTargetMD->numFrames; k++)
		{
			for(j = 0; j < pMK->numDOFs; j++)
			{
				val = pSourceMD->InterpolatedCurveValue(pMK, j, time);

				pTargetMK->m_cbsc[j].pps.pp[k].SetAmpVal(val);
				pTargetMK->m_cbsc[j].pps.pp[k].SetTimeVal(delta);
			}

			time += timestep;
			delta += timestep;
		}
	}	

	pMEInfo->pOutputMD = pTargetMD;

	return true;
}

bool MotionEdit::GetUpperBodyOrientation(MotionEditInfo *pMEInfo)
{
	if(pMEInfo->pMD1 == NULL)
		return false;

	MotionMarker *pBackBoneMK = NULL, *pLeftCollarMK = NULL, *pRightCollarMK = NULL, *pRootMK;

	pRootMK = pMEInfo->pMD1->GetMarkerByName("root");
	pBackBoneMK = pMEInfo->pMD1->GetMarkerByName("backbone");
	pLeftCollarMK = pMEInfo->pMD1->GetMarkerByName("left_collar");
	pRightCollarMK = pMEInfo->pMD1->GetMarkerByName("right_collar");

	if(pRootMK == NULL || pBackBoneMK == NULL || pLeftCollarMK == NULL || pRightCollarMK == NULL)
		return false;

	float amp, sx, cx, sy, cy, sz, cz;
	Matrix TranMat_Root(4, 4), TranMat_BackBone(4, 4), TranMat_LeftCollar(4, 4), TranMat_RightCollar(4, 4);
	Matrix Rx(4, 4), Ry(4, 4), Rz(4, 4), TempMat(4, 4), TMat(4, 4), RMat(4, 4), LocalTranMat(4, 4);
	Matrix OP(4, 1), TP(4, 1);
	Vector3D BackBone, LeftCollar, RightCollar;

	// Find root's transformation matrix.
	amp = pRootMK->m_cbsc[4].pps.pp[pMEInfo->nFrame].GetAmpVal();
	sx = sinf(DegToRad(amp));
	cx = cosf(DegToRad(amp));
	amp = pRootMK->m_cbsc[5].pps.pp[pMEInfo->nFrame].GetAmpVal();
	sy = sinf(DegToRad(amp));
	cy = cosf(DegToRad(amp));
	amp = pRootMK->m_cbsc[3].pps.pp[pMEInfo->nFrame].GetAmpVal();
	sz = sinf(DegToRad(amp));
	cz = cosf(DegToRad(amp));

	Rx.SetToIdentity();
	Rx.SetVal(1, 1, cx);
	Rx.SetVal(1, 2, -sx);
	Rx.SetVal(2, 1, sx);
	Rx.SetVal(2, 2, cx);

	Ry.SetToIdentity();
	Ry.SetVal(0, 0, cy);
	Ry.SetVal(0, 2, sy);
	Ry.SetVal(2, 0, -sy);
	Ry.SetVal(2, 2, cy);
					
	Rz.SetToIdentity();
	Rz.SetVal(0, 0, cz);
	Rz.SetVal(0, 1, -sz);
	Rz.SetVal(1, 0, sz);
	Rz.SetVal(1, 1, cz);	

	Rz.Product(&Rx, &TempMat);
	TempMat.Product(&Ry, &TranMat_Root);

	// Find backbone's transformation matrix.
	amp = pBackBoneMK->m_cbsc[1].pps.pp[pMEInfo->nFrame].GetAmpVal();
	sx = sinf(DegToRad(amp));
	cx = cosf(DegToRad(amp));
	amp = pBackBoneMK->m_cbsc[2].pps.pp[pMEInfo->nFrame].GetAmpVal();
	sy = sinf(DegToRad(amp));
	cy = cosf(DegToRad(amp));
	amp = pBackBoneMK->m_cbsc[0].pps.pp[pMEInfo->nFrame].GetAmpVal();
	sz = sinf(DegToRad(amp));
	cz = cosf(DegToRad(amp));

	Rx.SetToIdentity();
	Rx.SetVal(1, 1, cx);
	Rx.SetVal(1, 2, -sx);
	Rx.SetVal(2, 1, sx);
	Rx.SetVal(2, 2, cx);

	Ry.SetToIdentity();
	Ry.SetVal(0, 0, cy);
	Ry.SetVal(0, 2, sy);
	Ry.SetVal(2, 0, -sy);
	Ry.SetVal(2, 2, cy);
					
	Rz.SetToIdentity();
	Rz.SetVal(0, 0, cz);
	Rz.SetVal(0, 1, -sz);
	Rz.SetVal(1, 0, sz);
	Rz.SetVal(1, 1, cz);

	Rz.Product(&Rx, &TempMat);
	TempMat.Product(&Ry, &RMat);

	TMat.SetToIdentity();
	TMat.SetVal(0, 3, pBackBoneMK->offset.x);
	TMat.SetVal(1, 3, pBackBoneMK->offset.y);
	TMat.SetVal(2, 3, pBackBoneMK->offset.z);	

	TMat.Product(&RMat, &LocalTranMat);
	TranMat_Root.Product(&LocalTranMat, &TranMat_BackBone);

	// Find LeftCollar's transformation matrix.
	amp = pLeftCollarMK->m_cbsc[1].pps.pp[pMEInfo->nFrame].GetAmpVal();
	sx = sinf(DegToRad(amp));
	cx = cosf(DegToRad(amp));
	amp = pLeftCollarMK->m_cbsc[2].pps.pp[pMEInfo->nFrame].GetAmpVal();
	sy = sinf(DegToRad(amp));
	cy = cosf(DegToRad(amp));
	amp = pLeftCollarMK->m_cbsc[0].pps.pp[pMEInfo->nFrame].GetAmpVal();
	sz = sinf(DegToRad(amp));
	cz = cosf(DegToRad(amp));

	Rx.SetToIdentity();
	Rx.SetVal(1, 1, cx);
	Rx.SetVal(1, 2, -sx);
	Rx.SetVal(2, 1, sx);
	Rx.SetVal(2, 2, cx);

	Ry.SetToIdentity();
	Ry.SetVal(0, 0, cy);
	Ry.SetVal(0, 2, sy);
	Ry.SetVal(2, 0, -sy);
	Ry.SetVal(2, 2, cy);
					
	Rz.SetToIdentity();
	Rz.SetVal(0, 0, cz);
	Rz.SetVal(0, 1, -sz);
	Rz.SetVal(1, 0, sz);
	Rz.SetVal(1, 1, cz);

	Rz.Product(&Rx, &TempMat);
	TempMat.Product(&Ry, &RMat);

	TMat.SetToIdentity();
	TMat.SetVal(0, 3, pLeftCollarMK->offset.x);
	TMat.SetVal(1, 3, pLeftCollarMK->offset.y);
	TMat.SetVal(2, 3, pLeftCollarMK->offset.z);	

	TMat.Product(&RMat, &LocalTranMat);
	TranMat_BackBone.Product(&LocalTranMat, &TranMat_LeftCollar);

	// Find RightCollar's transformation matrix.
	amp = pRightCollarMK->m_cbsc[1].pps.pp[pMEInfo->nFrame].GetAmpVal();
	sx = sinf(DegToRad(amp));
	cx = cosf(DegToRad(amp));
	amp = pRightCollarMK->m_cbsc[2].pps.pp[pMEInfo->nFrame].GetAmpVal();
	sy = sinf(DegToRad(amp));
	cy = cosf(DegToRad(amp));
	amp = pRightCollarMK->m_cbsc[0].pps.pp[pMEInfo->nFrame].GetAmpVal();
	sz = sinf(DegToRad(amp));
	cz = cosf(DegToRad(amp));

	Rx.SetToIdentity();
	Rx.SetVal(1, 1, cx);
	Rx.SetVal(1, 2, -sx);
	Rx.SetVal(2, 1, sx);
	Rx.SetVal(2, 2, cx);

	Ry.SetToIdentity();
	Ry.SetVal(0, 0, cy);
	Ry.SetVal(0, 2, sy);
	Ry.SetVal(2, 0, -sy);
	Ry.SetVal(2, 2, cy);
					
	Rz.SetToIdentity();
	Rz.SetVal(0, 0, cz);
	Rz.SetVal(0, 1, -sz);
	Rz.SetVal(1, 0, sz);
	Rz.SetVal(1, 1, cz);

	Rz.Product(&Rx, &TempMat);
	TempMat.Product(&Ry, &RMat);

	TMat.SetToIdentity();
	TMat.SetVal(0, 3, pRightCollarMK->offset.x);
	TMat.SetVal(1, 3, pRightCollarMK->offset.y);
	TMat.SetVal(2, 3, pRightCollarMK->offset.z);	

	TMat.Product(&RMat, &LocalTranMat);
	TranMat_BackBone.Product(&LocalTranMat, &TranMat_RightCollar);

	// Find position of BackBone marker.
	OP.SetVal(0, 0, 0.0f);
	OP.SetVal(1, 0, 0.0f);
	OP.SetVal(2, 0, 0.0f);
	OP.SetVal(3, 0, 1.0f);

	TranMat_BackBone.Product(&OP, &TP);
	BackBone = Vector3D(TP.GetVal(0, 0), TP.GetVal(1, 0), TP.GetVal(2, 0));

	// Find position of LeftCollar marker.
	OP.SetVal(0, 0, 0.0f);
	OP.SetVal(1, 0, 0.0f);
	OP.SetVal(2, 0, 0.0f);
	OP.SetVal(3, 0, 1.0f);

	TranMat_LeftCollar.Product(&OP, &TP);
	LeftCollar = Vector3D(TP.GetVal(0, 0), TP.GetVal(1, 0), TP.GetVal(2, 0));

	// Find position of RightCollar marker.
	OP.SetVal(0, 0, 0.0f);
	OP.SetVal(1, 0, 0.0f);
	OP.SetVal(2, 0, 0.0f);
	OP.SetVal(3, 0, 1.0f);

	TranMat_RightCollar.Product(&OP, &TP);
	RightCollar = Vector3D(TP.GetVal(0, 0), TP.GetVal(1, 0), TP.GetVal(2, 0));

	(*pMEInfo->pVector3D) = ((LeftCollar - BackBone).CrossProduct(&(RightCollar - BackBone))).Normalize();

	return true;
}

float MotionEdit::DegToRad(float deg)
{
	return deg * 0.0174533f;
}

bool MotionEdit::GetPathTangent(Path *pPath, unsigned int nFrame, Vector3D *pTangent)
{
	if(pPath == NULL)
		return false;

	if(nFrame == 0 || nFrame > pPath->numKeyFrames)
		return false;

	if(pPath->pInternalPathMD == NULL)
		return false;

	MotionData *pMD = pPath->pInternalPathMD;

	nFrame -= 1;

	if(nFrame == 0)
	{
		Vector3D v2, v1;

		v1.x = pMD->pMarkerHead->m_cbsc[0].cps.cp[0].GetAmpVal();
		v1.y = pMD->pMarkerHead->m_cbsc[1].cps.cp[0].GetAmpVal();
		v1.z = pMD->pMarkerHead->m_cbsc[2].cps.cp[0].GetAmpVal();

		v2.x = pMD->pMarkerHead->m_cbsc[0].cps.cp[1].GetAmpVal();
		v2.y = pMD->pMarkerHead->m_cbsc[1].cps.cp[1].GetAmpVal();
		v2.z = pMD->pMarkerHead->m_cbsc[2].cps.cp[1].GetAmpVal();

		*pTangent = ((v2 - v1) * 3.0f).Normalize();
	}

	if(nFrame == 1)
	{
		Vector3D v2, v3, v4;

		v2.x = pMD->pMarkerHead->m_cbsc[0].cps.cp[1].GetAmpVal();
		v2.y = pMD->pMarkerHead->m_cbsc[1].cps.cp[1].GetAmpVal();
		v2.z = pMD->pMarkerHead->m_cbsc[2].cps.cp[1].GetAmpVal();

		v3.x = pMD->pMarkerHead->m_cbsc[0].cps.cp[2].GetAmpVal();
		v3.y = pMD->pMarkerHead->m_cbsc[1].cps.cp[2].GetAmpVal();
		v3.z = pMD->pMarkerHead->m_cbsc[2].cps.cp[2].GetAmpVal();

		v4.x = pMD->pMarkerHead->m_cbsc[0].cps.cp[3].GetAmpVal();
		v4.y = pMD->pMarkerHead->m_cbsc[1].cps.cp[3].GetAmpVal();
		v4.z = pMD->pMarkerHead->m_cbsc[2].cps.cp[3].GetAmpVal();

		*pTangent = ((v2 * (-1.5f) + v3 * 0.5f + v4) * 0.5f).Normalize();
	}

	if(nFrame > 1 && nFrame < pMD->numFrames - 3)
	{
		Vector3D v1, v2;

		v1.x = pMD->pMarkerHead->m_cbsc[0].cps.cp[nFrame - 1].GetAmpVal();
		v1.y = pMD->pMarkerHead->m_cbsc[1].cps.cp[nFrame - 1].GetAmpVal();
		v1.z = pMD->pMarkerHead->m_cbsc[2].cps.cp[nFrame - 1].GetAmpVal();

		v2.x = pMD->pMarkerHead->m_cbsc[0].cps.cp[nFrame + 1].GetAmpVal();
		v2.y = pMD->pMarkerHead->m_cbsc[1].cps.cp[nFrame + 1].GetAmpVal();
		v2.z = pMD->pMarkerHead->m_cbsc[2].cps.cp[nFrame + 1].GetAmpVal();

		*pTangent = ((v1 * (-1.0f) + v2) * 0.5f).Normalize();
	}

	if(nFrame == pMD->numFrames - 3)
	{
		Vector3D v1, v2;

		v1.x = pMD->pMarkerHead->m_cbsc[0].cps.cp[nFrame + 2].GetAmpVal();
		v1.y = pMD->pMarkerHead->m_cbsc[1].cps.cp[nFrame + 2].GetAmpVal();
		v1.z = pMD->pMarkerHead->m_cbsc[2].cps.cp[nFrame + 2].GetAmpVal();

		v2.x = pMD->pMarkerHead->m_cbsc[0].cps.cp[nFrame].GetAmpVal();
		v2.y = pMD->pMarkerHead->m_cbsc[1].cps.cp[nFrame].GetAmpVal();
		v2.z = pMD->pMarkerHead->m_cbsc[2].cps.cp[nFrame].GetAmpVal();

		*pTangent = ((v1 - v2) * 0.5f).Normalize();
	}

	if(nFrame == pMD->numFrames - 2)
	{
		Vector3D v1, v2, v3;

		v1.x = pMD->pMarkerHead->m_cbsc[0].cps.cp[nFrame].GetAmpVal();
		v1.y = pMD->pMarkerHead->m_cbsc[1].cps.cp[nFrame].GetAmpVal();
		v1.z = pMD->pMarkerHead->m_cbsc[2].cps.cp[nFrame].GetAmpVal();

		v2.x = pMD->pMarkerHead->m_cbsc[0].cps.cp[nFrame + 1].GetAmpVal();
		v2.y = pMD->pMarkerHead->m_cbsc[1].cps.cp[nFrame + 1].GetAmpVal();
		v2.z = pMD->pMarkerHead->m_cbsc[2].cps.cp[nFrame + 1].GetAmpVal();

		v3.x = pMD->pMarkerHead->m_cbsc[0].cps.cp[nFrame + 2].GetAmpVal();
		v3.y = pMD->pMarkerHead->m_cbsc[1].cps.cp[nFrame + 2].GetAmpVal();
		v3.z = pMD->pMarkerHead->m_cbsc[2].cps.cp[nFrame + 2].GetAmpVal();

		*pTangent = ((v1 + v2 * 0.5f + v3 * (-1.5f)) * (-1.5f)).Normalize();
	}

	if(nFrame == pMD->numFrames - 1)
	{
		Vector3D v1, v2, v3;

		v1.x = pMD->pMarkerHead->m_cbsc[0].cps.cp[nFrame + 2].GetAmpVal();
		v1.y = pMD->pMarkerHead->m_cbsc[1].cps.cp[nFrame + 2].GetAmpVal();
		v1.z = pMD->pMarkerHead->m_cbsc[2].cps.cp[nFrame + 2].GetAmpVal();

		v2.x = pMD->pMarkerHead->m_cbsc[0].cps.cp[nFrame + 1].GetAmpVal();
		v2.y = pMD->pMarkerHead->m_cbsc[1].cps.cp[nFrame + 1].GetAmpVal();
		v2.z = pMD->pMarkerHead->m_cbsc[2].cps.cp[nFrame + 1].GetAmpVal();

		*pTangent = ((v1 - v2) * (3.0f)).Normalize();
	}	

	return true;
}
