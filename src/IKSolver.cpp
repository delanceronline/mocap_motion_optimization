// IKSolver.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "MotionCurve\MotionData.h"
#include "Tuple.h"
#include "Weight.h"
#include "Cylinder.h"
#include <string.h>

#define DEG_RAD 0.017453292519943295769236907684886
#define RAD_DEG 57.295779513082320876798154814105
#define ARM		4000
#define LEG		5000
#define LEFT	6000
#define RIGHT	7000

bool GetOptimalValue(MotionData *pMD, Vector3D *pGoal, Tuple *pTnow, unsigned int nFrame, float *pEVal);
bool GetRM(Vector3D *pEulerZXY, Matrix *pMat);
bool GetEffectorPosition(MotionData *pMD, unsigned int nFrame, Tuple *pTuple, Vector3D *pEffectorPosition);
void RotMatToEulerRPH(Matrix *pRMat, Vector3D *pEuler);
bool GetOptGradient(MotionData *pMD, unsigned int nFrame, Vector3D *pGoal, float *pOptValPrev, Tuple *pTnow, Tuple *pTprev, Matrix *pOGMat);
bool GetMaxDiagVal(Matrix *pJMat, Matrix *pSqrMat, float *pMaxDiagVal);
bool GuassianSolver(Matrix *pAMat, Matrix *pBMat, Matrix *pXMat);
bool GetJocabian3x3(MotionData *pMD, unsigned int nFrame, Tuple *pTnow, Tuple *pTprev, Matrix *pJMat);

MotionMarker *pJoint4, *pJoint3, *pJoint2, *pJoint1;
Cylinder CylPointSet;
float Deg_Rad = 0.0174533f, Rad_Deg = 57.29578f, threshold = 1e-6f;

int main(int argc, char* argv[])
{
	MotionData MD;
	Tuple Tnow, Tprev, Ttemp;
	Vector3D Goal, *pNewLocus;
	float EValNow, EValPrev, EValTemp, MaxDiagVal, nouda, d, factor;
	bool IsSmaller = true;
	unsigned int count = 0, nFrame = 0, a;
	Matrix Jocabian(3, 3), OGMat(3, 1), SqrMat(3, 3), IMat(3, 3), DeltaMat(3, 1), TempMat(3, 3);
	unsigned int i, nPart, nSide;
	char pSrcMotionFileName[256], pOutputMotionFileName[256], pCylFileName[256];

	if(argc != 7)
	{
		printf("Invalid parameters!\n");
		printf("Format: [Source BVH file] [Cylinder file] [Output BVH file] [Part] [Side] [Factor]\n");
		return 0;
	}

	strcpy(pSrcMotionFileName, argv[1]);
	strcpy(pCylFileName, argv[2]);
	strcpy(pOutputMotionFileName, argv[3]);
	
	if(_stricmp(argv[4], "arm") != 0 && _stricmp(argv[4], "leg") != 0)
	{
		printf("[Part] is not well specified.\n");
		printf("Format: [Source BVH file] [Cylinder file] [Output BVH file] [Part] [Side] [Factor]\n");
		printf("[Part] must be either \"arm\" or \"leg\"\n" );

		return 0;
	}

	if(_stricmp(argv[4], "arm") == 0)
		nPart = ARM;

	if(_stricmp(argv[4], "leg") == 0)
		nPart = LEG;

	if(_stricmp(argv[5], "left") != 0 && _stricmp(argv[5], "right") != 0)
	{
		printf("[Side] is not well specified.\n");
		printf("Format: [Source BVH file] [Cylinder file] [Output BVH file] [Part] [Side] [Factor]\n");
		printf("[Side] must be either \"left\" or \"right\"\n" );

		return 0;
	}

	if(_stricmp(argv[5], "left") == 0)
		nSide = LEFT;

	if(_stricmp(argv[5], "right") == 0)
		nSide = RIGHT;	

	if(!sscanf(argv[6], "%f", &factor))
	{
		printf("[Factor] is not well specified.\n");
		printf("Format: [Source BVH file] [Cylinder file] [Output BVH file] [Part] [Side] [Factor]\n");
		printf("[Factor] must be a floating number");
	};

	printf("Factor = %f\n", factor);

	if(!MD.LoadBVHFile(pSrcMotionFileName))
	{
		printf("Can't open source BVH file!\n");
		
		return 0;
	}

	switch(nPart)
	{
	case ARM:
		{
			if(nSide == LEFT)
			{
				pJoint4 = MD.GetMarkerByName("left_collar");
				pJoint3 = MD.GetMarkerByName("left_shoulder");
				pJoint2 = MD.GetMarkerByName("left_elbow");
				pJoint1 = MD.GetMarkerByName("left_hand");
			}
			else
			{
				pJoint4 = MD.GetMarkerByName("right_collar");
				pJoint3 = MD.GetMarkerByName("right_shoulder");
				pJoint2 = MD.GetMarkerByName("right_elbow");
				pJoint1 = MD.GetMarkerByName("right_hand");
			}

			break;
		}

	case LEG:
		{
			if(nSide == LEFT)
			{
				pJoint4 = MD.GetMarkerByName("root");
				pJoint3 = MD.GetMarkerByName("left_leg");
				pJoint2 = MD.GetMarkerByName("left_knee");
				pJoint1 = MD.GetMarkerByName("left_ankle");
			}
			else
			{
				pJoint4 = MD.GetMarkerByName("root");
				pJoint3 = MD.GetMarkerByName("right_leg");
				pJoint2 = MD.GetMarkerByName("right_knee");
				pJoint1 = MD.GetMarkerByName("right_ankle");
			}
			break;
		}
	}

	if(pJoint4 == NULL || pJoint3 == NULL || pJoint2 == NULL || pJoint1 == NULL)
	{
		printf("Unable to get marker from BVH!\n");

		return 0;
	}

	printf("Joint 1: %s\n", pJoint1->name);
	printf("Joint 2: %s\n", pJoint2->name);
	printf("Joint 3: %s\n", pJoint3->name);
	printf("Joint 4: %s\n", pJoint4->name);

	// Load target locus for optimization.
	if(!CylPointSet.LoadCylinder(pCylFileName))
	{
		printf("Can't open Cylinder file!\n");

		return 0;
	}

	pNewLocus = new Vector3D[CylPointSet.numPoints];
	for(i = 0; i < CylPointSet.numPoints; i++)
		pNewLocus[i] = CylPointSet.GetWristAbsPos(i);
	
	for(a = 0; a < MD.numFrames; a++)
	{
		count = 0;
		IsSmaller = true;

		d = 0.001f;

		Goal = pNewLocus[a];
		
		/////////////////////////////////////////////////////////////////////////////////////
		// Get initial guess for Tnow.
	
		if(a == 0)
		{
			//Use source motion as initial guess for first frame.
			Tprev.alpha1 = pJoint3->m_cbsc[1].pps.pp[0].GetAmpVal() * Deg_Rad;
			Tprev.beta1 = pJoint3->m_cbsc[2].pps.pp[0].GetAmpVal() * Deg_Rad;
			Tprev.gamma1 = pJoint3->m_cbsc[0].pps.pp[0].GetAmpVal() * Deg_Rad;

			Tprev.alpha2 = pJoint2->m_cbsc[1].pps.pp[0].GetAmpVal() * Deg_Rad;
			Tprev.beta2 = pJoint2->m_cbsc[2].pps.pp[0].GetAmpVal() * Deg_Rad;
			Tprev.gamma2 = pJoint2->m_cbsc[0].pps.pp[0].GetAmpVal() * Deg_Rad;
		}
		else
			//Use previous result as initial guess.
			Tprev = Tnow;
			
		/////////////////////////////////////////////////////////////////////////////////////

		/////////////////////////////////////////////////////////////////////////////////////
		// Set value of Tnow from original frame.

		Tnow.alpha1 = Tprev.alpha1;
		Tnow.beta1 = Tprev.beta1 + 0.001f * (float)fabs(Tprev.beta1);
		Tnow.gamma1 = Tprev.gamma1 + 0.001f  * (float)fabs(Tprev.gamma1);

		Tnow.alpha2 = Tprev.alpha2;
		Tnow.beta2 = Tprev.beta2 + 0.001f * (float)fabs(Tprev.beta2);
		Tnow.gamma2 = Tprev.gamma2;

		/////////////////////////////////////////////////////////////////////////////////////

		//Get current and previous values from objective function.
		GetOptimalValue(&MD, &Goal, &Tnow, a, &EValNow);
		GetOptimalValue(&MD, &Goal, &Tprev, a, &EValPrev);

		//Gauss-Newton method with Levenberg-Marquardt concept.
		//Maximum number of iterations is 200.
		while(count < 200)
		{
			
			if(IsSmaller)
			{
				// Get Jocabian matrix.
				GetJocabian3x3(&MD, a, &Tnow, &Tprev, &Jocabian);
				// Get maximun diagonal value from square matrix.
				GetMaxDiagVal(&Jocabian, &SqrMat, &MaxDiagVal);

				IsSmaller = false;
			}

			nouda = (float)fabs(d * MaxDiagVal);

			TempMat = Jocabian;
			TempMat.Transpose();
			
			TempMat.Product(&Jocabian, &SqrMat);
			IMat.SetToIdentity();
			IMat.ScalarProduct(nouda);
			SqrMat.Add(&IMat);
			
			// Get gradient of optimal function with respect to tuple.
			GetOptGradient(&MD, a, &Goal, &EValPrev, &Tnow, &Tprev, &OGMat);
			OGMat.ScalarProduct(-0.5f);

			//If no unique solution exists, stop iterations.
			if(!GuassianSolver(&SqrMat, &OGMat, &DeltaMat))
			{
				printf("%d Last EValTemp (no unique solution): %f\n", a, EValTemp);

				pJoint3->m_cbsc[1].pps.pp[a].SetAmpVal(Ttemp.alpha1 * Rad_Deg);
				pJoint3->m_cbsc[2].pps.pp[a].SetAmpVal(Ttemp.beta1 * Rad_Deg);
				pJoint3->m_cbsc[0].pps.pp[a].SetAmpVal(Ttemp.gamma1 * Rad_Deg);

				pJoint2->m_cbsc[1].pps.pp[a].SetAmpVal(Ttemp.alpha2 * Rad_Deg);
				pJoint2->m_cbsc[2].pps.pp[a].SetAmpVal(Ttemp.beta2 * Rad_Deg);
				pJoint2->m_cbsc[0].pps.pp[a].SetAmpVal(Ttemp.gamma2 * Rad_Deg);

				break;
			}

			Ttemp.alpha1 = Tnow.alpha1;
			Ttemp.beta1 = Tnow.beta1 + DeltaMat.m[0];
			Ttemp.gamma1 = Tnow.gamma1 + DeltaMat.m[1];
			Ttemp.alpha2 = Tnow.alpha2;
			Ttemp.beta2 = Tnow.beta2 + DeltaMat.m[2];
			Ttemp.gamma2 = Tnow.gamma2;

			GetOptimalValue(&MD, &Goal, &Ttemp, a, &EValTemp);

			//If current EVal is unchanged after one loop or is already small enough, stop iterations.
			if(fabs(EValTemp - EValNow) < threshold || fabs(EValTemp) < threshold)
			{
				printf("%d Last EValTemp (small enough): %f\n", a, EValTemp);

				pJoint3->m_cbsc[1].pps.pp[a].SetAmpVal(Ttemp.alpha1 * Rad_Deg);
				pJoint3->m_cbsc[2].pps.pp[a].SetAmpVal(Ttemp.beta1 * Rad_Deg);
				pJoint3->m_cbsc[0].pps.pp[a].SetAmpVal(Ttemp.gamma1 * Rad_Deg);

				pJoint2->m_cbsc[1].pps.pp[a].SetAmpVal(Ttemp.alpha2 * Rad_Deg);
				pJoint2->m_cbsc[2].pps.pp[a].SetAmpVal(Ttemp.beta2 * Rad_Deg);
				pJoint2->m_cbsc[0].pps.pp[a].SetAmpVal(Ttemp.gamma2 * Rad_Deg);

				break;
			}

			if(EValTemp < EValNow)
			{
				d = d / factor;
				
				Tprev = Tnow;
				Tnow = Ttemp;
				
				EValPrev = EValNow;
				EValNow = EValTemp;

				IsSmaller = true;
			}
			else
			{
				d = d * factor;

				IsSmaller = false;
			}

			count++;
		}
		
		if(count >= 200)
		{
			printf("%d Last EValTemp (end loop): %f\n", a, EValTemp);

			pJoint3->m_cbsc[1].pps.pp[a].SetAmpVal(Ttemp.alpha1 * Rad_Deg);
			pJoint3->m_cbsc[2].pps.pp[a].SetAmpVal(Ttemp.beta1 * Rad_Deg);
			pJoint3->m_cbsc[0].pps.pp[a].SetAmpVal(Ttemp.gamma1 * Rad_Deg);

			pJoint2->m_cbsc[1].pps.pp[a].SetAmpVal(Ttemp.alpha2 * Rad_Deg);
			pJoint2->m_cbsc[2].pps.pp[a].SetAmpVal(Ttemp.beta2 * Rad_Deg);
			pJoint2->m_cbsc[0].pps.pp[a].SetAmpVal(Ttemp.gamma2 * Rad_Deg);				
		}
		
	}

	delete[] pNewLocus;

	MD.SaveBVHFile(pOutputMotionFileName);

	return 0;
}

bool GetOptimalValue(MotionData *pMD, Vector3D *pGoal, Tuple *pTnow, unsigned int nFrame, float *pEVal)
{
	if(pMD == NULL || pTnow == NULL)
		return false;

	Vector3D EffecterPos;

	GetEffectorPosition(pMD, nFrame, pTnow, &EffecterPos);

	*pEVal = 1.0f * powf((EffecterPos - *pGoal).Mag(), 2.0f);

	return true;
}

void RotMatToEulerRPH(Matrix *pRMat, Vector3D *pEuler)
{
	// Rotational Matrix to Euler angle ZXY.
	pEuler->x = (float)asin(pRMat->GetVal(2, 1));
	if(pEuler->x < 1.5708f)
	{
		if(pEuler->x > -1.5708f)
		{
			pEuler->z = (float)atan2(-pRMat->GetVal(0, 1) , pRMat->GetVal(1, 1));
			pEuler->y = (float)atan2(-pRMat->GetVal(2, 0) , pRMat->GetVal(2, 2));
		}
		else
		{
			pEuler->z = -(float)atan2(pRMat->GetVal(0, 2) , pRMat->GetVal(0, 0));
			pEuler->y = 0.0f;
		}
	}
	else
	{
		pEuler->z = (float)atan2(pRMat->GetVal(0, 2) , pRMat->GetVal(0, 0));
		pEuler->y = 0.0f;
	}
}

//This function returns effector position (wrist for punching, ankle for kicking) using current tuple.
bool GetEffectorPosition(MotionData *pMD, unsigned int nFrame, Tuple *pTuple, Vector3D *pEffectorPosition)
{
	if(pMD == NULL || pTuple == NULL || pEffectorPosition == NULL)
		return false;

	Matrix TranslMat(4, 4), TempMat(4, 4), LocalTM1(4, 4), LocalTM2(4, 4), TranMat(4, 4);
	Vector3D Euler1, Euler2;

	Euler1.x = pTuple->alpha1;
	Euler1.y = pTuple->beta1;
	Euler1.z = pTuple->gamma1;

	Euler2.x = pTuple->alpha2;
	Euler2.y = pTuple->beta2;
	Euler2.z = pTuple->gamma2;

	pMD->UpdateMarkerTM(pMD->pMarkerHead, nFrame);

	GetRM(&Euler1, &LocalTM1);
	LocalTM1.m[3] = pJoint3->offset.x;
	LocalTM1.m[7] = pJoint3->offset.y;
	LocalTM1.m[11] = pJoint3->offset.z;

	GetRM(&Euler2, &LocalTM2);
	LocalTM2.m[3] = pJoint2->offset.x;
	LocalTM2.m[7] = pJoint2->offset.y;
	LocalTM2.m[11] = pJoint2->offset.z;

	pJoint4->TranMat.Product(&LocalTM1, &TempMat);
	TempMat.Product(&LocalTM2, &TranMat);

	TranslMat.SetToIdentity();
	TranslMat.m[3] = pJoint1->offset.x;
	TranslMat.m[7] = pJoint1->offset.y;
	TranslMat.m[11] = pJoint1->offset.z;

	TranMat.Product(&TranslMat, &TempMat);

	pEffectorPosition->x = TempMat.m[3];
	pEffectorPosition->y = TempMat.m[7];
	pEffectorPosition->z = TempMat.m[11];

	return true;
}

bool GetRM(Vector3D *pEulerZXY, Matrix *pMat)
{
	if(pEulerZXY == NULL || pMat == NULL)
		return false;

	if(pMat->numColumns != 4 || pMat->numRows != 4)
		return false;

	float sx, cx, sy, cy, sz, cz;

	sx = sinf(pEulerZXY->x);
	cx = cosf(pEulerZXY->x);

	sy = sinf(pEulerZXY->y);
	cy = cosf(pEulerZXY->y);

	sz = sinf(pEulerZXY->z);
	cz = cosf(pEulerZXY->z);

	pMat->SetToIdentity();
	pMat->m[0] = cz * cy - sz * sx * sy;
	pMat->m[1] = -sz * cx;
	pMat->m[2] = cz * sy + sz * sx * cy;

	pMat->m[4] = sz * cy + cz * sx * sy;
	pMat->m[5] = cz * cx;
	pMat->m[6] = sz * sy - cz * sx * cy;

	pMat->m[8] = -cx * sy;
	pMat->m[9] = sx;
	pMat->m[10] = cx * cy;	

	return true;
}

bool GetJocabian3x3(MotionData *pMD, unsigned int nFrame, Tuple *pTnow, Tuple *pTprev, Matrix *pJMat)
{
	if(pMD == NULL || pTnow == NULL || pTprev == NULL || pJMat == NULL)
		return false;

	float dx, dy, dz, db1, dg1, db2;
	Vector3D EffecterPosNow, EffecterPosPrev;
	Tuple Ttemp;

	//Get previous effecter's position.
	GetEffectorPosition(pMD, nFrame, pTprev, &EffecterPosPrev);
	
	//Get change in tuple.
	db1 = pTnow->beta1 - pTprev->beta1;
	dg1 = pTnow->gamma1 - pTprev->gamma1;
	db2 = pTnow->beta2 - pTprev->beta2;

	if((float)fabs(db1) < threshold)
		 db1 = threshold;
	if((float)fabs(dg1) < threshold)
		 dg1 = threshold;
	if((float)fabs(db2) < threshold)
		 db2 = threshold;
	
	/////////////////////////////////////////////////////////////////////////////////////
	//First column
	Ttemp = *pTprev;
	Ttemp.beta1 += db1;
	GetEffectorPosition(pMD, nFrame, &Ttemp, &EffecterPosNow);

	dx = EffecterPosNow.x - EffecterPosPrev.x;
	dy = EffecterPosNow.y - EffecterPosPrev.y;
	dz = EffecterPosNow.z - EffecterPosPrev.z;

	pJMat->m[0] = dx / db1;
	pJMat->m[3] = dy / db1;
	pJMat->m[6] = dz / db1;

	/////////////////////////////////////////////////////////////////////////////////////
	//Second column
	Ttemp = *pTprev;
	Ttemp.gamma1 += dg1;
	GetEffectorPosition(pMD, nFrame, &Ttemp, &EffecterPosNow);

	dx = EffecterPosNow.x - EffecterPosPrev.x;
	dy = EffecterPosNow.y - EffecterPosPrev.y;
	dz = EffecterPosNow.z - EffecterPosPrev.z;

	pJMat->m[1] = dx / dg1;
	pJMat->m[4] = dy / dg1;
	pJMat->m[7] = dz / dg1;

	/////////////////////////////////////////////////////////////////////////////////////
	//Third column
	Ttemp = *pTprev;
	Ttemp.beta2 += db2;
	GetEffectorPosition(pMD, nFrame, &Ttemp, &EffecterPosNow);

	dx = EffecterPosNow.x - EffecterPosPrev.x;
	dy = EffecterPosNow.y - EffecterPosPrev.y;
	dz = EffecterPosNow.z - EffecterPosPrev.z;

	pJMat->m[2] = dx / db2;
	pJMat->m[5] = dy / db2;
	pJMat->m[8] = dz / db2;

	/////////////////////////////////////////////////////////////////////////////////////

	return true;
}

//Get gradient of optimal value.
bool GetOptGradient(MotionData *pMD, unsigned int nFrame, Vector3D *pGoal, float *pOptValPrev, Tuple *pTnow, Tuple *pTprev, Matrix *pOGMat)
{
	if(pOptValPrev == NULL || pTnow == NULL || pTprev == NULL || pOGMat == NULL)
		return false;
	
	float dE, db1, dg1, db2;
	Tuple Ttemp;
	float EValNow;

	db1 = pTnow->beta1 - pTprev->beta1;
	dg1 = pTnow->gamma1 - pTprev->gamma1;
	db2 = pTnow->beta2 - pTprev->beta2;

	if((float)fabs(db1) < threshold)
		 db1 = threshold;
	if((float)fabs(dg1) < threshold)
		 dg1 = threshold;
	if((float)fabs(db2) < threshold)
		 db2 = threshold;

	///////////////////////////////////////////////////////////////////////////////
	Ttemp = *pTprev;
	Ttemp.beta1 += db1;

	GetOptimalValue(pMD, pGoal, &Ttemp, nFrame, &EValNow);

	dE = EValNow - *pOptValPrev;
	
	pOGMat->m[0] = dE / db1;

	///////////////////////////////////////////////////////////////////////////////
	Ttemp = *pTprev;
	Ttemp.gamma1 += dg1;
	
	GetOptimalValue(pMD, pGoal, &Ttemp, nFrame, &EValNow);

	dE = EValNow - *pOptValPrev;
	
	pOGMat->m[1] = dE / dg1;

	///////////////////////////////////////////////////////////////////////////////
	Ttemp = *pTprev;
	Ttemp.beta2 += db2;

	GetOptimalValue(pMD, pGoal, &Ttemp, nFrame, &EValNow);
	
	dE = EValNow - *pOptValPrev;
	
	pOGMat->m[2] = dE / db2;

	///////////////////////////////////////////////////////////////////////////////
	return true;
}

//Get maximum diagonal value.
bool GetMaxDiagVal(Matrix *pJMat, Matrix *pSqrMat, float *pMaxDiagVal)
{
	if(pJMat == NULL || pMaxDiagVal == NULL)
		return false;

	int i;
	Matrix J, JTranspose, TempMat;

	J = *pJMat;
	JTranspose = *pJMat;

	JTranspose.Transpose();

	if(pSqrMat != NULL)
		JTranspose.Product(&J, pSqrMat);

	J.ToDiagonal();

	JTranspose.Product(&J, &TempMat);

	*pMaxDiagVal = (float)fabs(TempMat.GetVal(0, 0));

	if(fabs(*pMaxDiagVal) < threshold)
		*pMaxDiagVal = 0.0f;

	for(i = 1; i < TempMat.numColumns; i++)
	{
		if(*pMaxDiagVal < (float)fabs(TempMat.GetVal(i, i)))
		{
			*pMaxDiagVal = (float)fabs(TempMat.GetVal(i, i));

			if(fabs(*pMaxDiagVal) < threshold)
				*pMaxDiagVal = 0.0f;
		}
	}

	return true;
}

//Solve system of linear equations.
bool GuassianSolver(Matrix *pAMat, Matrix *pBMat, Matrix *pXMat)
{
	if(pAMat == NULL || pBMat == NULL || pXMat == NULL)
		return false;

	if(pAMat->numColumns != pAMat->numRows || pBMat->numColumns != 1 || pAMat->numRows != pBMat->numRows)
		return false;

	Matrix *pInvMat;

	pInvMat = pAMat->Inverse();

	if(pInvMat == NULL)
		return false;

	pInvMat->Product(pBMat, pXMat);

	delete pInvMat;

	return true;
}
