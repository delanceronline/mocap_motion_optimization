// MotionData.cpp: implementation of the MotionData class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "MotionData.h"
#include "Vector3D.h"
#include <string.h>

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

MotionData::MotionData()
{
	pMarkerHead = NULL;
	pNext = NULL;
	m_pCoefMat = NULL;
	m_pInvMat = NULL;
	m_pMSeq = NULL;
	pFitPointTime = NULL;
	m_MotionDataType = UNKNOWN_MOTION_TYPE;

	numFrames = 0;
	numMarkers = 0;
	numPPs = 0;
	numCPs  =0;

	knotstep = 2;
	numFitPointTime = 0;

	m_nMSeq = 0;
	FrameTime = 0.0f;
	TimeLength = 0.0f;

	DataRate = 0.0f;
	CameraRate = 0.0f;

	m_AreMarkersFitted = false;

	BuildConstantMatrix();
}

MotionData::~MotionData()
{
	if(pMarkerHead != NULL)
	{
		MotionMarker *temp1, *temp2;

		temp1 = pMarkerHead;
		while(temp1 != NULL)
		{
			temp2 = temp1->next;
			delete temp1;
			temp1 = temp2;
		}
	}

	if(m_pCoefMat != NULL)
	{
		delete m_pCoefMat;
		m_pCoefMat = NULL;
	}

	if(m_pInvMat != NULL)
	{
		delete m_pInvMat;
		m_pInvMat = NULL;
	}

	if(m_pMSeq != NULL)
	{
		delete[] m_pMSeq;
		m_pMSeq = NULL;
	}

	if(pFitPointTime != NULL)
	{
		delete[] pFitPointTime;
		pFitPointTime = NULL;
	}
}

unsigned int MotionData::FileType()
{
	return m_MotionDataType;
}

bool MotionData::LoadTRCFile(const char *filename)
{
	FILE *stream;

	stream = fopen(filename, "r");
	if(stream != NULL)
	{
		unsigned int i, j;
		float amp, time;
		char line[2048], word[256];

		fseek(stream, 0L, SEEK_SET);
		fgets(line, 2048, stream);
		fgets(line, 2048, stream);
		fgets(line, 2048, stream);
		sscanf(line, "%f\t%f\t%d\t%d\t%f\t%d\t%d\t%s\n", &DataRate, &CameraRate, &numFrames, &numMarkers, Units, &time, &i, &j);

		numPPs = numFrames;

		// Allocate markers.
		MotionMarker *pMK = NULL, *pPrevMK = NULL;

		fscanf(stream, "%s", word);
		fscanf(stream, "%s", word);
		for(i = 0; i < numMarkers; i++)
		{
			pMK = new MotionMarker;
			
			if(i == 0)
				pMarkerHead = pMK;
			else
				pPrevMK->next = pMK;

			// Read marker's name.
			fscanf(stream, "%s", word);

			// Initialize markers
			pMK->Initialize(3, numFrames, word, TRC_MARKER);
			pMK->nID = i;

			pPrevMK = pMK;
		}

		// Set names for all DOFs.
		for(i = 0; i < numMarkers; i++)
		{
			MotionMarker *pMK = GetMarker(i);
			for(j = 0; j < pMK->numDOFs; j++)
			{
				if(fscanf(stream, "%s", word) != 1)
					return false;
				pMK->SetDOFName(j, word);
			}
		}
		
		// Jump down one line.
		while(fgetc(stream) != '\n');

		for(i = 0; i < numFrames; i++)
		{
			fscanf(stream, "%s", word);
			fscanf(stream, "%f", &time);

			pMK = pMarkerHead;
			for(j = 0; j < numMarkers; j++)
			{
				fscanf(stream, "%f", &amp);
				pMK->m_cbsc[0].pps.pp[i].SetAmpVal(amp);
				fscanf(stream, "%f", &amp);
				pMK->m_cbsc[1].pps.pp[i].SetAmpVal(amp);
				fscanf(stream, "%f", &amp);
				pMK->m_cbsc[2].pps.pp[i].SetAmpVal(amp);

				pMK->m_cbsc[0].pps.pp[i].SetTimeVal(amp);
				pMK->m_cbsc[1].pps.pp[i].SetTimeVal(amp);
				pMK->m_cbsc[2].pps.pp[i].SetTimeVal(amp);

				pMK = pMK->next;
			}
		}

	}
	else
		return false;

	fclose(stream);

	SetMotionFileType(TRC_MOTION_TYPE);

	return true;
}

bool MotionData::SaveTRCFile(const char *filename)
{

	return true;
}

char * MotionData::GetMarkerName(unsigned int index)
{
	if(m_MotionDataType != UNKNOWN_MOTION_TYPE)
		return GetMarker(index)->name;

	return NULL;
}

void MotionData::SetMotionFileType(unsigned int type)
{
	m_MotionDataType = type;

	if(	m_MotionDataType == TRC_MOTION_TYPE)
		m_maxDOFs = 3;

	if(	m_MotionDataType == BVH_MOTION_TYPE)
		m_maxDOFs = 6;
}

PathPoint MotionData::GetPathPoint(unsigned int nMarker, unsigned int nFrame, unsigned int nDOF)
{
	PathPoint pp;

	if(m_MotionDataType != UNKNOWN_MOTION_TYPE)
	{
		MotionMarker *pMK = GetMarker(nMarker);

		if(pMK != NULL)
			if(!pMK->IsEndSite)
				pp = pMK->m_cbsc[nDOF].pps.pp[nFrame];
	}

	return pp;
}

float MotionData::GetMaxAmp(unsigned int nMarker, unsigned int nDOF)
{
	if(m_MotionDataType != UNKNOWN_MOTION_TYPE)
	{
		MotionMarker *pMK = GetMarker(nMarker);

		if(pMK != NULL)
			if(!pMK->IsEndSite)
				return GetMarker(nMarker)->m_cbsc[nDOF].GetMaxAmp();
	}
	
	return 0.0f;
}

float MotionData::GetMinAmp(unsigned int nMarker, unsigned int nDOF)
{
	if(m_MotionDataType != UNKNOWN_MOTION_TYPE)
	{
		MotionMarker *pMK = GetMarker(nMarker);

		if(pMK != NULL)
			if(!pMK->IsEndSite)
				return pMK->m_cbsc[nDOF].GetMinAmp();
	}

	return 0.0f;
}

// This function is to get address of marker in marker linked list.
MotionMarker *MotionData::GetMarker(unsigned int nMarker)
{
	if(pMarkerHead != NULL)
	{
		MotionMarker *pMK;

		pMK = pMarkerHead;
		while(pMK != NULL)
		{
			if(pMK->nID == nMarker)
				return pMK;
			pMK = pMK->next;
		}
		return NULL;
	}
	else
		return NULL;
}

char * MotionData::GetDOFName(unsigned int nMarker, unsigned int nDOF)
{
	return GetMarker(nMarker)->CurveName[nDOF].name;
}

unsigned int MotionData::GetNumDOFs()
{
	unsigned int count = 0;
	MotionMarker *pMK = pMarkerHead;

	while(pMK != NULL)
	{
		count += pMK->numDOFs;

		pMK = pMK->next;
	}

	return count;
}

bool MotionData::IsMarkerEndSite(unsigned int nMarker)
{
	return GetMarker(nMarker)->IsEndSite;
}

bool MotionData::BuildCoefMatrix(unsigned int N)
{
	//Setup coefficient matrix with N x N dimension.
	if(N >= 4)
	{
		m_pCoefMat = new Matrix(N, N);

		//coef_mat.ResetDimension(N, N);

		m_pCoefMat->SetVal(0, 0, 9.0f); m_pCoefMat->SetVal(0, 1, -3.0f);
		m_pCoefMat->SetVal(1, 0, 0.25f); m_pCoefMat->SetVal(1, 1, 7.0f / 12.0f); m_pCoefMat->SetVal(1, 2, 1.0f / 6.0f);

		for(unsigned int i = 2; i < N - 2; i++)
		{
			m_pCoefMat->SetVal(i, i - 1, 1.0f / 6.0f);
			m_pCoefMat->SetVal(i, i, 2.0f / 3.0f);
			m_pCoefMat->SetVal(i, i + 1, 1.0f / 6.0f);
		}

		m_pCoefMat->SetVal(N - 2, N - 3, 1.0f / 6.0f); m_pCoefMat->SetVal(N - 2, N - 2, 7.0f / 12.0f); m_pCoefMat->SetVal(N - 2, N - 1, 0.25f);
		m_pCoefMat->SetVal(N - 1, N - 2, -3.0f); m_pCoefMat->SetVal(N - 1, N - 1, 9.0f); 

		return true;
	}
	else
		return false;
}

bool MotionData::LoadBVHFile(const char *filename)
{
	FILE *stream;

	stream = fopen(filename, "r");
	if(stream != NULL)
	{
		unsigned int i, j;
		float amp, time;
		char line[4096], word1[256], word2[256];

		// Read number of frames first.
		while(strcmp(word1, "MOTION") != 0)
		{
			if(fgets(line, 4096, stream) == NULL)
				return false;
			sscanf(line, "%s", word1);
		}
		fgets(line, 4096, stream);
		sscanf(line, "%s %d", word1, &numFrames);
		if(strcmp(word1, "Frames:") != 0)
			return false;

		numPPs = numFrames;

		// Read from beginning again now.
		fseek(stream, 0L, SEEK_SET);

		// Check hierarchy header.
		fgets(line, 4096, stream);
		sscanf(line, "%s", word1);
		if(strcmp(word1, "HIERARCHY") != 0)
			return false;

		// Read all markers and their relation.
		bool flag;
		if(!LoadBVHMarker(NULL, NULL, stream, &flag))
		{
			fclose(stream);
			return false;
		}

		// Check motion data section.
		fgets(line, 4096, stream);
		sscanf(line, "%s", word1);
		if(strcmp(word1, "MOTION") != 0)
			return false;

		// Read frame time.
		fgets(line, 4096, stream);
		fgets(line, 4096, stream);
		sscanf(line, "%s %s %f", word1, word2, &FrameTime);
		
		MotionMarker *temp;
		temp = pMarkerHead;
		time = 0.0f;

		// Write path points.
		for(i = 0; i < numFrames; i++)
		{
			while(temp != NULL)
			{
				if(!temp->IsEndSite)
				{
					for(j = 0; j < temp->numDOFs; j++)
					{
						fscanf(stream, "%f", &amp);
						temp->m_cbsc[j].pps.pp[i].SetAmpVal(amp);
						temp->m_cbsc[j].pps.pp[i].SetTimeVal(time);
					}
				}
				temp = temp->next;
			}
			
			time += FrameTime;
			temp = pMarkerHead;
		}
			
		TimeLength = pMarkerHead->m_cbsc[0].pps.pp[numFrames - 1].GetTimeVal();

		for(temp = pMarkerHead; temp != NULL; temp = temp->next)
		{
			for(j = 0; j < temp->numDOFs; j++)
				temp->m_cbsc[j].numPPs = numFrames;
		}
		
	}
	else
		return false;

	fclose(stream);

	// Find number of markers.
	MotionMarker *temp;
	numMarkers = 0;
	temp = pMarkerHead;
	while(temp != NULL)
	{
		numMarkers++;
		temp = temp->next;
	}

	SetMotionFileType(BVH_MOTION_TYPE);

	return true;
}

bool MotionData::LoadBVHMarker(MotionMarker *parent, MotionMarker *prev, FILE *stream, bool *IsNoMoreChild)
{
	bool IsRoot;
	unsigned int i, numDOFs;
	char line[4096], word1[256], name[256];
	MotionMarker *temp = NULL, *pMK = NULL;

	if(parent == NULL)
		IsRoot = true;
	else
		IsRoot = false;

	fgets(line, 4096, stream);
	sscanf(line, "%s %s", word1, name);

	// Add new node to chain.
	if(IsRoot)
	{
		if(strcmp(word1, "ROOT") != 0)
			return false;

		// Allocate first node.
		pMarkerHead = new MotionMarker;
		pMarkerHead->nID = 0;
		pMarkerHead->IsEndSite = false;
		pMK = pMarkerHead;
	}
	else
	{
		if(strcmp(word1, "JOINT") != 0 && strcmp(word1, "End") != 0 && strcmp(word1, "}") != 0)
			return false;

		// Exit peacefully.
		if(strcmp(word1, "}") == 0)
		{
			*IsNoMoreChild = true;
			return true;
		}

		// Allocate new node.
		temp = new MotionMarker;
		temp->prev = prev;
		if(prev != NULL)
		{
			prev->next = temp;
			temp->nID = prev->nID + 1;
			temp->parent = parent;

			// Determine whether END SITE JOINT.
			if(strcmp(word1, "JOINT") == 0)
				temp->IsEndSite = false;
			else
				temp->IsEndSite = true;
		}
		pMK = temp;
	}

	// Read "{".
	fgets(line, 4096, stream);
	sscanf(line, "%s", word1);
	if(strcmp(word1, "{") != 0)
		return false;
	
	// Read offset.
	fgets(line, 4096, stream);
	sscanf(line, "%s %f %f %f", word1, &pMK->offset.x, &pMK->offset.y, &pMK->offset.z);
	if(strcmp(word1, "OFFSET") != 0)
		return false;

	if(!pMK->IsEndSite)
	{
		// Read channel information.
		fscanf(stream, "%s", word1);
		if(strcmp(word1, "CHANNELS") != 0)
			return false;
		fscanf(stream, "%d", &numDOFs);

		// Initialize newly allocated marker.
		pMK->Initialize(numDOFs, numFrames, name, BVH_MARKER);

		// Set names to all DOFs (curves).
		for(i = 0; i < pMK->numDOFs; i++)
		{
			fscanf(stream, "%s", word1);
			pMK->SetDOFName(i, word1);
		}
		fgets(line, 4096, stream);
		
		// Recursively load markers.
		bool flag = false;
		while(!flag)
		{
			// Find for last node.
			temp = pMarkerHead;
			while(temp->next != NULL)
				temp = temp->next;

			// Run recurive function and flag is for notifing whether there are no more children.
			if(!LoadBVHMarker(pMK, temp, stream, &flag))
				return false;
		}
	}
	else
	{
		// End Site marker doesn't have motion data and is not initialized.
		// It has offset data and the marker name of "End Site".
		strcpy(pMK->name, "End Site");
		// Read "}".
		fgets(line, 4096, stream);
		sscanf(line, "%s", word1);
		if(strcmp(word1, "}") != 0)
			return false;
	}

	// Exit recurive function.
	*IsNoMoreChild = false;
	return true;
}

bool MotionData::SaveBVHFile(const char *filename)
{
	if(m_MotionDataType == BVH_MOTION_TYPE && pMarkerHead != NULL)
	{
		FILE *stream;

		stream = fopen(filename, "w");
		if(stream != NULL)
		{
			PathPoint pp;
			MotionMarker *pMK = pMarkerHead;
			m_pMSeq = new MarkerSeq[numMarkers];
			m_nMSeq = 0;

			while(pMK->parent != NULL)
				pMK = pMK->next;

			fprintf(stream, "HIERARCHY\n");
			SaveBVHMarker(pMK, 0, stream);
			fprintf(stream, "MOTION\n");
			fprintf(stream, "Frames:	%u\n", numFrames);
			fprintf(stream, "Frame Time:	%f\n", FrameTime);

			for(unsigned int i = 0; i < numFrames; i++)
			{
				for(unsigned int j = 0; j < numMarkers; j++)
				{
					pMK = m_pMSeq[j].pMK;
					for(unsigned int k = 0; k < pMK->numDOFs; k++)
					{
						//pMK->m_cbsc[k].GetPathPoint(i, &pp);
						fprintf(stream, "%0.2f	", pMK->m_cbsc[k].pps.pp[i].GetAmpVal());
					}
				}
				fprintf(stream, "\n");
			}

			delete[] m_pMSeq;
			m_pMSeq = NULL;
		}
		fclose(stream);
	}
	else
		return false;

	return true;
}

bool MotionData::SaveBVHMarker(MotionMarker *pParentMK, int level, FILE *stream)
{
	MotionMarker *pMK = pMarkerHead;

	if(pParentMK->IsEndSite)
	{
		// Output End Site.
		PrintTab(level, stream);
		fprintf(stream, "End Site\n");
		
		PrintTab(level, stream);
		fprintf(stream, "{\n");
		
		PrintTab(level, stream);
		fprintf(stream, "\tOFFSET\t%0.2f\t%0.2f\t%0.2f\n", pParentMK->offset.x, pParentMK->offset.y, pParentMK->offset.z);

		PrintTab(level, stream);
		fprintf(stream, "}\n");

		m_pMSeq[m_nMSeq].pMK = pParentMK;
		m_nMSeq++;
	}
	else
	{
		// Output child joint.
		if(level == 0)
			fprintf(stream, "ROOT %s\n", pParentMK->name);
		else
		{
			PrintTab(level, stream);
			fprintf(stream, "JOINT %s\n", pParentMK->name);
		}

		PrintTab(level, stream);
		fprintf(stream, "{\n");

		PrintTab(level, stream);
		fprintf(stream, "\tOFFSET\t%0.2f\t%0.2f\t%0.2f\n", pParentMK->offset.x, pParentMK->offset.y, pParentMK->offset.z);

		PrintTab(level, stream);
		fprintf(stream, "\tCHANNELS %u", pParentMK->numDOFs);
		for(unsigned int i = 0; i < pParentMK->numDOFs; i++)
			fprintf(stream, " %s", pParentMK->CurveName[i].name);
		fprintf(stream, "\n");

		m_pMSeq[m_nMSeq].pMK = pParentMK;
		m_nMSeq++;

		while(pMK != NULL)
		{
			if(pMK->parent == pParentMK)
				SaveBVHMarker(pMK, level + 1, stream);

			pMK = pMK->next;
		}

		PrintTab(level, stream);
		fprintf(stream, "}\n");
	}

	return true;
}

bool MotionData::PrintTab(int numTabs, FILE *stream)
{
	for(int i = 0; i < numTabs; i++)
	{
		if(fprintf(stream, "\t") < 0)
			return false;
	}

	return true;
}

bool MotionData::SaveBVHFile(const char *filename, float _TimeLength, float _FrameTime)
{
	if(m_MotionDataType == BVH_MOTION_TYPE && pMarkerHead != NULL && m_AreMarkersFitted)
	{
		FILE *stream;

		stream = fopen(filename, "w");
		if(stream != NULL)
		{
			float var;
			MotionMarker *pMK = pMarkerHead;
			m_pMSeq = new MarkerSeq[numMarkers];
			m_nMSeq = 0;

			while(pMK->parent != NULL)
				pMK = pMK->next;

			fprintf(stream, "HIERARCHY\n");
			SaveBVHMarker(pMK, 0, stream);
			fprintf(stream, "MOTION\n");
			fprintf(stream, "Frames:	%u\n", (unsigned int)(_TimeLength / _FrameTime));
			fprintf(stream, "Frame Time:	%f\n", _FrameTime);

			for(float i = 0; i <= _TimeLength; i += _FrameTime)
			{
				for(unsigned int j = 0; j < numMarkers; j++)
				{
					pMK = m_pMSeq[j].pMK;
					for(unsigned int k = 0; k < pMK->numDOFs; k++)
					{
						//var = pMK->m_cbsc[k].GetCurveVal(i);
						var = InterpolatedCurveValue(pMK, k, i);
						fprintf(stream, "%0.2f	", var);
					}
				}
				fprintf(stream, "\n");
			}

			delete[] m_pMSeq;
			m_pMSeq = NULL;
		}
		fclose(stream);
	}
	else
		return false;

	return true;
}

unsigned int MotionData::GetMarkerIndex(const char *MarkerName)
{
	MotionMarker *pMK;

	for(pMK = pMarkerHead; pMK != NULL; pMK = pMK->next)
	{
		if(strcmp(pMK->name, MarkerName) == 0)
			return pMK->nID;
	}

	return 0;
}

ControlPoint MotionData::GetControlPoint(unsigned int nMarker, unsigned int nFrame, unsigned int nDOF)
{
	ControlPoint cp;

	if(m_MotionDataType != UNKNOWN_MOTION_TYPE)
	{
		MotionMarker *pMK = GetMarker(nMarker);

		if(pMK != NULL)
			if(!pMK->IsEndSite)
				cp = pMK->m_cbsc[nDOF].cps.cp[nFrame];
	}

	return cp;
}

bool MotionData::RenewMaxAndMinPPAmp(unsigned int nMarker, unsigned int nDOF)
{
	GetMarker(nMarker)->m_cbsc[nDOF].FindMaxAndMinPPAmp();

	return true;
}

bool MotionData::SaveBVHMotionPath(const char *filename)
{
	if(m_MotionDataType == BVH_MOTION_TYPE && pMarkerHead != NULL)
	{
		FILE *stream;

		stream = fopen(filename, "w");
		if(stream != NULL)
		{
			MotionMarker *pMarker = NULL;
			unsigned int i;

			fprintf(stream, "METPATHFILE1.0\n");
			fprintf(stream, "FrameTime: %f\n", FrameTime);
			fprintf(stream, "Duration: %f\n", (float)(numFrames - 1) * FrameTime);
			
			// Find root marker.
			for(i = 0; i < numMarkers; i++)
			{
				pMarker = GetMarker(i);
				
				if(pMarker == NULL)
					return false;

				if(pMarker->parent == NULL)
					break;
			}

			for(i = 0; i < numFrames; i++)
				fprintf(stream, "%f %f %f %f\n", pMarker->m_cbsc[0].pps.pp[i].GetAmpVal(), pMarker->m_cbsc[1].pps.pp[i].GetAmpVal(), pMarker->m_cbsc[2].pps.pp[i].GetAmpVal(), pMarker->m_cbsc[0].pps.pp[i].GetTimeVal());
		}
		fclose(stream);
	}
	else
		return false;

	return true;
}

KeyFrame *MotionData::ImportMotionPath(const char *filename, unsigned int *p_numFrames)
{
	FILE *stream;
	KeyFrame *pKeyFrame = NULL;

	stream = fopen(filename, "r");
	if(stream != NULL)
	{
		float frame_time, duration;
		char line[4096];
		unsigned int i, numKeyFrames = 0;
		MotionMarker *pMarker = NULL;

		fgets(line, 4096, stream);
		if(strcmp(line, "METPATHFILE1.0") == 0)
			return NULL;
	
		fgets(line, 4096, stream);
		sscanf(line, "Frame Time: %f", &frame_time);		

		fgets(line, 4096, stream);
		sscanf(line, "Duration: %f", &duration);

		// Find root marker.
		for(i = 0; i < numMarkers; i++)
		{
			pMarker = GetMarker(i);
				
			if(pMarker == NULL)
				return NULL;

			if(pMarker->parent == NULL)
				break;
		}

		while(!feof(stream))
		{
			fgets(line, 4096, stream);
			numKeyFrames++;
		}
		numKeyFrames--;

		*p_numFrames = numKeyFrames;

		if(numKeyFrames < 1)
			return NULL;

		fseek(stream, 0, 0);
		fgets(line, 4096, stream);
		fgets(line, 4096, stream);
		fgets(line, 4096, stream);

		pKeyFrame = new KeyFrame[numKeyFrames];

		for(i = 0; i < numKeyFrames; i++)
		{
			fgets(line, 4096, stream);
			sscanf(line, "%f %f %f %f", &pKeyFrame[i].Val.x, &pKeyFrame[i].Val.y, &pKeyFrame[i].Val.z, &pKeyFrame[i].t);
		}

	}
	else
		return NULL;

	return pKeyFrame;
}

void MotionData::ResetFitPointTimeArray()
{
	unsigned i, j;
	MotionMarker *pMK = NULL;

	if(pFitPointTime != NULL)
		delete[] pFitPointTime;

	// Calculate number of control points needed.
	numCPs = 4;
	for(j = 1; j < numFrames - 1; j += knotstep)
		numCPs++;

	numFitPointTime = numCPs - 2;

	pFitPointTime = new FitPointTimeArray[numFitPointTime];		
	j = 1;
	pMK = pMarkerHead;

	pFitPointTime[0].time = pMK->m_cbsc[0].pps.pp[0].GetTimeVal();
	pFitPointTime[0].nPathPoint = 0;
	for(i = 1; i < numFrames - 1; i += knotstep)
	{
		pFitPointTime[j].time = pMK->m_cbsc[0].pps.pp[i].GetTimeVal();
		pFitPointTime[j].nPathPoint = i;

		j++;
	}
	pFitPointTime[numFitPointTime - 1].time = pMK->m_cbsc[0].pps.pp[numFrames - 1].GetTimeVal();
	pFitPointTime[numFitPointTime - 1].nPathPoint = numFrames - 1;
}

bool MotionData::InsertMarker(MotionMarker *pMK)
{
	if(pMK == NULL)
		return false;

	if(pMarkerHead == NULL)
	{
		pMarkerHead = pMK;
		//pMarkerHead->nPosition = 0;
	}
	else
	{
		MotionMarker *pTemp = pMarkerHead;
		//unsigned int i = 1;

		while(pTemp->next != NULL)
		{
			pTemp = pTemp->next;
			//i++;
		}

		pTemp->next = pMK;
		pMK->prev = pTemp;
		//pMK->nPosition = i;
	}

	return true;
}

bool MotionData::BuildConstantMatrix()
{
	m_ConstMat1.ResetDimension(4, 4);
	m_ConstMat2.ResetDimension(4, 4);
	m_ConstMat3.ResetDimension(4, 4);
	m_ConstMat4.ResetDimension(4, 4);
	m_ConstMat5.ResetDimension(4, 4);

	//Set values for these five constant 4x4 matirces.
	m_ConstMat1.SetVal(0, 0, -1.0f); m_ConstMat1.SetVal(0, 1, 7.0f / 4.0f); m_ConstMat1.SetVal(0, 2, -11.0f / 12.0f); m_ConstMat1.SetVal(0, 3, 1 / 6.0f);
	m_ConstMat1.SetVal(1, 0, 3.0f); m_ConstMat1.SetVal(1, 1, -4.5f); m_ConstMat1.SetVal(1, 2, 1.5f); m_ConstMat1.SetVal(1, 3, 0.0f);
	m_ConstMat1.SetVal(2, 0, -3.0f); m_ConstMat1.SetVal(2, 1, 3.0f); m_ConstMat1.SetVal(2, 2, 0.0f); m_ConstMat1.SetVal(2, 3, 0.0f);
	m_ConstMat1.SetVal(3, 0, 1.0f); m_ConstMat1.SetVal(3, 1, 0.0f); m_ConstMat1.SetVal(3, 2, 0.0f); m_ConstMat1.SetVal(3, 3, 0.0f);

	m_ConstMat2.SetVal(0, 0, -0.25f); m_ConstMat2.SetVal(0, 1, 7.0f / 12.0f); m_ConstMat2.SetVal(0, 2, -0.5f); m_ConstMat2.SetVal(0, 3, 1 / 6.0f);
	m_ConstMat2.SetVal(1, 0, 3.0f / 4.0f); m_ConstMat2.SetVal(1, 1, -5.0f / 4.0f); m_ConstMat2.SetVal(1, 2, 0.5f); m_ConstMat2.SetVal(1, 3, 0.0f);
	m_ConstMat2.SetVal(2, 0, -3.0f / 4.0f); m_ConstMat2.SetVal(2, 1, 0.25f); m_ConstMat2.SetVal(2, 2, 0.5f); m_ConstMat2.SetVal(2, 3, 0.0f);
	m_ConstMat2.SetVal(3, 0, 0.25f); m_ConstMat2.SetVal(3, 1, 7.0f / 12.0f); m_ConstMat2.SetVal(3, 2, 1.0f / 6.0f); m_ConstMat2.SetVal(3, 3, 0.0f);

	m_ConstMat3.SetVal(0, 0, -1.0f); m_ConstMat3.SetVal(0, 1, 3.0f); m_ConstMat3.SetVal(0, 2, -3.0f); m_ConstMat3.SetVal(0, 3, 1.0f);
	m_ConstMat3.SetVal(1, 0, 3.0f); m_ConstMat3.SetVal(1, 1, -6.0f); m_ConstMat3.SetVal(1, 2, 3.0f); m_ConstMat3.SetVal(1, 3, 0.0f);
	m_ConstMat3.SetVal(2, 0, -3.0f); m_ConstMat3.SetVal(2, 1, 0.0f); m_ConstMat3.SetVal(2, 2, 3.0f); m_ConstMat3.SetVal(2, 3, 0.0f);
	m_ConstMat3.SetVal(3, 0, 1.0f); m_ConstMat3.SetVal(3, 1, 4.0f); m_ConstMat3.SetVal(3, 2, 1.0f); m_ConstMat3.SetVal(3, 3, 0.0f);

	m_ConstMat4.SetVal(0, 0, -1.0f / 6.0f); m_ConstMat4.SetVal(0, 1, 0.5f); m_ConstMat4.SetVal(0, 2, -7.0f / 12.0f); m_ConstMat4.SetVal(0, 3, 0.25f);
	m_ConstMat4.SetVal(1, 0, 0.5f); m_ConstMat4.SetVal(1, 1, -1.0f); m_ConstMat4.SetVal(1, 2, 0.5f); m_ConstMat4.SetVal(1, 3, 0.0f);
	m_ConstMat4.SetVal(2, 0, -0.5f); m_ConstMat4.SetVal(2, 1, 0.0f); m_ConstMat4.SetVal(2, 2, 0.5f); m_ConstMat4.SetVal(2, 3, 0.0f);
	m_ConstMat4.SetVal(3, 0, 1.0f / 6.0f); m_ConstMat4.SetVal(3, 1, 2.0f / 3.0f); m_ConstMat4.SetVal(3, 2, 1.0f / 6.0f); m_ConstMat4.SetVal(3, 3, 0.0f);

	m_ConstMat5.SetVal(0, 0, -1.0f / 6.0f); m_ConstMat5.SetVal(0, 1, 11.0f / 12.0f); m_ConstMat5.SetVal(0, 2, -7.0f / 4.0f); m_ConstMat5.SetVal(0, 3, 1.0f);
	m_ConstMat5.SetVal(1, 0, 0.5f); m_ConstMat5.SetVal(1, 1, -5.0f / 4.0f); m_ConstMat5.SetVal(1, 2, 3.0f / 4.0f); m_ConstMat5.SetVal(1, 3, 0.0f);
	m_ConstMat5.SetVal(2, 0, -0.5f); m_ConstMat5.SetVal(2, 1, -0.25f); m_ConstMat5.SetVal(2, 2, 3.0f / 4.0f); m_ConstMat5.SetVal(2, 3, 0.0f);
	m_ConstMat5.SetVal(3, 0, 1.0f / 6.0f); m_ConstMat5.SetVal(3, 1, 7.0f / 12.0f); m_ConstMat5.SetVal(3, 2, 0.25f); m_ConstMat5.SetVal(3, 3, 0.0f);

	return true;
}

float MotionData::InterpolatedCurveValue(MotionMarker *pMK, unsigned int nDOF, float time)
{
	if(pMK == NULL || !m_AreMarkersFitted || numFitPointTime == 0 || pFitPointTime == NULL)
		return 0.0f;

	float u;
	unsigned int index = 0;

	while(1)
	{
		if(time < pFitPointTime[0].time)
		{
			index = 0;
			break;
		}

		if(time >= pFitPointTime[index].time && time <= pFitPointTime[index + 1].time)
			break;
		
		if(time > pFitPointTime[numFitPointTime - 1].time)
		{
			index = numFitPointTime - 2;
			break;
		}
		
		index++;
	}

	//Calculate u for this segment of B-Spline using time.
	u = (time - pFitPointTime[index].time) / (pFitPointTime[index + 1].time - pFitPointTime[index].time);

	//Row poly matrix of u.
	Matrix poly_mat(1, 4);
	poly_mat.SetVal(0, 0, u * u * u);
	poly_mat.SetVal(0, 1, u * u);
	poly_mat.SetVal(0, 2, u);
	poly_mat.SetVal(0, 3, 1.0f);

	//Column matrices for interpolation.
	Matrix Amp_mat, cpAmp_mat(4, 1);

	cpAmp_mat.SetVal(0, 0, pMK->m_cbsc[nDOF].cps.cp[index].GetAmpVal());
	cpAmp_mat.SetVal(1, 0, pMK->m_cbsc[nDOF].cps.cp[index + 1].GetAmpVal());
	cpAmp_mat.SetVal(2, 0, pMK->m_cbsc[nDOF].cps.cp[index + 2].GetAmpVal());
	cpAmp_mat.SetVal(3, 0, pMK->m_cbsc[nDOF].cps.cp[index + 3].GetAmpVal());

	//Multiply different matrix according to different segments.
	if(index > 1 && index < numFitPointTime - 3)
	{
		m_ConstMat3.Product(&cpAmp_mat, &Amp_mat);
		Amp_mat.ScalarProduct(1.0f / 6.0f);
	}
	else
	{
		if(index == 0)
			m_ConstMat1.Product(&cpAmp_mat, &Amp_mat);

		if(index == 1)
			m_ConstMat2.Product(&cpAmp_mat, &Amp_mat);

		if(index == numFitPointTime - 3)
			m_ConstMat4.Product(&cpAmp_mat, &Amp_mat);

		if(index == numFitPointTime - 2)
			m_ConstMat5.Product(&cpAmp_mat, &Amp_mat);
	}

	//Finally multiply polynomal row matrix to Amp_mat.
	Matrix finalAmp_mat, finalTime_mat;
	poly_mat.Product(&Amp_mat, &finalAmp_mat);

	//Return interpolated value.
	return finalAmp_mat.GetVal(0, 0);
}

bool MotionData::FitAllMarkersToCBSC()
{
	if(m_AreMarkersFitted)
		return true;

	unsigned int j = 0;
	MotionMarker *pMK;

	if(m_MotionDataType != UNKNOWN_MOTION_TYPE)
	{
		ResetFitPointTimeArray();

		if(!BuildCoefMatrix(numCPs - 2))
			return false;

		if(!BuildInverseMatrix())
			return false;

		for(pMK = pMarkerHead; pMK != NULL; pMK = pMK->next)
		{
			if(!pMK->IsEndSite)
			{
				unsigned int i;
					
				for(i = 0; i < pMK->numDOFs; i++)
				{
					pMK->m_cbsc[i].pInverse_mat = m_pInvMat;
					pMK->m_cbsc[i].CalculateCPFromPP(numCPs, knotstep);
					pMK->m_cbsc[i].pInverse_mat = NULL;
				}

			}
		}

		ReleaseCoefMatrix();
		ReleaseInverseMatrix();
		m_AreMarkersFitted = true;
	}

	return true;
}

bool MotionData::ReleaseCoefMatrix()
{
	if(m_pCoefMat == NULL)
		return false;

	delete m_pCoefMat;
	m_pCoefMat = NULL;

	return true;
}

bool MotionData::BuildInverseMatrix()
{
	if(m_pCoefMat == NULL)
		return false;

	m_pInvMat = (*m_pCoefMat).Inverse();

	if(m_pInvMat == NULL)
		return false;

	return true;
}

bool MotionData::ReleaseInverseMatrix()
{
	if(m_pInvMat == NULL)
		return false;

	delete m_pInvMat;
	m_pInvMat = NULL;

	return true;
}

MotionData& MotionData::operator =(const MotionData& MD_in)
{
	if(this == &MD_in)
		return *this;

	pNext = NULL;

	numFrames = MD_in.numFrames;
	numMarkers = MD_in.numMarkers;
	numPPs = MD_in.numPPs;
	numCPs  =MD_in.numCPs;

	knotstep = MD_in.knotstep;
	numFitPointTime = MD_in.numFitPointTime;

	m_nMSeq = 0;
	FrameTime = MD_in.FrameTime;
	TimeLength = MD_in.TimeLength;

	DataRate = MD_in.DataRate;
	CameraRate = MD_in.CameraRate;

	m_AreMarkersFitted = MD_in.m_AreMarkersFitted;

	m_MotionDataType = MD_in.m_MotionDataType;

	// Clean up pre-allocated resource first.
	if(pMarkerHead != NULL)
	{
		MotionMarker *temp1, *temp2;

		temp1 = pMarkerHead;
		while(temp1 != NULL)
		{
			temp2 = temp1->next;
			delete temp1;
			temp1 = temp2;
		}

		pMarkerHead = NULL;
	}

	if(pFitPointTime != NULL)
	{
		delete[] pFitPointTime;
		pFitPointTime = NULL;
	}

	if(m_pCoefMat != NULL)
	{
		delete m_pCoefMat;
		m_pCoefMat = NULL;
	}

	if(m_pInvMat != NULL)
	{
		delete m_pInvMat;
		m_pInvMat = NULL;
	}

	if(m_pMSeq != NULL)
	{
		delete[] m_pMSeq;
		m_pMSeq = NULL;
	}

	MotionMarker *pMK, *pNode;
	for(pMK = MD_in.pMarkerHead; pMK != NULL; pMK = pMK->next)
	{
		pNode = new MotionMarker;

		*pNode = *pMK;
		InsertMarker(pNode);
	}

	LinkToParent(&MD_in, this);

	return *this;
}

// This function links all joints in pMD2 to its parents referring to hierarchy in pMD1.
bool MotionData::LinkToParent(const MotionData *pMD1, MotionData *pMD2)
{
	MotionMarker *pMK1;

	pMK1 = pMD1->pMarkerHead;
	//pMK2 = pMD2->pMarkerHead;

	for(pMK1 = pMD1->pMarkerHead; pMK1 != NULL; pMK1 = pMK1->next)
	{
		if(pMK1->parent != NULL)
			(pMD2->GetMarker(pMK1->nID))->parent = pMD2->GetMarker(pMK1->parent->nID);
	}

	return true;
}

bool MotionData::UnFitAllCurves()
{
	if(!m_AreMarkersFitted)
		return true;

	unsigned int i;
	MotionMarker *pMK;

	for(pMK = pMarkerHead; pMK != NULL; pMK = pMK->next)
	{
		for(i = 0; i < pMK->numDOFs; i++)
			pMK->m_cbsc[i].UnFitCurve();
	}

	numFitPointTime = 0;
	if(pFitPointTime != NULL)
	{
		delete[] pFitPointTime;
		pFitPointTime = NULL;
	}

	numCPs = 0;
	m_AreMarkersFitted = false;

	return true;
}

float MotionData::GetPathPointAmp(MotionMarker *pMK, unsigned int nFrame, unsigned int nDOF)
{
	float amp = 0.0f;

	if(m_MotionDataType != UNKNOWN_MOTION_TYPE && pMK == NULL && !pMK->IsEndSite)
		amp = pMK->m_cbsc[nDOF].pps.pp[nFrame].GetAmpVal();

	return amp;
}


MotionMarker * MotionData::GetMarkerByName(const char *MarkerName)
{
	MotionMarker *pMK;

	for(pMK = pMarkerHead; pMK != NULL; pMK = pMK->next)
	{
		if(strcmp(pMK->name, MarkerName) == 0)
			return pMK;
	}

	return NULL;
}

bool MotionData::UpdateMarkerTM(MotionMarker *pMK, unsigned int nFrame)
{
	MotionMarker *pChildMK;

	if(pMK->IsEndSite)
		return false;

	if(pMK == pMarkerHead)
	{
		float sx, cx, sy, cy, sz, cz;

		sx = (float)sin(pMK->m_cbsc[4].pps.pp[nFrame].GetAmpVal() * 0.017453292519943295769236907684886);
		cx = (float)cos(pMK->m_cbsc[4].pps.pp[nFrame].GetAmpVal() * 0.017453292519943295769236907684886);

		sy = (float)sin(pMK->m_cbsc[5].pps.pp[nFrame].GetAmpVal() * 0.017453292519943295769236907684886);
		cy = (float)cos(pMK->m_cbsc[5].pps.pp[nFrame].GetAmpVal() * 0.017453292519943295769236907684886);

		sz = (float)sin(pMK->m_cbsc[3].pps.pp[nFrame].GetAmpVal() * 0.017453292519943295769236907684886);
		cz = (float)cos(pMK->m_cbsc[3].pps.pp[nFrame].GetAmpVal() * 0.017453292519943295769236907684886);

		pMK->TranMat.SetToIdentity();
		pMK->TranMat.m[0] = cz * cy - sz * sx * sy;
		pMK->TranMat.m[1] = -sz * cx;
		pMK->TranMat.m[2] = cz * sy + sz * sx * cy;
		pMK->TranMat.m[3] = pMK->offset.x + pMK->m_cbsc[0].pps.pp[nFrame].GetAmpVal();

		pMK->TranMat.m[4] = sz * cy + cz * sx * sy;
		pMK->TranMat.m[5] = cz * cx;
		pMK->TranMat.m[6] = sz * sy - cz * sx * cy;
		pMK->TranMat.m[7] = pMK->offset.y + pMK->m_cbsc[1].pps.pp[nFrame].GetAmpVal();

		pMK->TranMat.m[8] = -cx * sy;
		pMK->TranMat.m[9] = sx;
		pMK->TranMat.m[10] = cx * cy;
		pMK->TranMat.m[11] = pMK->offset.z + pMK->m_cbsc[2].pps.pp[nFrame].GetAmpVal();

		pMK->CurrentPosition.x = pMK->TranMat.m[3];
		pMK->CurrentPosition.y = pMK->TranMat.m[7];
		pMK->CurrentPosition.z = pMK->TranMat.m[11];

		pMK->LocalRM.SetToIdentity();
		pMK->LocalRM.m[0] = pMK->TranMat.m[0];
		pMK->LocalRM.m[1] = pMK->TranMat.m[1];
		pMK->LocalRM.m[2] = pMK->TranMat.m[2];

		pMK->LocalRM.m[4] = pMK->TranMat.m[4];
		pMK->LocalRM.m[5] = pMK->TranMat.m[5];
		pMK->LocalRM.m[6] = pMK->TranMat.m[6];

		pMK->LocalRM.m[8] = pMK->TranMat.m[8];
		pMK->LocalRM.m[9] = pMK->TranMat.m[9];
		pMK->LocalRM.m[10] = pMK->TranMat.m[10];
	}

	for(pChildMK = pMarkerHead; pChildMK != NULL; pChildMK = pChildMK->next)
	{
		if(pChildMK->parent == pMK)
		{
			Matrix matRot(4, 4);

			if(!pChildMK->IsEndSite)
			{
				float sx, cx, sy, cy, sz, cz;

				sx = (float)sin(pChildMK->m_cbsc[1].pps.pp[nFrame].GetAmpVal() * 0.017453292519943295769236907684886);
				cx = (float)cos(pChildMK->m_cbsc[1].pps.pp[nFrame].GetAmpVal() * 0.017453292519943295769236907684886);

				sy = (float)sin(pChildMK->m_cbsc[2].pps.pp[nFrame].GetAmpVal() * 0.017453292519943295769236907684886);
				cy = (float)cos(pChildMK->m_cbsc[2].pps.pp[nFrame].GetAmpVal() * 0.017453292519943295769236907684886);

				sz = (float)sin(pChildMK->m_cbsc[0].pps.pp[nFrame].GetAmpVal() * 0.017453292519943295769236907684886);
				cz = (float)cos(pChildMK->m_cbsc[0].pps.pp[nFrame].GetAmpVal() * 0.017453292519943295769236907684886);

				matRot.SetToIdentity();
				matRot.m[0] = cz * cy - sz * sx * sy;
				matRot.m[1] = -sz * cx;
				matRot.m[2] = cz * sy + sz * sx * cy;
				matRot.m[3] = pChildMK->offset.x;

				matRot.m[4] = sz * cy + cz * sx * sy;
				matRot.m[5] = cz * cx;
				matRot.m[6] = sz * sy - cz * sx * cy;
				matRot.m[7] = pChildMK->offset.y;

				matRot.m[8] = -cx * sy;
				matRot.m[9] = sx;
				matRot.m[10] = cx * cy;
				matRot.m[11] = pChildMK->offset.z;

				pChildMK->LocalRM.SetToIdentity();
				pChildMK->LocalRM.m[0] = matRot.m[0];
				pChildMK->LocalRM.m[1] = matRot.m[1];
				pChildMK->LocalRM.m[2] = matRot.m[2];

				pChildMK->LocalRM.m[4] = matRot.m[4];
				pChildMK->LocalRM.m[5] = matRot.m[5];
				pChildMK->LocalRM.m[6] = matRot.m[6];

				pChildMK->LocalRM.m[8] = matRot.m[8];
				pChildMK->LocalRM.m[9] = matRot.m[9];
				pChildMK->LocalRM.m[10] = matRot.m[10];
			}
			else
			{
				matRot.SetToIdentity();
				matRot.m[3] = pChildMK->offset.x;
				matRot.m[7] = pChildMK->offset.y;
				matRot.m[11] = pChildMK->offset.z;

				pChildMK->LocalRM.SetToIdentity();
			}

			pMK->TranMat.Product(&matRot, &pChildMK->TranMat);

			pChildMK->CurrentPosition.x = pChildMK->TranMat.m[3];
			pChildMK->CurrentPosition.y = pChildMK->TranMat.m[7];
			pChildMK->CurrentPosition.z = pChildMK->TranMat.m[11];

			UpdateMarkerTM(pChildMK, nFrame);
		}
	}

	return true;
}

bool MotionData::UpdateVelAcc(unsigned int nFrame)
{
	if(nFrame >= numFrames)
		return false;

	if(nFrame == 0)
	{
		unsigned int i;
		MotionMarker *pMK;

		UpdateMarkerTM(pMarkerHead, nFrame);

		i = 0;
		for(pMK = pMarkerHead; pMK != NULL; pMK = pMK->next)
		{
			pMK->Velocity = Vector3D(0.0f, 0.0f, 0.0f);
			pMK->Acceleration = Vector3D(0.0f, 0.0f, 0.0f);
		}
	}

	if(nFrame == 1)
	{
		Vector3D *pRprev;

		pRprev = new Vector3D[numMarkers];
		
		unsigned int i;
		MotionMarker *pMK;

		UpdateMarkerTM(pMarkerHead, 0);
		i = 0;
		for(pMK = pMarkerHead; pMK != NULL; pMK = pMK->next)
		{
			pRprev[i] = pMK->CurrentPosition;
			i++;
		}

		UpdateMarkerTM(pMarkerHead, 1);
		i = 0;
		for(pMK = pMarkerHead; pMK != NULL; pMK = pMK->next)
		{
			pMK->Velocity = (pMK->CurrentPosition - pRprev[i]) * (1.0f / FrameTime);
			pMK->Acceleration = Vector3D(0.0f, 0.0f, 0.0f);
			i++;
		}		

		delete[] pRprev;
	}

	if(nFrame > 1)
	{
		Vector3D *pRprev1, *pRprev2, TempV;

		pRprev1 = new Vector3D[numMarkers];
		pRprev2 = new Vector3D[numMarkers];
		
		unsigned int i;
		MotionMarker *pMK;

		UpdateMarkerTM(pMarkerHead, nFrame - 2);
		i = 0;
		for(pMK = pMarkerHead; pMK != NULL; pMK = pMK->next)
		{
			pRprev2[i] = pMK->CurrentPosition;
			i++;
		}

		UpdateMarkerTM(pMarkerHead, nFrame - 1);
		i = 0;
		for(pMK = pMarkerHead; pMK != NULL; pMK = pMK->next)
		{
			pRprev1[i] = pMK->CurrentPosition;
			i++;
		}

		UpdateMarkerTM(pMarkerHead, nFrame);
		i = 0;
		for(pMK = pMarkerHead; pMK != NULL; pMK = pMK->next)
		{
			pMK->Velocity = (pMK->CurrentPosition - pRprev1[i]) * (1.0f / FrameTime);
			
			TempV = (pRprev1[i] - pRprev2[i]) * (1.0f / FrameTime);

			pMK->Acceleration = (pMK->Velocity - TempV) * (1.0f/ FrameTime);

			i++;
		}		

		delete[] pRprev1;
		delete[] pRprev2;
	}

	return true;
}
