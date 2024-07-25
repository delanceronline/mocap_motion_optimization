// Cylinder.cpp: implementation of the Cylinder class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "Cylinder.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

Cylinder::Cylinder()
{
	pVecWrist = NULL;
	pVecElbow = NULL;
	pWristRadius = NULL;
	pWristHeight = NULL;
	pElbowRadius = NULL;
	pElbowHeight = NULL;
}

Cylinder::~Cylinder()
{
	if(pVecWrist != NULL)
		delete[] pVecWrist;

	if(pVecElbow != NULL)
		delete[] pVecElbow;

	if(pWristRadius != NULL)
		delete[] pWristRadius;

	if(pWristHeight != NULL)
		delete[] pWristHeight;

	if(pElbowRadius != NULL)
		delete[] pElbowRadius;

	if(pElbowHeight != NULL)
		delete[] pElbowHeight;
}

Vector3D Cylinder::GetElbowAbsPos(unsigned int nFrame)
{
	Vector3D p;

	p = r1 + (r2 - r1) * pElbowHeight[nFrame] + pVecElbow[nFrame] * pElbowRadius[nFrame];

	return p;
}

Vector3D Cylinder::GetWristAbsPos(unsigned int nFrame)
{
	Vector3D p;

	p = r1 + (r2 - r1) * pWristHeight[nFrame] + pVecWrist[nFrame] * pWristRadius[nFrame];

	return p;
}

bool Cylinder::LoadCylinder(char *filename)
{
	unsigned int i, numFrames;
	FILE *pStream = fopen(filename, "r");

	if(pStream == NULL)
		return false;

	fscanf(pStream, "%d\n", &numFrames);
	numPoints = numFrames;
	pVecElbow = new Vector3D[numFrames];
	pVecWrist = new Vector3D[numFrames];
	pElbowRadius = new float[numFrames];
	pElbowHeight = new float[numFrames];
	pWristRadius = new float[numFrames];
	pWristHeight = new float[numFrames];
	fscanf(pStream, "r1: %f %f %f\n", &r1.x, &r1.y, &r1.z);
	fscanf(pStream, "r2: %f %f %f\n", &r2.x, &r2.y, &r2.z);
	for(i = 0; i < numFrames; i++)
	{
		fscanf(pStream, "%f %f\n", &pElbowRadius[i], &pElbowHeight[i]);
		fscanf(pStream, "%f %f %f\n", &pVecElbow[i].x, &pVecElbow[i].y, &pVecElbow[i].z);
	}
	for(i = 0; i < numFrames; i++)
	{
		fscanf(pStream, "%f %f\n", &pWristRadius[i], &pWristHeight[i]);
		fscanf(pStream, "%f %f %f\n", &pVecWrist[i].x, &pVecWrist[i].y, &pVecWrist[i].z);
	}
	fclose(pStream);	

	return true;
}

bool Cylinder::SaveCylinder(char *filename)
{
	unsigned int i;
	FILE *pStream = fopen(filename, "w");

	if(pStream == NULL)
		return false;
	
	fprintf(pStream, "%d\n", numPoints);
	fprintf(pStream, "r1: %f %f %f\n", r1.x, r1.y, r1.z);
	fprintf(pStream, "r2: %f %f %f\n", r2.x, r2.y, r2.z);
	for(i = 0; i < numPoints; i++)
	{
		fprintf(pStream, "%f %f\n", pElbowRadius[i], pElbowHeight[i]);
		fprintf(pStream, "%f %f %f\n", pVecElbow[i].x, pVecElbow[i].y, pVecElbow[i].z);
	}
	for(i = 0; i < numPoints; i++)
	{
		fprintf(pStream, "%f %f\n", pWristRadius[i], pWristHeight[i]);
		fprintf(pStream, "%f %f %f\n", pVecWrist[i].x, pVecWrist[i].y, pVecWrist[i].z);
	}
	fclose(pStream);	

	return true;
}
