// pbunit.cpp : Defines the entry point for the console application.
//
#include <stdlib.h>
#include "polybool.h" 
using namespace POLYBOOLEAN; 
int main() 
{ 
	static const GRID2 a1[4] = 
	{ {-7, 8}, {-7, -3}, {2, -3}, {2, 8} }; 
	static const GRID2 a2[4] = 
	{ {-5, 6}, {0, 6}, {0, 0}, {-5, 0} }; 
	static const GRID2 b[11] = 
	{ {-5, -6}, {7,-6}, {7, 4}, {-5, 4}, {0, 0}, {0, 2}, 
	{5, 2}, {5, -4}, {0, -4}, {0, 0}, {-5, 0} }; 
	PAREA * A = NULL; 
	PAREA * B = NULL; 
	int     i; 
	PLINE2 * pline = NULL; 

	// construct 1st polygon 
	for (i = 0; i < 4; i++) 

	PLINE2::Incl(&pline, a1[i]);
	pline->Prepare(); 
	if (not pline->IsOuter()) // make sure the contour is outer 
	pline->Invert();
	PAREA::InclPline(&A, pline); 
	pline = NULL; 
	for (i = 0; i < 4; i++) 

	PLINE2::Incl(&pline, a2[i]);
	pline->Prepare(); 
	if (pline->IsOuter()) // make sure the contour is a hole 
	pline->Invert();
	PAREA::InclPline(&A, pline); 
	// construct 2nd polygon 
	pline = NULL; 
	for (i = 0; i < 11; i++) 

	PLINE2::Incl(&pline, b[i]);
	pline->Prepare(); 
	if (not pline->IsOuter()) // make sure the contour is outer 
	pline->Invert();
	PAREA::InclPline(&B, pline); 
	// do Boolean operation XOR
	PAREA * R = NULL; 
	int err = PAREA::Boolean(A, B, &R, PAREA::XR);

	// triangulate R 
	err = PAREA::Triangulate(R);

	// delete all polygons 
	PAREA::Del(&A); 
	PAREA::Del(&B); 
	PAREA::Del(&R);

	return err;
} 
