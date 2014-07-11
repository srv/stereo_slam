#ifndef _PBIO_H_
#define _PBIO_H_

#include "polybool.h"

namespace POLYBOOLEAN
{

void LoadPline(const char * szFname, PLINE2 ** pline);
void LoadParea(const char * szFname, PAREA ** area);
void SavePline(const char * szFname, const PLINE2 * pline);
void SaveParea(const char * szFname, const PAREA * area);

// calculates area bounding box
void CalcPareaBox(PAREA * area, VECT2 * vMin, VECT2 * vMax);

// scale area to grid, may delete contours with null area
void PareaToGrid(PAREA ** area, const VECT2 & vMin, const VECT2 & vMax);

// scale area from grid
void PareaFromGrid(PAREA * area, const VECT2 & vMin, const VECT2 & vMax);

} // namespace POLYBOOLEAN

#endif // _PBIO_H_

