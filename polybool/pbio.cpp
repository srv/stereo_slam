#include <float.h>
#include <stdio.h>

#include "pbio.h"
#include "pbimpl.h"

namespace POLYBOOLEAN
{

/////////////////////// file I/O stuff ////////////////////////////

local
void LoadPLINE(FILE * f, PLINE2 **pline)
{
	*pline = NULL;

    int	nCount;
	if (fscanf(f, "%d", &nCount) != 1)
		error(err_io);
    if (nCount < 3)
		error(err_bad_parm);
    for (int i = 0; i < nCount; i++)
    {
	    GRID2 g;
		if (fscanf(f, "%d,%d", &g.x, &g.y) != 2)
			error(err_io);
		PLINE2::Incl(pline, g);
    }
} // LoadPLINE

local
void LoadPAREA(FILE *f, PAREA ** area)
{
    int paCount;

    *area = NULL;
	if (fscanf(f, "%d", &paCount) != 1 or paCount < 1)
		error(err_io);
    for (int j = 0; j < paCount; j++)
    {
		int plCount;
		if (fscanf(f, "%d", &plCount) != 1 or plCount < 1)
			error(err_io);

        for (int i = 0; i < plCount; i++)
        {
			bool	bOuter = (i == 0);
			PLINE2 * cntr;

            try
			{
				LoadPLINE(f, &cntr);
			}
			catch (int)
			{
				PLINE2::Del(&cntr);
				PAREA::Del(area);
                throw;
			}
			if (not cntr->Prepare())
			{
				PLINE2::Del(&cntr);
				PAREA::Del(area);
				error(err_bad_parm);
			}
			if (cntr->IsOuter() != bOuter)
				cntr->Invert();

			PAREA::InclPline(area, cntr);
        }
	}
} // LoadPAREA

void LoadPline(const char *szFname, PLINE2 **pline)
{
    FILE * f = fopen(szFname, "r");
    if (f == NULL)
		error(err_io);
	LoadPLINE(f, pline);
	if (fclose(f) != 0)
		error(err_io);
} // LoadPline

void LoadParea(const char *szFname, PAREA **area)
{
    FILE * f = fopen(szFname, "r");
    if (f == NULL)
		error(err_io);
	LoadPAREA(f, area);
	if (fclose(f) != 0)
		error(err_io);
} // LoadParea

/*///////////////////// Save to file stuff /////////////////////////*/

local
void SavePLINE(FILE * f, const PLINE2 * pline)
{
    if (fprintf(f, "%d\n", pline->Count) < 0)
		error(err_io);

    const VNODE2 * vn = pline->head;
    do {
		if (fprintf(f, "%d,%d\n", vn->g.x, vn->g.y) < 0)
			error(err_io);
    } while ((vn = vn->next) != pline->head);
} // SavePLINE

local
void SavePAREA(FILE * f, const PAREA *area)
{
	int i = 0;
	const PAREA * pa = area;
    do {
        i++;
    } while ((pa = pa->f) != area);

    if (fprintf(f, "%d\n", i) < 0)
		error(err_io);
    do {
		i = 0;
        for (const PLINE2 * pline = pa->cntr; pline != NULL; pline = pline->next)
            i++;

        if (fprintf(f, "%d\n", i) < 0)
			error(err_io);

        for (const PLINE2 * pline = pa->cntr; pline != NULL; pline = pline->next)
            SavePLINE(f, pline);
    } while ((pa = pa->f) != area);
} // SavePAREA

void SavePline(const char *szFname, const PLINE2 * pline)
{
	FILE * f = fopen(szFname, "w");
    if (f == NULL)
		error(err_io);
	SavePLINE(f, pline);
	if (fclose(f) != 0)
		error(err_io);
} // SavePline

void SaveParea(const char *szFname, const PAREA * area)
{
	FILE * f = fopen(szFname, "w");
    if (f == NULL)
		error(err_io);
	SavePAREA(f, area);
	if (fclose(f) != 0)
		error(err_io);
} // SaveParea

///////////////////////// scale stuff ///////////////////////////

template<class TYPE>
local void MinMax(TYPE dVal, TYPE * dMin, TYPE * dMax)
{
	if (*dMin > dVal)
		*dMin = dVal;
	if (*dMax < dVal)
		*dMax = dVal;
} // MinMax

// calculates area bounding box
void CalcPareaBox(PAREA * area, VECT2 * vMin, VECT2 * vMax)
{
	if (area == NULL)
		return;
	PAREA * pa = area;
	do {
		for (PLINE2 * pline = pa->cntr; pline != NULL; pline = pline->next)
		{
			VNODE2 * vn = pline->head;
			do {
				MinMax<double>(vn->p.x, &vMin->x, &vMax->x);
				MinMax<double>(vn->p.y, &vMin->y, &vMax->y);
			} while ((vn = vn->next) != pline->head);
		}
	} while ((pa = pa->f) != area);
} // CalcPareaBox

local
void ToGrid(GRID2 * g, const VECT2 & v, const VECT2 & vMin, const VECT2 & vMax)
{
	assert(vMax.x > vMin.x);
	assert(vMax.y > vMin.y);
	assert(vMin.x <= v.x and v.x <= vMax.x);
	assert(vMin.y <= v.y and v.y <= vMax.y);

	g->x = INT20_MIN + (INT32)floor(0.5 + (v.x - vMin.x) / (vMax.x - vMin.x) * (INT20_MAX - INT20_MIN));
	g->y = INT20_MIN + (INT32)floor(0.5 + (v.y - vMin.y) / (vMax.y - vMin.y) * (INT20_MAX - INT20_MIN));

	assert(INT20_MIN <= g->x and g->x <= INT20_MAX);
	assert(INT20_MIN <= g->y and g->y <= INT20_MAX);
} // ToGrid

// scale area to grid, may delete contours with null area
void PareaToGrid(PAREA ** area, const VECT2 & vMin, const VECT2 & vMax)
{
	if (*area == NULL)
		return;
	PAREA * pa = *area;
	for (;;)
	{
		PLINE2 * PrevPline = NULL;
		bool bValid = true;
		for (PLINE2 * pline = pa->cntr; pline != NULL; pline = (PrevPline = pline)->next)
		{
			pline->gMin.x = pline->gMin.y = INT20_MAX;
			pline->gMax.x = pline->gMax.y = INT20_MIN;

			VNODE2 * vn = pline->head;
			GRID2 g;
			do {
				ToGrid(&g, vn->p, vMin, vMax);
				MinMax<INT32>(g.x, &pline->gMin.x, &pline->gMax.x);
				MinMax<INT32>(g.y, &pline->gMin.y, &pline->gMax.y);
				if (vn == pline->head or vn->prev->g != g)
					vn->g = g;
				else
				{
					VNODE2 * save = vn->prev;
					vn->Excl(), pline->Count--, vn = save;
				}
			} while ((vn = vn->next) != pline->head);

			if (pline->Count >= 3)
				continue;

			// here we have contour with null area after scaling
			if (pline == pa->cntr)
			{
				bValid = false;
				break;
			}

			// untie hole
			PrevPline->next = pline->next, pline = PrevPline;
		} // for (pline)
		if (bValid)
		{
			if ((pa = pa->f) == *area)
				break;
		}
		else if (pa != *area)
		{
			PAREA * save = pa->f;
			pa->Excl(), PAREA::Del(&pa), pa = save;
			if (pa == *area)
				break;
		}
		else if (pa->f == pa)
		{
			PAREA::Del(area);
			return;
		}
		else
		{
			PAREA * save = pa->f;
			pa->Excl(), PAREA::Del(&pa), *area = pa = save;
		}
	}; // for (area)
} // PareaToGrid

local
void FromGrid(VECT2 * v, const GRID2 & g, const VECT2 & vMin, const VECT2 & vMax)
{
	assert(vMax.x > vMin.x);
	assert(vMax.y > vMin.y);
	assert(INT20_MIN <= g.x and g.x <= INT20_MAX);
	assert(INT20_MIN <= g.y and g.y <= INT20_MAX);

	v->x = vMin.x + (vMax.x - vMin.x) * (g.x - INT20_MIN) / (INT20_MAX - INT20_MIN);
	v->y = vMin.y + (vMax.y - vMin.y) * (g.y - INT20_MIN) / (INT20_MAX - INT20_MIN);
} // FromGrid

// scale area from grid
void PareaFromGrid(PAREA * area, const VECT2 & vMin, const VECT2 & vMax)
{
	if (area == NULL)
		return;
	PAREA * pa = area;
	do {
		for (PLINE2 * pline = pa->cntr; pline != NULL; pline = pline->next)
		{
			VNODE2 * vn = pline->head;
			VECT2 v;
			do {
				FromGrid(&v, vn->g, vMin, vMax), vn->p = v;
			} while ((vn = vn->next) != pline->head);
				FromGrid(&v, pline->gMin, vMin, vMax), pline->vMin = v;
				FromGrid(&v, pline->gMax, vMin, vMax), pline->vMax = v;
		}
	} while ((pa = pa->f) != area);
} // PareaFromGrid

} // namespace POLYBOOLEAN

