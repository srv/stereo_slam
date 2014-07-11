//	pbpolys.cpp - basic operations with vertices and polygons
//
//	This file is a part of PolyBoolean software library
//	(C) 1998-1999 Michael Leonov
//	Consult your license regarding permissions and restrictions
//
//	fix: 2006 Oct 2 Alexey Nikitin
//		PLINE2::Prepare / left middle point from 3 points on the same line

#include "polybool.h"
#include "pbimpl.h"
//#include <stdio.h>
#include <iostream>
using namespace std;
#define SETBITS(n,mask,s) (((n)->Flags & ~(mask)) | ((s) & mask))

namespace POLYBOOLEAN
{

////////////////////// class VNODE2 /////////////////////////////

VNODE2 * VNODE2::New(const GRID2 & g)
{
	VNODE2 * res = (VNODE2 *)calloc(1, sizeof(VNODE2));
	if (res == NULL)
		error(err_no_memory);
	res->g = g;
	res->prev = res->next = res;
	return res;
} // VNODE2::New

//////////////////// class PLINE2 /////////////////////////


void AdjustBox(PLINE2 * pline, const GRID2 & g)
{
	if (pline->gMin.x > g.x)
		pline->gMin.x = g.x;
	if (pline->gMin.y > g.y)
		pline->gMin.y = g.y;
	if (pline->gMax.x < g.x)
		pline->gMax.x = g.x;
	if (pline->gMax.y < g.y)
		pline->gMax.y = g.y;
} // AdjustBox

local
bool GridInBox(const GRID2 & g, const PLINE2 * pline)
{
	return (g.x > pline->gMin.x and g.y > pline->gMin.y and
			g.x < pline->gMax.x and g.y < pline->gMax.y);
} // GridInBox

local
bool BoxInBox(const PLINE2 * c1, const PLINE2 * c2)
{
	return (c1->gMin.x >= c2->gMin.x and
            c1->gMin.y >= c2->gMin.y and
            c1->gMax.x <= c2->gMax.x and
            c1->gMax.y <= c2->gMax.y);
} // BoxInBox

void PLINE2::Del(PLINE2 ** pline)
{
    if (*pline == NULL)
		return;

    VNODE2 *vn;
    while ((vn = (*pline)->head->next) != (*pline)->head)
        vn->Excl(), free(vn);
    free(vn);
    free(*pline), *pline = NULL;
} // PLINE2::Del

PLINE2 * PLINE2::New(const GRID2 &g)
{
    PLINE2 *res = (PLINE2 *)calloc(1, sizeof(PLINE2));
	if (res == NULL)
        error(err_no_memory);
	try
	{
		res->head = VNODE2::New(g);
	}
	catch (int)
	{
		free(res);
		throw;
	}
    res->gMax = res->gMin = g;
    res->Count = 1;
    return res;
} // PLINE2::New

local
void InclPlineVnode(PLINE2 *c, VNODE2 *vn)
{
    assert(vn != NULL and c != NULL);
    vn->Incl(c->head->prev);
    c->Count++;
    AdjustBox(c, vn->g);
} // InclPlineVnode

void PLINE2::Incl(PLINE2 ** pline, const GRID2 & g)
{
	if (*pline == NULL)
        *pline = PLINE2::New(g);
	else
		InclPlineVnode(*pline, VNODE2::New(g));
} // PLINE2::Incl

PLINE2 *	PLINE2::Copy(bool bMakeLinks) const
{
	PLINE2 * dst = NULL;
    VNODE2*	vn = head;
	try
	{
		do {
			Incl(&dst, vn->g);
            if (bMakeLinks)
                (vn->vn = dst->head->prev)->vn = vn;
		} while ((vn = vn->next) != head);
	}
	catch (int)
	{
		Del(&dst);
		throw;
	}
    dst->Count = Count;
    dst->Flags = Flags;
    dst->gMin  = gMin;
    dst->gMax  = gMax;
	return dst;
} // PLINE2::Copy

// returns if b lies on (a,c)
local
bool PointOnLine(const GRID2 & a, const GRID2 & b, const GRID2 & c)
{
	return	(INT64)(b.x - a.x) * (c.y - b.y) ==
			(INT64)(b.y - a.y) * (c.x - b.x);
} // PointOnLine

bool PLINE2::Prepare()
{
    gMin.x = gMax.x = head->g.x;
    gMin.y = gMax.y = head->g.y;

	// remove coincident vertices and those lying on the same line
	VNODE2 * p = head;
	VNODE2 * c = p->next;
	//cout << "Count" << Count;
	for (int test = 0, tests = Count; test < tests; ++test)
	{
		VNODE2 * n = c->next;

		if (PointOnLine(p->g, c->g, n->g))
		{
			++tests;
			if (n == head)
				++tests;

			--Count;
			if (Count < 3)
				return false;

			if (c == head)
				head = p;

			c->Excl(), free(c);
			p = (c = p)->prev;
		}
		else
		{
			p = c;
			c = n;
		}
	}

	c = head;
    p = c->prev;
    INT64  nArea = 0;

	do {
		nArea += (INT64)(p->g.x - c->g.x) * (p->g.y + c->g.y);
		AdjustBox(this, c->g);
	} while ((c = (p = c)->next) != head);
	
	if (nArea == 0)
		return false;
    
	Flags = SETBITS(this, ORIENT, (nArea < 0) ? INV : DIR);
	return true;
} // PLINE2::Prepare

// Prepare2 computes the area under a closed polygon. Returning the area value. If 0--> error; area=point

double  PLINE2::Prepare2()
{
	    gMin.x = gMax.x = head->g.x;
	    gMin.y = gMax.y = head->g.y;

		// remove coincident vertices and those lying on the same line
		VNODE2 * p ;
		VNODE2 * c ;

		c = head;
	    p = c->prev;
	    double  nArea = 0.0;

		do {
			nArea += (double)(p->g.x - c->g.x) * (p->g.y + c->g.y);
			AdjustBox(this, c->g);
		} while ((c = (p = c)->next) != head);
		
		Flags = SETBITS(this, ORIENT, (nArea < 0) ? INV : DIR);
		return nArea;
}

void PLINE2::Invert()
{
    VNODE2 *vn = head, *next;
    do {
        next = vn->next, vn->next = vn->prev, vn->prev = next;
    } while ((vn = next) != head);

    Flags ^= ORIENT;
} // PLINE2::Invert

/********************** PAREA stuff **********************************/

PAREA *PAREA::New()
{
    PAREA *res = (PAREA *)calloc(1, sizeof(PAREA));
    if (res == NULL)
		error(err_no_memory);
    return res->f = res->b = res;
} // PAREA::New

local
void ClearArea(PAREA *P)
{
    PLINE2 *p;

    assert(P != NULL);
    while ((p = P->cntr) != NULL)
    {
        P->cntr = p->next;
		PLINE2::Del(&p);
    }
    free(P->tria), P->tria = NULL;
    P->tnum = 0;
} // ClearArea

void PAREA::Del(PAREA ** p)
{
    if (*p == NULL)
		return;
    PAREA *cur;
    while ((cur = (*p)->f) != *p)
    {
        (cur->b->f = cur->f)->b = cur->b;
        ClearArea(cur);
        free(cur);
    }
    ClearArea(cur);
    free(cur), *p = NULL;
} // PAREA::Del

local
void InclPareaPline(PAREA * p, PLINE2 * c)
{
    assert(c->next == NULL);

    if (c->IsOuter())
    {
        assert(p->cntr == NULL);
        p->cntr = c;
    }
    else
    {
        assert(p->cntr != NULL);
        c->next = p->cntr->next, p->cntr->next = c;
    }
} // InclPareaPline

void PAREA::InclParea(PAREA **list, PAREA *a)
{
	if (a != NULL)
	{
		if (*list == NULL)
			*list = a;
		else
			(((a->b = (*list)->b)->f = a)->f = *list)->b = a;
	}
} // PAREA::InclParea

void PAREA::InclPline(PAREA ** area, PLINE2 * pline)
{
	assert(pline != NULL and pline->next == NULL);
	PAREA * t;
    if (pline->IsOuter())
	{
		t = New();
		InclParea(area, t);
	}
    else
    {
		assert(*area != NULL);
        // find the smallest container for the hole
        t = NULL;
        PAREA *	pa = *area;
        do {
            if (PlineInPline(pline, pa->cntr) and
               (t == NULL or PlineInPline(pa->cntr, t->cntr)))
                t = pa;
        } while ((pa = pa->f) != *area);

		if (t == NULL)	// couldn't find a container for the hole
			error(err_bad_parm);
    }
    InclPareaPline(t, pline);
} // PAREA::InclPline

void PLINE2::Put(PLINE2 * pline, PAREA ** area, PLINE2 ** holes)
{
	assert(pline->next == NULL);
    if (pline->IsOuter())
        PAREA::InclPline(area, pline);
    else
        pline->next = *holes, *holes = pline;
} // PLINE2::Put

void PAREA::InsertHoles(PAREA ** area, PLINE2 ** holes)
{
    if (*holes == NULL)
		return;
    if (*area == NULL)
		error(err_bad_parm);
    
    while (*holes != NULL)
    {
		PLINE2 * next = (*holes)->next;
		(*holes)->next = NULL;
        InclPline(area, *holes);
        *holes = next;
    }
} // PAREA::InsertHoles

local
PAREA * PareaCopy0(const PAREA * area)
{
	PAREA * dst = NULL;

	try
	{
		for (const PLINE2 * pline = area->cntr; pline != NULL; pline = pline->next)
			PAREA::InclPline(&dst, pline->Copy(true));

		if (area->tria == NULL)
			return dst;
		dst->tria = (PTRIA2 *)calloc(area->tnum, sizeof(PTRIA2));
		if (dst->tria == NULL)
			error(err_no_memory);
	}
	catch (int)
	{
		PAREA::Del(&dst);
		throw;
	}
    dst->tnum = area->tnum;
    for (UINT32 i = 0; i < area->tnum; i++)
    {
        PTRIA2 * td =  dst->tria + i;
        PTRIA2 * ts = area->tria + i;

        td->v0 = ts->v0->vn;
        td->v1 = ts->v1->vn;
        td->v2 = ts->v2->vn;
    }
	return dst;
} // PareaCopy0

PAREA * PAREA::Copy() const
{
    PAREA * dst = NULL;
    const PAREA *src = this;
	try
	{
		do {
			PAREA *di = PareaCopy0(src);
			InclParea(&dst, di);
		} while ((src = src->f) != this);
	}
	catch (int)
	{
		Del(&dst);
		throw;
	}
	return dst;
} // PAREA::Copy

bool GridInPline(const GRID2 * g, const PLINE2 * outer)
{
    if (outer == NULL)
		return false;
    if (!GridInBox(*g, outer))
        return false;

    const VNODE2 *vn = outer->head;
    bool bInside = false;
    do {
	    const GRID2 & vc = vn->g;
	    const GRID2 & vp = vn->prev->g;

        if		(vc.y <= g->y and g->y < vp.y and
			(INT64)(vp.y - vc.y) * (g->x - vc.x) <
			(INT64)(g->y - vc.y) * (vp.x - vc.x))
		{
            bInside = not bInside;
		}
		else if (vp.y <= g->y and g->y < vc.y and
			(INT64)(vp.y - vc.y) * (g->x - vc.x) >
			(INT64)(g->y - vc.y) * (vp.x - vc.x))
		{
            bInside = not bInside;
		}
    } while ((vn = vn->next) != outer->head);
    return bInside;
} // GridInPline

bool PlineInPline(const PLINE2 * p, const PLINE2 * outer)
{
    assert(p != NULL);
    return outer != NULL && BoxInBox(p, outer) and
           GridInPline(&p->head->g, outer);
} // PlineInPline

typedef bool (*INSIDE_PROC)(const void *p, const PLINE2 *outer);

local
bool PolyInside(const void *p, const PAREA * outfst, INSIDE_PROC ins_actor)
{
    const PAREA * pa = outfst;

    if (pa == NULL)
		return false;
    assert(p != NULL && pa != NULL);
    do {
	    const PLINE2 *curc = pa->cntr;
		if (ins_actor(p, curc))
		{
			while ((curc = curc->next) != NULL)
				if (ins_actor(p, curc))
					goto Proceed;
			return true;
		}
Proceed: ;
    } while ((pa = pa->f) != outfst);
    return false;
} // PolyInside

bool GridInParea(const GRID2 * g, const PAREA * outer)
{
    return PolyInside(g, outer, (INSIDE_PROC)GridInPline);
} // GridInParea

bool PlineInParea(const PLINE2* p, const PAREA * outer)
{
    return PolyInside(p, outer, (INSIDE_PROC)PlineInPline);
} // PlineInParea

#ifndef NDEBUG

local
bool Chk(INT32 x)
{
	return INT20_MIN > x or x > INT20_MAX;
}

bool PAREA::CheckDomain()
{
	PAREA * pa = this;
	do {
		for (PLINE2 * pline = pa->cntr; pline != NULL; pline = pline->next)
		{
			if (Chk(pline->gMin.x) or 
				Chk(pline->gMin.y) or 
				Chk(pline->gMax.x) or 
				Chk(pline->gMax.y))
				return false;
		}
	} while ((pa = pa->f) != this);
	return true;
} // PAREA::CheckDomain

#endif // NDEBUG

} // namespace POLYBOOLEAN

