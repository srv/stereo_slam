//	triamono.cpp - trapezoids to triangles conversion routines
//
//	This file is a part of PolyBoolean software library
//	(C) 1998-1999 Michael Leonov
//	Consult your license regarding permissions and restrictions
//

#include "pbtria.h"

namespace POLYBOOLEAN
{

/* (v0, v1) is the new diagonal to be added to the polygon. Find which
   chain to use and return the positions of v0 and v1 in p and q.

   p is identified as follows. Scan from (v0, v1) rightwards till
   you hit the first segment starting from v0. That chain is the
   chain of our interest */
local
unsigned int GetVertexPosition(const VNODE2 *vp0, const VNODE2 *vp1)
{
    const GRID2   &	v0 = vp0->g;
    const V_DESC  *	vd = VN_D(vp0);
    const VNODE2 *const * vnext = vd->vnext;
    GRID2 a = vp1->g - v0;
    GRID2 q = vnext[0]->g - v0;
    unsigned int vp = 0;

    for (unsigned int i = 1; i < vd->nextfree; i++)
    {
        GRID2 t = vnext[i]->g - v0;

        if (GRID2::AngleInside(t, q, a))
            q = t, vp = i;
    }
    return vp;
} /* GetVertexPosition */

/* v0 and v1 are specified in anti-clockwise order with respect to
 * the current monotone polygon mcur. Split the current polygon into
 * two polygons using the diagonal (v0, v1)
 */
MCHAIN ** TRIAGLOBS::MakeNewMonotonePoly(MCHAIN ** mcur, VNODE2 *vp0, VNODE2 *vp1)
{
    MCHAIN  *p, *q, *i, *j, **mnew = newmon();
    unsigned int ip = GetVertexPosition(vp0, vp1);
	unsigned int iq = GetVertexPosition(vp1, vp0);
    V_DESC  *vd0 = VN_D(vp0);
    V_DESC  *vd1 = VN_D(vp1);

    p = vd0->mpos[ip];
    q = vd1->mpos[iq];

    /* At this stage, we have got the positions of v0 and v1 in the */
    /* desired chain. Now modify the linked lists */

    i = NEW_MCHAIN();  /* for the new list */
    j = NEW_MCHAIN();

    i->vn = vp0, j->vn = vp1;

    (((i->n = p->n)->p = i)->p =
      (j->p = q->p)->n = j)->n = i;
    (p->n = q)->p = p;

    vd0->vnext[ip] = vp1;

    assert(vd0->nextfree < 4);
    assert(vd1->nextfree < 4);

    vd0->mpos[vd0->nextfree] = i;
    vd1->mpos[vd1->nextfree] = j;

    vd0->vnext[vd0->nextfree++] = i->n->vn;
    vd1->vnext[vd1->nextfree++] = vp0;

    *mcur = p;
    *mnew = i;
    return mnew;
} /* MakeNewMonotonePoly */

void TRIAGLOBS::TraverseCusps(MCHAIN **mcur, TRAP *t, TRAP *from, TRAP *d0, TRAP *d1, bool fst)
{
    TraverseDir up, dn;

	if (fst)
		up = TR_FROM_UP, dn = TR_FROM_DN;
	else
		up = TR_FROM_DN, dn = TR_FROM_UP;

    if (d0 != NULL && d1 != NULL)
    { // up/downward opening triangle
		VNODE2 * vp0, * vp1;
        if (fst)
            vp0 = t->d1->l, vp1 = t->l;
        else
            vp0 = t->r,		vp1 = t->u0->r;

        if (from == d1)
        {
			MCHAIN  **mnew = MakeNewMonotonePoly( mcur, vp1, vp0);

            TraversePolygon( mcur, d1, t, up);
            TraversePolygon( mnew, d0, t, up);
        }
        else
        {
			MCHAIN  **mnew = MakeNewMonotonePoly( mcur, vp0, vp1);

            TraversePolygon( mcur, d0, t, up);
            TraversePolygon( mnew, d1, t, up);
        }
    }
    else
    {
        TraversePolygon( mcur, d0, t, up);
        assert(d1 == NULL);
    }
} /* TraverseCusps */

void TRIAGLOBS::OnlyCusp(MCHAIN **mcur, TRAP *t, TRAP *from, TraverseDir dir,
                    TRAP *u0, TRAP *u1, TRAP *d0, TRAP *d1, bool fst,
                    const GRID2 * chk1, const GRID2 * chk2)
{
    TraverseDir up, dn;

	if (fst)
		up = TR_FROM_UP, dn = TR_FROM_DN;
	else
		up = TR_FROM_DN, dn = TR_FROM_UP;

    assert(d1 == NULL);

    if (*chk1 == *chk2)
    {
	    VNODE2 * vp0, * vp1;
        if (fst)
            vp0 = t->u0->r, vp1 = t->l->next;
        else
            vp0 = t->l,		vp1 = t->d1->l;

        if (dir == up && u0 == from)
        {
            MCHAIN  **mnew = MakeNewMonotonePoly( mcur, vp1, vp0);
            TraversePolygon( mcur, u0, t, dn);
            TraversePolygon( mnew, d0, t, up);
            TraversePolygon( mnew, u1, t, dn);
        }
        else
        {
            MCHAIN  **mnew = MakeNewMonotonePoly( mcur, vp0, vp1);
            TraversePolygon( mcur, u1, t, dn);
            TraversePolygon( mcur, d0, t, up);
            TraversePolygon( mnew, u0, t, dn);
        }
    }
    else
    {
	    VNODE2 * vp0, * vp1;
        if (fst)
            vp0 = t->r,		vp1 = t->u0->r;
        else
            vp0 = t->d1->l,	vp1 = t->r->next;

        if (dir == up && u1 == from)
        {
            MCHAIN  **mnew = MakeNewMonotonePoly( mcur, vp1, vp0);
            TraversePolygon( mcur, u1, t, dn);
            TraversePolygon( mnew, d0, t, up);
            TraversePolygon( mnew, u0, t, dn);
        }
        else
        {
            MCHAIN  **mnew = MakeNewMonotonePoly( mcur, vp0, vp1);
            TraversePolygon( mcur, u0, t, dn);
            TraversePolygon( mcur, d0, t, up);
            TraversePolygon( mnew, u1, t, dn);
        }
    }
} /* OnlyCusp */

void TRIAGLOBS::NoCusp(TRAP *t, MCHAIN **mcur, VNODE2 *vp0, VNODE2 *vp1, TraverseDir dir)
{
    assert(t->u1 == NULL && t->d1 == NULL);
    if (dir == TR_FROM_UP)
    {
	    MCHAIN ** mnew = MakeNewMonotonePoly(mcur, vp1, vp0);
        TraversePolygon(mcur, t->u0, t, TR_FROM_DN);
        TraversePolygon(mnew, t->d0, t, TR_FROM_UP);
    }
    else
    {
        MCHAIN ** mnew = MakeNewMonotonePoly(mcur, vp0, vp1);
        TraversePolygon(mcur, t->d0, t, TR_FROM_UP);
        TraversePolygon(mnew, t->u0, t, TR_FROM_DN);
    }
} /* NoCusp */

/* recursively visit all the trapezoids */
void TRIAGLOBS::TraversePolygon(MCHAIN **mcur, TRAP *t, TRAP *from, TraverseDir dir)
{
    MCHAIN ** mnew;
    VNODE2  * vp0, *vp1;

    if (t == NULL or t->bVisited)
		return;

    t->bVisited = true;

    /* We have much more information available here. */
    /* r: goes upwards   */
    /* l: goes downwards */

    /* Initially assume that dir = TR_FROM_DN (from the left) */
    /* Switch v0 and v1 if necessary afterwards */

    /* special cases for triangles with cusps at the opposite ends. */
    /* take care of this first */

    if      (t->u0 == NULL && t->u1 == NULL)
        TraverseCusps( mcur, t, from, t->d0, t->d1,  true);
    else if (t->d0 == NULL && t->d1 == NULL)
        TraverseCusps( mcur, t, from, t->u0, t->u1, false);
    else if (t->u0 != NULL && t->u1 != NULL)
    {
        if (t->d0 != NULL && t->d1 != NULL)
        { /* downward + upward cusps */
            vp0 = t->d1->l;
            vp1 = t->u0->r;
            if (dir == TR_FROM_DN && t->d1 == from ||
                dir == TR_FROM_UP && t->u1 == from)
            {
                mnew = MakeNewMonotonePoly( mcur, vp1, vp0);
                TraversePolygon( mcur, t->u1, t, TR_FROM_DN);
                TraversePolygon( mcur, t->d1, t, TR_FROM_UP);
                TraversePolygon( mnew, t->u0, t, TR_FROM_DN);
                TraversePolygon( mnew, t->d0, t, TR_FROM_UP);
            }
            else
            {
                mnew = MakeNewMonotonePoly( mcur, vp0, vp1);
                TraversePolygon( mcur, t->u0, t, TR_FROM_DN);
                TraversePolygon( mcur, t->d0, t, TR_FROM_UP);
                TraversePolygon( mnew, t->u1, t, TR_FROM_DN);
                TraversePolygon( mnew, t->d1, t, TR_FROM_UP);
            }
        }
        else
        /* only downward cusp */
            OnlyCusp( mcur, t, from, dir,
                    t->u0, t->u1, t->d0, t->d1, true,
                    t->lo, &t->l->next->g);

    }
    else
    {  /* no downward cusp */
        assert(t->u0 != NULL || t->u1 != NULL);
        assert(t->u1 == NULL);
        if (t->d0 != NULL && t->d1 != NULL)
        /* only upward cusp */
            OnlyCusp( mcur, t, from, dir,
                    t->d0, t->d1, t->u0, t->u1, false,
                    t->hi, &t->l->g);
        else
        { /* no cusp */
            assert(t->d1 == NULL);
            if (*t->hi == t->l->g and
                *t->lo == t->r->g)
                NoCusp(t, mcur, t->r, t->l, dir);

            else if (*t->hi == t->r->next->g and
                     *t->lo == t->l->next->g)
                NoCusp( t, mcur, t->r->next, t->l->next, dir);
            else
            { /* no split possible */
                TraversePolygon( mcur, t->u0, t, TR_FROM_DN);
                TraversePolygon( mcur, t->d0, t, TR_FROM_UP);
            }
        }
    }
} // TraversePolygon

/* Function returns true if the trapezoid lies inside the polygon */
local
bool InsidePolygon(const TRAP * t)
{
    const VNODE2 *r = t->r;

    if (t->l == NULL ||
           r == NULL ||
        !(  t->u0 == NULL && t->u1 == NULL ||
            t->d0 == NULL && t->d1 == NULL)) /* not triangle */
        return false;

    return GRID2::gt(r->next->g, r->g);
} /* InsidePolygon */

/* Main routine to get monotone polygons from the trapezoidation of
 * the polygon.
 */

void TRIAGLOBS::MonotonateTrapezoids()
{
	TRAP * tr_start = NULL;
	for (TRAP_HEAP::iterator iter = m_trap.begin(); iter != m_trap.end(); ++iter)
	{
        if (InsidePolygon(&*iter))
		{
			tr_start = &*iter;
			break;
		}
	}
	if (tr_start == NULL)
		error(err_bad_parm);
    m_mon_idx   = 1;

    if (tr_start->u0 != NULL)
        TraversePolygon(m_mon, tr_start, tr_start->u0, TR_FROM_UP);
    else
    {
        assert(tr_start->d0 != NULL);
        TraversePolygon(m_mon, tr_start, tr_start->d0, TR_FROM_DN);
    }
} // MonotonateTrapezoids

/* A greedy corner-cutting algorithm to triangulate a y-monotone
 * polygon in O(n) time.
 * Joseph O-Rourke, Computational Geometry in C.
 */
void TRIAGLOBS::TriangulateSinglePolygon(MCHAIN * posmax, MonSingleSide side)
{
    MCHAIN* mpos;
    VNODE2*	v, *endv;
	{
		MCHAIN* t = (side == TRI_R)
			? posmax->p	// RHS is a single segment
			: posmax;	// LHS is a single segment

		endv =			   t->vn;
		m_rc[0] = (t = t->n)->vn;
		m_rc[1] = (t = t->n)->vn;
		v = (mpos	 = t->n)->vn;
	}
    UINT32   ri = 1; /* reflex chain */

    while (v != endv || ri > 1)
    {
        if (ri > 0)
        { /* reflex chain is non-empty */
            const VNODE2 * vp = m_rc[ri - 1];
            const VNODE2 * vc = m_rc[ri    ];

            if (GRID2::AngleConvex(vp->g, vc->g, v->g))
            {
				TriActor(vp, vc, v);
                ri--;
            }
            else
            { /* non-convex, add v to the chain */
                m_rc[++ri] = v;
                v = (mpos = mpos->n)->vn;
            }
        }
        else
        { /* reflex-chain empty: add v to the reflex chain and
             advance it  */
            m_rc[++ri] = v;
            v = (mpos = mpos->n)->vn;
        }
    }  /* end-while */

    /* reached the bottom vertex. Add in the triangle formed */
    TriActor(m_rc[ri - 1], m_rc[ri], v);
} /* TriangulateSinglePolygon */

/* For each monotone polygon, find the ymax and ymin (to determine the
   two y-monotone chains) and pass on this monotone polygon for greedy
   triangulation.
   Take care not to triangulate duplicate monotone polygons */

void TRIAGLOBS::TriangulateMonPolys()
{
    for (UINT32 i = 0; i < m_mon_idx; i++)
    {
        MCHAIN  *posmax  = m_mon[i];
        MCHAIN  *p = posmax->n;
        VNODE2   *vfirst = posmax->vn;
        bool    processed = false;
        UINT32	vcount = 1;
        const   GRID2 *ymax, *ymin;

        ymax = ymin = &vfirst->g;
        posmax->marked = true;        
        for (vcount = 1, p = posmax->n;
             p->vn != vfirst;
             p = p->n, vcount++)
        {
            VNODE2 *v = p->vn;

            if (p->marked)
            {
                processed = true;
                break;
            }
            p->marked = true;

            if (GRID2::gt(v->g, *ymax))
                ymax = &v->g, posmax = p;

            if (GRID2::ls(v->g, *ymin))
                ymin = &v->g;
        }
        if (!processed)
        {
            if (vcount == 3)
				TriActor(p->vn, p->n->vn, p->p->vn);
            else
            {
				TriangulateSinglePolygon(posmax,
					(posmax->n->vn->g == *ymin) ? TRI_L : TRI_R);
            }
        }
    }
} /* TriangulateMonPolys */

void TRIAGLOBS::TriActor(const VNODE2 *v0, const VNODE2 *v1, const VNODE2 *v2)
{
    if (m_tria_idx >= m_area->tnum)
		error(err_bad_parm);

    PTRIA2 * t = m_area->tria + m_tria_idx++;
    t->v0 = (VNODE2 *)v0;
    t->v1 = (VNODE2 *)v1;
    t->v2 = (VNODE2 *)v2;
} // TriActor

void TRIAGLOBS::Triangulate(PAREA * p)
{
    assert(p != NULL);

	m_area = p;

	for (PLINE2 * curc = p->cntr; curc != NULL; curc = curc->next)
        m_nseg += curc->Count;

    try
    {
		if ((m_mon  = (MCHAIN **)calloc(m_nseg, sizeof(MCHAIN*))) == NULL ||
			(m_rndv = (VNODE2 **)calloc(m_nseg, sizeof(UINT32 ))) == NULL)
			error(err_no_memory);

        m_mon[0] = NULL;

		UINT32	vi = 0;
		UINT32	holes = 0;
	    for (PLINE2 * curc = p->cntr; curc != NULL; curc = curc->next, holes++)
        {
            MCHAIN*	mhead = NULL;
			VNODE2* curn = curc->head;
		    do {
                MCHAIN *m = NEW_MCHAIN();
                curn->v = m_vd.Get(true);
                if (mhead == NULL)
                    m->n = m->p = mhead = m;
                else
                    (((m->p = mhead->p)->n = m)->n = mhead)->p = m;

                VN_D(curn)->vnext[0] = curn->next;
                VN_D(curn)->mpos [0] = m;
                VN_D(curn)->nextfree = 1;

        		m_rndv[vi++] = m->vn = curn;
		    } while ((curn = curn->next) != curc->head);
            if (holes == 0)
                m_mon[0] = mhead;
        }
        if (m_nseg != vi or holes <= 0 or vi < 3)
			error(err_bad_parm);

        p->tnum = (vi - 2) + 2 * (--holes);

		free(p->tria); // free previous triangulation

        if ((p->tria = (PTRIA2*)calloc(p->tnum, sizeof(PTRIA2))) == NULL)
            error(err_no_memory);

	    ConstructTrapezoids();

        if ((m_rc = (const VNODE2 **)calloc(m_nseg + 1, sizeof(VNODE2 *))) == NULL)
            error(err_no_memory);

        MonotonateTrapezoids();
	    TriangulateMonPolys();

        if (p->tnum != m_tria_idx) // too few triangles - bad input data
			error(err_bad_parm);
    }
	catch (...)
	{
        free(p->tria), p->tria = NULL;
        p->tnum = 0;
		throw;
	}
} // Triangulate

int PAREA::Triangulate(PAREA * area)
{
	if (area == NULL)
		return 0;
    int		errc = 0;
	try
	{
	    PAREA *	pa   = area;
		do {
			TRIAGLOBS g;
			g.Triangulate(pa);
		} while ((pa = pa->f) != area);
	}
	catch (int e)
	{
		errc = e;
	}
	catch (...)
	{
		errc = err_bad_parm;
	}
    return errc;
} /* PAREA::Triangulate */

} // namespace POLYBOOLEAN

