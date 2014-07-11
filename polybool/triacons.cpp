//	triacons.cpp - trapezoid construction routines
//
//	This file is a part of PolyBoolean software library
//	(C) 1998-1999 Michael Leonov
//	Consult your license regarding permissions and restrictions
//
//	fix: 2006 Oct 2 Alexey Nikitin
//		TRIAGLOBS::AddSegment/Runtime Check #3 in VC++ 2003

#include <limits.h>
#include <time.h> /* clock() */

#include "pbtria.h"

namespace POLYBOOLEAN
{

local double log2(double x) {
	return log(x);
}

// Get log*n for given n
local
UINT32 math_logstar_n(UINT32 n)
{
    UINT32    i = 0;
    for (double v = n; v >= 1.0; v = log2(v))
        i++;

    assert(i > 0);
    return i - 1;
} /* math_logstar_n */

static
UINT32 math_N(UINT32 n, UINT32 h)
{
    double  v = n;
    for (UINT32 i = 0; i < h; i++)
        v = log2(v);

	assert(v != 0.0);
    return (UINT32)ceil(((double)n) / v);
} /* math_N */

/*  Initilialise the query structure (Q) and the trapezoid table (T)
when the first segment is added to start the trapezoidation. The
query-tree starts out with 4 trapezoids, one S-node and 2 Y-nodes
  
                       t4 - i2
     --------i1-------------------------
    		  \
    t1 - i6    i5      t2 - i7
    		    \
     ------------i3---------------------
  
                  t3 - i4
*/

QNODE *TRIAGLOBS::InitQueryStructure(VNODE2 *s)
{
    QNODE    *i1, *i2, *i3, *i4, *i5, *i6, *i7;
    TRAP     *t1, *t2, *t3, *t4;

    // i1 - root node in query structure
    (i1 = NEW_NODE())->yn.y = GRID2::ymax(&s->g, &s->next->g);
    (i3 = NEW_NODE())->yn.y = GRID2::ymin(&s->g, &s->next->g);

    (i1->yn.b = i3)->type = i1->type = QNODE::Y;
    (i3->yn.a = i5 = NEW_NODE())->type = QNODE::X;

	(i1->yn.a = i2 = NEW_NODE())->type =
    (i3->yn.b = i4 = NEW_NODE())->type =
    (i5->xn.l = i6 = NEW_NODE())->type =
    (i5->xn.r = i7 = NEW_NODE())->type = QNODE::SINK;

    i3->parent = i2->parent = i1;
    i5->parent = i4->parent = i3;
    i7->parent = i6->parent = i5;

    t1 = newtrap(); /* middle l */
    t2 = newtrap(); /* middle r */
    t3 = newtrap(); /* bottom-most */
    t4 = newtrap(); /* topmost */
    
    t1->hi = t2->hi = t4->lo = i1->yn.y;    
    t3->hi = t2->lo = t1->lo = i3->yn.y;

    t4->hi = &GRID2::PosInfinity;
    t3->lo = &GRID2::NegInfinity;

    i5->xn.vn = t1->r = t2->l = s;

    t1->u0 = t2->u0 = (t4->sink = i2)->tr = t4;
    t1->d0 = t2->d0 = (t3->sink = i4)->tr = t3;
    t4->d0 = t3->u0 = (t1->sink = i6)->tr = t1;
    t4->d1 = t3->u1 = (t2->sink = i7)->tr = t2;

    return i1;
} /* InitQueryStructure */

local
TRAP *locate_endpoint(const GRID2 & v, const GRID2 & vo, const QNODE * rptr)
{
    while (rptr->type != QNODE::SINK)
    {
        if (rptr->type == QNODE::Y)
        {
            const YNODE *yn = &rptr->yn;
            const GRID2 & y = *yn->y;

            rptr = GRID2::gt((v == y) ? vo : v, y)
				? yn->a : yn->b;
        }
        else
        {
            const XNODE *xn = &rptr->xn;
            const GRID2 & v0 = xn->vn->g;
            const GRID2 & v1 = xn->vn->next->g;

            assert(rptr->type == QNODE::X);

            if (v == v0 or v == v1)
            {
                if (v.y == vo.y)
                /* horizontal segment */
                    rptr = (vo.x < v.x) ? xn->l : xn->r;
                else
                    rptr = (GRID2::IsLeft(vo, v0, v1)) ? xn->l : xn->r;
            }
            else
                rptr = (GRID2::IsLeft(v, v0, v1)) ? xn->l : xn->r;
        }
    }
    return rptr->tr;
} /* locate_endpoint */

local
void redirect_parent(QNODE *ptnext, QNODE *chk, QNODE *q)
{
    assert(ptnext->type == QNODE::X);
    if (ptnext->xn.l == chk)
        ptnext->xn.l = q;
    else
        ptnext->xn.r = q;
} /* redirect_parent */

local
void reassign_upper(TRAP *t, const TRAP *o, TRAP *n)
{
    if (t != NULL)
    {
        if (t->u0 == o) t->u0 = n;
        if (t->u1 == o) t->u1 = n;
    }
} /* reassign_upper */


/* Thread in the segment into the existing trapezoidation. The
 * limiting trapezoids are given by tfirst and tlast (which are the
 * trapezoids containing the two endpoints of the segment. Merges all
 * possible trapezoids which flank this segment and have been recently
 * divided because of its insertion
 */

void TRIAGLOBS::MergeTrapezoids(const VNODE2 *s, TRAP *tfirst, TRAP *tlast, UpSide side)
{
    TRAP    *t, *tnext;
    bool    cond;

    t = tfirst;
    while (t != NULL && GRID2::ge(*t->lo, *tlast->lo))
    {
        if (side == S_LEFT)
            cond = (tnext = t->d0) != NULL && tnext->r == s ||
                   (tnext = t->d1) != NULL && tnext->r == s;
        else
            cond = (tnext = t->d0) != NULL && tnext->l == s ||
                   (tnext = t->d1) != NULL && tnext->l == s;

        if (cond && t->l == tnext->l
                 && t->r == tnext->r)
        { /* good neighbours, merge them */
            /* Use the upper node as the new node i.e. t */

            redirect_parent(tnext->sink->parent, tnext->sink, t->sink);

            /* Change the upper neighbours of the lower trapezoids */

            reassign_upper(t->d0 = tnext->d0, tnext, t);
            reassign_upper(t->d1 = tnext->d1, tnext, t);

            t->lo = tnext->lo;
            puttrap(tnext); /* free tnext */
        }
        else  /* not good neighbours or !cond */
            t = tnext;
    }  /* while */
} /* MergeTrapezoids */

local
bool pt_on_vn(const VNODE2 *s, const GRID2 & v)
{
    return s != NULL and (
        v == s->g or
        v == s->next->g);
} /* pt_on_vn */

/* Returns true if v0 is already inserted into the query tree. */
local
bool check_divide(const GRID2 & v0, const TRAP *t)
{
    return  *t->hi == v0 or
            *t->lo == v0 or
            pt_on_vn(t->l, v0) or
            pt_on_vn(t->r, v0);
} /* check_divide */

/*  insert vertex into trapezoidation as Y-node,
    create corresponding trapezoid */
TRAP *TRIAGLOBS::InsertVertex(bool fst, const QNODE *root,
                    const GRID2 * v0, const GRID2 * v1)
{
    TRAP *tu = locate_endpoint(*v0, *v1, root);

    if (!check_divide(*v0, tu))
    {   /* insert v0 in the tree
        tu is the containing trapezoid
        tl is the new lower trapezoid */
        QNODE   *i1, *i2, *sk;
        TRAP    *tl = newtrap();

        *tl = *tu;
        tu->lo = tl->hi = v0;

        ((tu->d0 = tl)->u0 = tu)->d1 = tl->u1 = NULL;

        reassign_upper(tl->d0, tu, tl);
        reassign_upper(tl->d1, tu, tl);

        /* Now update the query structure and obtain the sinks for the
        two trapezoids:
        i1 - upper trapezoid sink
        i2 - lower trapezoid sink */

        (sk = tu->sink)->type = QNODE::Y, sk->yn.y = v0;
        (sk->yn.a = i1 = NEW_NODE())->type =
        (sk->yn.b = i2 = NEW_NODE())->type = QNODE::SINK;

        ((i1->tr = tu)->sink = i1)->parent =
        ((i2->tr = tl)->sink = i2)->parent = sk;

        if (fst) tu = tl; /* start trapezoid */
    }
    return tu;
} /* InsertVertex */

TRAP *TRIAGLOBS::SplitCurTrap(TRAP *t, const VNODE2 *s)
{
    QNODE   *i1 = NEW_NODE();
	QNODE   *i2 = NEW_NODE();
	QNODE   *sk = t->sink;
    TRAP    *tn = newtrap();

    sk->type = QNODE::X;
    sk->xn.vn = s;

 /* i1 - left  trapezoid sink
    i2 - right trapezoid sink */
    (sk->xn.l = i1)->parent = (sk->xn.r = i2)->parent = sk;

    i1->type = i2->type = QNODE::SINK;
    i1->tr = t;
    i2->tr = tn;

    *tn = *t;
    t ->sink = i1;
    tn->sink = i2;

    return tn;
} /* SplitCurTrap */

/******************* upper trapezoid connection routines ************************/

static
void cont_chain(TRAP *t, TRAP *tn, TRAP **usave, TRIAGLOBS::UpSide uside)
{
    if (*usave != NULL)
    { /* three upper neighbours */
        if (uside == TRIAGLOBS::S_LEFT)
        {
            tn->u0 = t->u1, t->u1 = NULL;
            tn->u1 = *usave;
            t->u0->d0 = t;
            tn->u0->d0 = tn->u1->d0 = tn;
        }
        else
        {
            tn->u1 = NULL;
            tn->u0 = t->u1, t->u1 = t->u0, t->u0 = *usave;
            t->u0->d0 = t->u1->d0 = t;
            tn->u0->d0 = tn;
        }
        *usave = NULL;
    }
    else
    {
        tn->u0 = t->u1, t->u1 = tn->u1 = NULL;
        tn->u0->d0 = tn;
    }
} /* cont_chain */

static
void fresh_segm(TRAP *t, TRAP *tn, const GRID2 & v0)
{
    TRAP *u = t->u0, **ud = NULL;

    assert(t->u1 == NULL);
    if      (u->d0 == t)
        ud = &u->d0;
    else if (u->d1 == t)
        ud = &u->d1;

    assert(ud != NULL);
    
    if      (pt_on_vn(t->l, v0))
        *ud = tn, t->u0 = NULL;
    else if (pt_on_vn(t->r, v0))
        *ud = t, tn->u0 = NULL;
    else
    {
        assert(ud == &u->d0);
        u->d1 = tn;
    }
} /* fresh_segm */

/*************************** lower trapezoid connection routines ***************/

static
TRAP *split_one_trap(TRAP *t, TRAP *tn, TRAP **usave, TRIAGLOBS::UpSide *uside)
{
    TRAP *d = t->d0;
    if (d->u0 != NULL && d->u1 != NULL)
    {
        if (d->u0 == t)
        {
            *usave = t->d0->u1;
            *uside = TRIAGLOBS::S_LEFT;
        }
        else
        {
            *usave = t->d0->u0;
            *uside = TRIAGLOBS::S_RIGHT;
        }
    }
    d->u0 = t;
    d->u1 = tn;
    return d;
} /* split_one_trap */

static
TRAP *split_two_trap(TRAP *t, TRAP *tn, const GRID2 & v0, const GRID2 & v1)
{
    TRAP    *dl = t->d0;
    TRAP    *dr = t->d1;

    assert(dl->u1 == NULL && dr->u1 == NULL);
    assert(dl->u0 == t);
    assert(dr->u0 == t);

    if ((INT64)(t->lo->y - v0.y) * (v1.x - v0.x) >
		(INT64)(t->lo->x - v0.x) * (v1.y - v0.y))
    { /* intersecting dl */
        dl->u1 = dr->u0 = tn;
        t->d1  = NULL;
        return dl;
    }
    else
    { /* intersecting dr */
        (dr->u1 = tn)->d0 = dr;
        tn->d1 = NULL;
        return dr;
    }
} /* split_two_trap */

static
void last_segm(TRAP *t, TRAP *tn, const GRID2 & v1)
{
    TRAP *d = t->d0, **du = NULL;

    assert(t->d1 == NULL);
    if      (d->u0 == t)
        du = &d->u0;
    else if (d->u1 == t)
        du = &d->u1;

    assert(du != NULL);
    
    if      (pt_on_vn(t->l, v1))
        *du = tn, t->d0 = NULL;
    else if (pt_on_vn(t->r, v1))
        *du = t, tn->d0 = NULL;
    else
    {
        assert(du == &d->u0);
        d->u1 = tn;
    }
} /* last_segm */

void TRIAGLOBS::AddSegment(VNODE2 *s)
{
    TRAP  *	tfirst, *tlast, *tfirstr, *tlastr;
    TRAP  *	t, *tn, *next;
    TRAP  *	usave = NULL;
	UpSide  uside = TRIAGLOBS::S_LEFT;

    const GRID2 * v0 = &s->g;
	const GRID2 * v1 = &s->next->g;

    const QNODE * root0 = VN_D(s)->root_n;
    const QNODE * root1 = VN_D(s->next)->root_p;

	if (*v1 == *v0)
		error(err_bad_parm);

    if (GRID2::gt(*v1, *v0))
    {  // Get higher vertex in v0
        const GRID2 * tpt = v0; v0 = v1, v1 = tpt;
        const QNODE * tmp = root0; root0 = root1, root1 = tmp;
    }

    tfirst = InsertVertex(true,  root0, v0, v1);
    tlast  = InsertVertex(false, root1, v1, v0);

    tfirstr = tlastr = NULL;

    for (t = tfirst; t != NULL; t = next)
    {
        if (GRID2::ls(*t->lo, *v1))
			error(err_bad_parm);

        tn = SplitCurTrap(t, s);

        /* connect with upper trapezoid(s) */
        if (t != tfirst)
        {
            if (t->u0 == NULL or t->u1 == NULL)
				error(err_bad_parm);
            cont_chain(t, tn, &usave, uside);
        }
        else
        {
            tfirstr = tn;
            if (t->u0 != NULL)
            {
                if (t->u1 != NULL)
                {
                    assert(t->u0->d1 == NULL && t->u1->d1 == NULL);
                    (tn->u0 = t->u1)->d0 = tn;
                    t->u1 = tn->u1 = NULL;
                }
                else
                    fresh_segm(t, tn, *v0);
            }
        }

        /* prepare lower trapezoid(s) */
        if (!(*t->lo == *v1))
        {
            if (t->d0 == NULL)
				error(err_bad_parm);
            if (t->d1 != NULL)
                next = split_two_trap(t, tn, *v0, *v1);
            else
                next = split_one_trap(t, tn, &usave, &uside);
        }
        else
        {
			if (tlast != t)
				error(err_bad_parm);
            /*tlast   = t;*/
            tlastr  = tn;
            if (t->d0 != NULL)
            {
                if (t->d1 != NULL)
                {
                    assert(t->d0->u1 == NULL && t->d1->u1 == NULL);
                    ((t->d0->u0 = t)->d1->u0 = tn)->d0 = t->d1;
                    t->d1 = tn->d1 = t->d0->u1 = t->d1->u1 = NULL;
                }
                else
                    last_segm(t, tn, *v1);
            }
            next = NULL;
        }
        t->r = tn->l = s;
    }
    assert(usave == NULL);
	if (tfirstr == NULL or tlastr == NULL)
		error(err_bad_parm);

    MergeTrapezoids(s, tfirst , tlast , S_LEFT);
    MergeTrapezoids(s, tfirstr, tlastr, S_RIGHT);
} // AddSegment


local
void find_new_roots(VNODE2 *s)
{
    VN_D(s)->root_n = locate_endpoint(s->g, s->next->g,
    VN_D(s)->root_n)->sink;
    VN_D(s)->root_p = locate_endpoint(s->g, s->prev->g,
    VN_D(s)->root_p)->sink;
} // find_new_roots

/* Main routine to perform trapezoidation */
void TRIAGLOBS::ConstructTrapezoids()
{
    QNODE *	root;
    UINT32	i, n = m_nseg;

    srand(clock());
    assert(n >= 3);
    m_choose_idx = 0;

    for (i = 0; i < n; i++)
	{
		UINT32    m = ((UINT32)(rand() * 0xffff + rand())) % n;
        VNODE2   *t;

		t = m_rndv[i], m_rndv[i] = m_rndv[m], m_rndv[m] = t;
	}

    root = InitQueryStructure(choose_segment());

    for (i = 0; i < n; i++)
        VN_D(m_rndv[i])->root_n =
        VN_D(m_rndv[i])->root_p = root;

    UINT32    prevN, curN, h;
    UINT32    logstar = math_logstar_n(n);

    for (h = 1, prevN = math_N(n, 0);
         h <= logstar;
         h++  , prevN = curN)
    {
        curN = math_N(n, h);

        for (i = prevN + 1; i <= curN; i++)
            AddSegment(choose_segment());
        
        for (i = m_choose_idx; i < n; i++)
            find_new_roots(m_rndv[i]);
    }
        for (i = prevN + 1; i <=    n; i++)
            AddSegment(choose_segment());
} /* ConstructTrapezoids */

} // namespace POLYBOOLEAN

