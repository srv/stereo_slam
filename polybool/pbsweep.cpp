//	pbsweep.cpp - plane sweep on integer grid
//
//	This file is a part of PolyBoolean software library
//	(C) 1998-1999 Michael Leonov
//	Consult your license regarding permissions and restrictions
//

#include "pbarea.h"

namespace POLYBOOLEAN
{

INT32 BOCTX::EVENT::Compare(const EVENT &a, const EVENT &b)
{
	assert(INT20_MIN <= a.x and a.x <= INT20_MAX);
	assert(INT20_MIN <= a.y and a.y <= INT20_MAX);
	assert(INT20_MIN <= b.x and b.x <= INT20_MAX);
	assert(INT20_MIN <= b.y and b.y <= INT20_MAX);

	if (a.x != b.x)
		return a.x - b.x;

	TYPE aType = a.GetType();
	TYPE bType = b.GetType();

	int asx, asy, bsx, bsy;
	if (aType == X)
		asx = a.GetSignX(), asy = a.GetSignY();
	else
		asx = asy = 0;
	if (bType == X)
		bsx = b.GetSignX(), bsy = b.GetSignY();
	else
		bsx = bsy = 0;

	if (asx != bsx)
		return asx - bsx;

	if (a.y != b.y)
		return a.y - b.y;

	if (asy != bsy)
		return asy - bsy;

	if (aType != bType)
		return aType - bType;

	return a.id - b.id;
} // EVENT::Compare

//////////////// basic geometric primitives //////////////////

// returns if s0 has a greater slope
local
bool CheckSlope(const SEGM2 & s0, const SEGM2 & s1)
{
	INT64 det =
		(INT64)(s0.r->g.y - s0.l->g.y) * (s1.r->g.x - s1.l->g.x) -
		(INT64)(s0.r->g.x - s0.l->g.x) * (s1.r->g.y - s1.l->g.y);
	return (det > 0);
} // CheckSlope

// returns if a is above b in main active list
// sweep line position is a.l->g.x
local
bool IsAbove(const SEGM2 & a, const SEGM2 & b)
{
	INT32 x0 = b.l->g.x;

	assert(a.l->g.x >= x0); // sweep line shouldn't be to the left of b

	if (INT64 yden = b.r->g.x - x0) // b is not vertical
	{
		assert(yden > 0);

		INT32 y0 = b.l->g.y;
		INT64 ynom = yden * y0 + (INT64)(b.r->g.y - y0) * (a.l->g.x - x0);
		INT64 sign = yden * a.l->g.y - ynom;

		if (sign > 0)
			return true;
		if (sign < 0)
			return false;
		// break tie
		return CheckSlope(a, b);
	}
	// now b is vertical, compare a with b's upper point
	return (a.l->g.y >= b.r->g.y);
} // IsAbove

local
void RoundTo(INT64 a, INT64 b, INT32 * nDiv, int * nSgn)
{
	INT64 m = a % b;
	if (a >= 0)
	{
		if (2 * m >= b)
			*nDiv = (INT32)(a / b) + 1, *nSgn = -1;
		else
			*nDiv = (INT32)(a / b) + 0, *nSgn = (m == 0) ? 0 : +1;
	}
	else
	{
		if (-2 * m > b)
			*nDiv = (INT32)(a / b) - 1, *nSgn = +1;
		else
			*nDiv = (INT32)(a / b) + 0, *nSgn = (m == 0) ? 0 : -1;
	}
} // RoundTo


// exactly compares nom/den with x
// return value is:
//   0	if nom/den == x
// < 0	if nom/den <  x
// > 0	if nom/den >  x
local
int SgnCmp(INT64 nom, INT64 den, INT32 x)
{
	assert(den > 0);
	INT64 den_x = den * x;
	if (nom < den_x)
		return -1;
	if (nom > den_x)
		return +1;
	return 0;
} // SgnCmp

// returns if point (xnom/xden, ynom/yden) is
// lexicographically less than (x,y)
local
bool LexLs(INT64 xnom, INT64 xden, INT64 ynom, INT64 yden,
			 INT32 x, INT32 y)
{
	int cmp = SgnCmp(xnom, xden, x);
	if (cmp > 0)
		return false;
	if (cmp < 0)
		return true;
	return (SgnCmp(ynom, yden, y) < 0);
} // LexLs

// returns if point (xnom/xden, ynom/yden) is
// lexicographically greater than (x,y)
local
bool LexGt(INT64 xnom, INT64 xden, INT64 ynom, INT64 yden,
			 INT32 x, INT32 y)
{
	int cmp = SgnCmp(xnom, xden, x);
	if (cmp > 0)
		return true;
	if (cmp < 0)
		return false;
	return (SgnCmp(ynom, yden, y) > 0);
} // LexGt

#ifndef NDEBUG
local
bool IntLess(INT32 xa, INT32 ya, INT32 xb, INT32 yb)
{
	return (xa < xb or xa == xb and ya < yb);
} // IntLess
#endif // NDEBUG

// calculates the intersection point of lines (a,b) & (c,d)
// assuming they are not parallel,
// the intersection point coordinates are represented as rational numbers
// with denominators > 0
// precondition is a < b and c < d
local
bool SegmIsect(INT32 xa, INT32 ya, INT32 xb, INT32 yb,
			   INT32 xc, INT32 yc, INT32 xd, INT32 yd,
			   INT64 * xnom, INT64 * xden,
			   INT64 * ynom, INT64 * yden)
{
	assert(IntLess(xa, ya, xb, yb));
	assert(IntLess(xc, yc, xd, yd));

    INT64 xcd = xc - xd;
    INT64 ycd = yc - yd;

    INT64 r = xcd * (yc - ya) - ycd * (xc - xa);

    INT32 xab = xa - xb;
    INT32 yab = ya - yb;
    INT64 d = ycd * xab - xcd * yab;

	assert(d != 0);

	if (d > 0)
	{
		*xden = *yden = d;
		*xnom = d * xa - r * xab;
		*ynom = d * ya - r * yab;
	}
	else
	{
		*xden = *yden = d = -d;
		*xnom = d * xa + r * xab;
		*ynom = d * ya + r * yab;
	}
	return	LexLs(*xnom, *xden, *ynom, *yden, xb, yb) and
			LexLs(*xnom, *xden, *ynom, *yden, xd, yd) and
			LexGt(*xnom, *xden, *ynom, *yden, xa, ya) and
			LexGt(*xnom, *xden, *ynom, *yden, xc, yc);
} // SegmIsect

local
bool CalcIsect(	const SEGM2 & s0, const SEGM2 & s1,
				INT32 * xDiv, int * xSgn,
				INT32 * yDiv, int * ySgn)
{
	assert(s0.n == &s1); // s0 is the lower neghbour of s1
	if (not CheckSlope(s0, s1))
		return false;

	INT64 xnom, xden, ynom, yden;
	if (not SegmIsect(
		s0.l->g.x, s0.l->g.y, s0.r->g.x, s0.r->g.y,
		s1.l->g.x, s1.l->g.y, s1.r->g.x, s1.r->g.y,
		&xnom, &xden, &ynom, &yden))
		return false;

	RoundTo(xnom, xden, xDiv, xSgn);
	RoundTo(ynom, yden, yDiv, ySgn);
	return true;
} // CalcIsect

local
void Isect2X(const SEGM2 & b, INT64 X2, INT32 * yDiv)
{
	INT32 x0 = b.l->g.x;
	INT32 y0 = b.l->g.y;
	INT32 x1 = b.r->g.x;
	INT32 y1 = b.r->g.y;

	INT64 yden = (INT64)(x1 - x0) * 2;
	assert(yden > 0);

	INT64 ynom = yden * y0 + (X2 - 2 * x0) * (y1 - y0);

	int ySgn;
	RoundTo(ynom, yden, yDiv, &ySgn);
} // IsectX

local
void ObtainSegmY(const SEGM2 & s, INT32 X, INT32 * lyDiv, INT32 * ryDiv)
{
	assert(s.l->g.x <= X);
	assert(s.r->g.x >= X);

	if (s.l->g.x == X)
		*lyDiv = s.l->g.y;
	else
		Isect2X(s, (INT64)X * 2 - 1, lyDiv);

	if (s.r->g.x == X)
		*ryDiv = s.r->g.y;
	else
		Isect2X(s, (INT64)X * 2 + 1, ryDiv);
} // ObtainSegmY

//////////////////// class BOCTX implementation /////////////////////

BOCTX::BOCTX(INS_PROC InsertProc, void * InsertParm)
{
	m_nId = 0;
	m_InsertProc = InsertProc;
	m_InsertParm = InsertParm;
} // BOCTX::BOCTX

void BOCTX::InsMainList(SEGM2 * s)
{
	SEGM_LIST::iterator segm = m_S.begin();
	while (segm != m_S.end() and IsAbove(*s, *segm))
		++segm;
	// insert s below segm
	m_S.insert(segm, *s);
} // BOCTX::InsMainList

void BOCTX::AddEvent(EVENTLIST * list, const EVENT & e)
{
	list->push_back(e);
} // BOCTX::AddEvent

void BOCTX::CollectEvents(SEGM2 * aSegms, UINT32 nSegms)
{
	for (UINT32 i = 0; i < nSegms; i++)
	{
		SEGM2 * segm = aSegms + i;
		EVENT s, e;

		s.SetType(EVENT::S);
		e.SetType(EVENT::E);

		s.x = segm->l->g.x, s.y = segm->l->g.y;
		e.x = segm->r->g.x, e.y = segm->r->g.y;

		s.id = m_nId++;
		e.id = m_nId++;

		s.s.s = e.e.s = segm;

		m_E.push(s);
		m_E.push(e);
	}
} /* CollectEvents */

void BOCTX::CheckForCross(SEGM2 * sp, SEGM2 * sn)
{
	assert(sp->n == sn);
	if (sn != sp and sn != &*m_S.end() and sp != &*m_S.end())
	{
		INT32	xDiv, yDiv;
		int		xSgn, ySgn;

		if (CalcIsect(*sp, *sn, &xDiv, &xSgn, &yDiv, &ySgn))
		{
			EVENT e;

			e.SetType(EVENT::X);

			e.x = xDiv, e.SetSignX(xSgn);
			e.y = yDiv, e.SetSignY(ySgn);
			e.c.s0 = sp;
			e.c.s1 = sn;
			e.id = m_nId++;

			m_E.push(e);
		}
	}
} // BOCTX::CheckForCross

void BOCTX::HandleEvent(EVENT & event)
{
	switch (event.GetType())
	{
	case EVENT::E:
	{
		SEGM2 * sn = event.e.s->n;
		SEGM2 * sp = event.e.s->p;
		// untie segment
		m_S.erase(SEGM_LIST::iterator(event.e.s));
		CheckForCross(sp, sn);
	}	break;

	case EVENT::S:
		InsMainList(event.s.s);
		CheckForCross(event.s.s, event.s.s->n);
		CheckForCross(event.s.s->p, event.s.s);
		break;

	case EVENT::X:
	{
		if	(event.c.s0->n != event.c.s1)
			return; // ignore crossing event in case of non-adjacent segments

		SEGM_LIST::iterator s0(event.c.s0), s1(event.c.s1);

		// swap segments
		assert(s0->n == &*s1 and s1->p == &*s0);
		// untie s0
		m_S.erase(s1);
		m_S.insert(s0, *s1);

		CheckForCross(s1->p, &*s1);
		CheckForCross(&*s0, s0->n);
	}	break;
	}
} // BOCTX::HandleEvent

void BOCTX::Sweep(SEGM2 * aSegms, UINT32 nSegms)
{
	CollectEvents(aSegms, nSegms);

	while (not m_E.empty())
	{
		// store the current state of main active list
		SAVE_LIST save_list;
		save_list.reserve(32);
		
		for (SEGM_LIST::iterator segm = m_S.begin(); segm != m_S.end(); ++segm)
			save_list.push_back(&*segm);

		// do Pass 1
		EVENT e = m_E.top();
		m_E.pop();
		EVENTLIST elist;
		elist.reserve(8);

		assert(INT20_MIN <= e.x and e.x <= INT20_MAX);

			AddEvent(&elist, e);
			HandleEvent(e);

		while (not m_E.empty() and m_E.top().x == e.x)
		{
			e = m_E.top();
			m_E.pop();
			AddEvent(&elist, e);
			HandleEvent(e);
		}

		// do Pass 2
		Pass2(&elist, &save_list);
	}
} // Sweep

void BOCTX::InsNewNode(SEGM2 * s, TSEL * tsel, INT32 X)
{
	INT32 Y = tsel->y;
	VNODE2 * vn;
	if		(s->l->g.x == X and s->l->g.y == Y)
		vn = s->l;
	else if	(s->r->g.x == X and s->r->g.y == Y)
		vn = s->r;
	else
	{
		VNODE2 * after = (s->m_bRight) ? s->r->prev : s->r;
		assert(after->g.x != X or after->g.y != Y);

		vn = (VNODE2 *)calloc(1, sizeof(VNODE2));
		if (vn == NULL)
			error(err_no_memory);

		vn->Flags = s->l->Flags;
		vn->g.x = X;
		vn->g.y = Y;
		vn->Incl(after);
	}
	m_InsertProc(&tsel->list, vn, m_InsertParm);
} // BOCTX::InsNewNode

// finds tolerance square intersections with segm
// and inserts them into it
void BOCTX::Intercept(LIST_TSEL * tsel, SEGM2 * segm, INT32 cx)
{
	assert(not tsel->empty());
	INT32 ly, ry;

	ObtainSegmY(*segm, cx, &ly, &ry);

	if		(ly > ry)	// dn
	{
		LIST_TSEL::reverse_iterator ts = tsel->rbegin();
		for (;;)
		{
			if (ts->y <= ly)
				break;
			if (++ts == tsel->rend())
				return;
		} 
		while (ts->y >= ry and ts->y <= ly)
		{
			InsNewNode(segm, &*ts, cx);
			if (++ts == tsel->rend())
				return;
		}
	}
	else if (ly < ry)	// up
	{
		LIST_TSEL::iterator ts = tsel->begin();
		for (;;)
		{
			if (ts->y >= ly)
				break;
			if (++ts == tsel->end())
				return;
		}
		while (ts->y >= ly and ts->y <= ry)
		{
			InsNewNode(segm, &*ts, cx);
			if (++ts == tsel->end())
				return;
		}
	}
	else				// horz
	{
		LIST_TSEL::iterator ts = tsel->begin();
		for (;;)
		{
			if (ts->y >= ly)
				break;
			if (++ts == tsel->end())
				return;
		}
		if (ts->y == ly)
			InsNewNode(segm, &*ts, cx);
	}
} // BOCTX::Intercept

void BOCTX::Pass2(EVENTLIST * elist, SAVE_LIST * save_list)
{
	assert(not elist->empty());

	LIST_TSEL tsel;

	// create tolerance squares for each event
	for (EVENTLIST::const_iterator e = elist->begin(); e != elist->end(); ++e)
	{
		TSEL e0 = { e->y, NULL };
		tsel.push_back(e0);
	}
	
	tsel.sort();
	tsel.unique();

	// simply intersect tolerance squares with each active segment
	INT32 elistx = elist->front().x;
	for (SAVE_LIST::iterator segm = save_list->begin(); segm != save_list->end(); ++segm)
		Intercept(&tsel, *segm, elistx);

	for (EVENTLIST::iterator e = elist->begin(); e != elist->end(); ++e)
	{
		if (e->GetType() == EVENT::S)
			Intercept(&tsel, e->s.s, elistx);
	}
} // BOCTX::Pass2

} // namespace POLYBOOLEAN

