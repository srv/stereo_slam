//	polybool.cpp - polygon Boolean operations
//
//	This file is a part of PolyBoolean software library
//	(C) 1998-1999 Michael Leonov
//	Consult your license regarding permissions and restrictions
//

#include "pbarea.h"
#include "ObjHeap.h"
#include "Sort.h"

namespace POLYBOOLEAN
{

local
int GetQuadrant(INT32 dx, INT32 dy)
{
	assert(dx != 0 or dy != 0);
	if (dx >  0 and dy >= 0)	return 0;
	if (dx <= 0 and dy >  0)	return 1;
	if (dx <  0 and dy <= 0)	return 2;
assert (dx >= 0 and dy <  0);	return 3;
} // GetQuadrant

struct VLINK
{
    VLINK *	n, * p;	// neighbours
    VNODE2*	vn;		// parent vnode
	INT32	dx;		// dx and dy determine descriptor angle
	INT32	dy;
	UINT32	m_flags;	
    VNODE2*	shared;	// shared edge

	enum LFLAGS
	{
		L_FIN		= 0x00000001,
		L_FVALID	= 0x00000002,
		L_FFIRST	= 0x00000004
	};

	void SetIn(bool bIn)
	{
		if (bIn)
			m_flags |= L_FIN;
		else
			m_flags &= ~L_FIN;
	} // SetIn

	bool IsIn() const
	{
		return (m_flags & L_FIN);
	} // IsIn

	void SetValid(bool bValid)
	{
		if (bValid)
			m_flags |= L_FVALID;
		else
			m_flags &= ~L_FVALID;
	} // SetValid

	bool IsValid() const
	{
		return (m_flags & L_FVALID);
	} // IsValid

	void SetFirst(bool bFirst)
	{
		if (bFirst)
			m_flags |= L_FFIRST;
		else
			m_flags &= ~L_FFIRST;
	} // SetFirst

	bool IsFirst() const
	{
		return (m_flags & L_FFIRST);
	} // IsFirst

	VLINK *& nxt() { return n; }

	static int Compare(const VLINK & a, const VLINK & b)
	{
		int aq = GetQuadrant(a.dx, a.dy);
		int bq = GetQuadrant(b.dx, b.dy);
		if (aq != bq)
			return aq - bq;

		INT64 nSign =	(INT64)a.dy * b.dx -
						(INT64)a.dx * b.dy;

		if (nSign < 0)	return -1;
		if (nSign > 0)	return +1;
		return a.IsFirst() - b.IsFirst();
	} // Compare
}; // struct VLINK

typedef ObjStorageClass<VLINK, 16, err_no_memory>	LNK_HEAP;

struct POLYBOOL
{
    LNK_HEAP	m_LinkHeap;
}; // class POLYBOOL

enum ELABEL
{
    E_INSIDE,
    E_OUTSIDE,
    E_SHARED1,
    E_SHARED2,
    E_SHARED3,
    E_UNKNOWN
};

const UINT32 E_MASK = 0x0007; // edge label mask
const UINT32 E_MARK = 0x0008; // vertex is already included into result
const UINT32 E_FST  = 0x0010; // vertex belongs to 1st PAREA

enum PLABEL
{
    P_OUTSIDE,
    P_INSIDE,
    P_UNKNOWN,
    P_ISECTED
};

const UINT32 P_MASK = 0x0003; // pline label mask

local
void SetVnodeLabel(VNODE2 * vn, ELABEL nLabel)
{
	vn->Flags = (vn->Flags & ~E_MASK) | (nLabel & E_MASK);
} // SetVnodeLabel

local
ELABEL GetVnodeLabel(const VNODE2 * vn)
{
	return (ELABEL)(vn->Flags & E_MASK);
} // GetVnodeLabel

local
void SetPlineLabel(PLINE2 * pline, PLABEL nLabel)
{
	pline->Flags = (pline->Flags & ~P_MASK) | (nLabel & P_MASK);
} // SetPlineLabel

local
PLABEL GetPlineLabel(const PLINE2 * pline)
{
	return (PLABEL)(pline->Flags & P_MASK);
} // GetPlineLabel

local
bool IsFirst(const VNODE2 * vn)
{
	return (vn->Flags & E_FST);
} // IsFirst

local
bool IsMarked(const VNODE2 * vn)
{
	return (vn->Flags & E_MARK);
} // IsMarked

local
void SetMarked(VNODE2 * vn)
{
	vn->Flags |= E_MARK;
} // SetMarked

local
void SetFirst(VNODE2 * vn, bool bFirst)
{
	if (bFirst)
		vn->Flags |= E_FST;
	else
		vn->Flags &= ~E_FST;
} // SetFirst

#define SETBITS(n,mask,s) (((n)->Flags & ~(mask)) | ((s) & (mask)))

//////////////////////////////////////////////////////////

local
bool HasShared3(const VLINK * a)
{
	if (a == NULL or a->n == NULL)
		return false;
	VLINK * b = a->n;

	return (a->dx == b->dx and a->dy == b->dy and a->IsFirst() == b->IsFirst());
} // HasShared3

local
void LnkUntie(const VLINK * l)
{
	if (l->IsIn())
		l->vn->lnk.i = NULL;
	else
		l->vn->lnk.o = NULL;
} // LnkUntie

local
void MarkAsDone(const VLINK * a)
{
	if (a->IsIn())
	{
		SetVnodeLabel(a->vn->prev, E_SHARED3);
		a->vn->lnk.i = NULL;
	}
	else
	{
		SetVnodeLabel(a->vn, E_SHARED3);
		a->vn->lnk.o = NULL;
	}
} // MarkAsDone

static
void SortDesc(PAREA * area)
{
	PAREA * pa = area;
	do {
		for (PLINE2 * pline = pa->cntr; pline != NULL; pline = pline->next)
		{
			VNODE2 * vn = pline->head;
			do {
				if (vn->lnk.i == NULL and vn->lnk.o == NULL)
					continue;

				VLINK * head;
				if		(vn->lnk.i == NULL)
					head = vn->lnk.o;
				else if (vn->lnk.o == NULL)
					head = vn->lnk.i;
				else if (vn->lnk.o->n == vn->lnk.i and vn->lnk.o->p == vn->lnk.i)
				{	// resolve trivial intersections at endpoints
					vn->lnk.o = vn->lnk.i = NULL;
					continue;
				}
				else
					head = vn->lnk.i;

				if (head->IsValid())
				{
					SetPlineLabel(pline, P_ISECTED);
					continue;
				}
				VLINK * l = head;
				do {
					VNODE2 * vnext = (l->IsIn()) ? l->vn->prev : l->vn->next;
					l->dx = vnext->g.x - vn->g.x;
					l->dy = vnext->g.y - vn->g.y;
					l->SetFirst(IsFirst(l->vn));
				} while ((l = l->n) != head);

				(l->p)->n = NULL;
				MergeSort<VLINK>(&l);
				
				// now, untie self shared edges
				{
					VLINK ** pp = &l, * c;
					while ((c = *pp) != NULL)
					{
						if (not HasShared3(c))
							pp = &c->n;
						else
						{
							SetPlineLabel(pline, P_ISECTED);
							MarkAsDone(c);
							MarkAsDone(c->n);
							*pp = c->n->n;
						}
					}
				}

				if (l == NULL)
					continue;
				assert(l->n != NULL);
				if (l->n->n == NULL)
				{	// only trivial intersection remained
					LnkUntie(l);
					LnkUntie(l->n);
					continue;
				}
				// now we have only true intersections
				// make valid doubly linked list

				SetPlineLabel(pline, P_ISECTED);
				VLINK * s = l, * q;
				while ((q = s->n) != NULL)
				{
					s->SetValid(true);
					q->p = s, s = s->n;
				}
				(l->p = s)->n = l;
			} while ((vn = vn->next) != pline->head);
		}
	} while ((pa = pa->f) != area);
} // SortDesc

///////////////////// Routines for making labels /////////////////

local
bool DoShared(VLINK * lnk, VLINK * chk)
{
	if (lnk->dx != chk->dx or lnk->dy != chk->dy)
		return false;

	// we have a pair of shared edges
	ELABEL	nLabel;
	VNODE2*	vn0;
	VNODE2*	vn1;
	if (chk->IsIn() == lnk->IsIn())
	{
		nLabel = E_SHARED1;
		if (chk->IsIn())
			vn0 = chk->vn->prev,vn1 = lnk->vn->prev;
		else
			vn0 = chk->vn,		vn1 = lnk->vn;
	}
	else
	{
		nLabel = E_SHARED2;
		if (chk->IsIn())
			vn0 = chk->vn->prev,vn1 = lnk->vn;
		else
			vn0 = chk->vn,		vn1 = lnk->vn->prev;
	}
	SetVnodeLabel(vn0, nLabel);
	SetVnodeLabel(vn1, nLabel);
	vn0->lnk.o->shared = vn1;
	vn1->lnk.o->shared = vn0;
	return true;
} // DoShared

local
bool DoLabel(VNODE2 * vn, VLINK * lnk)
{
	VLINK * n = NULL;
	for (VLINK * vl = lnk->n; vl != lnk; vl = vl->n)
	{
		if (vl->IsFirst() != lnk->IsFirst())
		{
			n = vl;
			break;
		}
	}
	if (n == NULL)
		return false;
	VLINK * p = NULL;
	for (VLINK * vl = lnk->p; vl != lnk; vl = vl->p)
	{
		if (vl->IsFirst() != lnk->IsFirst())
		{
			p = vl;
			break;
		}
	}
	assert(p != NULL and n != p);
	if (DoShared(lnk, n) or DoShared(lnk, p))
		return true;

	// check if lnk lies inside (p, n)
	bool bInside = n->IsIn() and not p->IsIn();
	SetVnodeLabel(vn, (bInside) ? E_INSIDE : E_OUTSIDE);
	return true;
} // DoLabel

static
void LabelIsected(PLINE2 * pline, PAREA * other)
{
    VNODE2 * vn = pline->head;
    do {
		ELABEL nLabel = GetVnodeLabel(vn);
		if (nLabel != E_UNKNOWN)
			continue;
		VLINK * lnk = NULL;
		if		(vn->lnk.o != NULL)
			lnk = vn->lnk.o;
		else if (vn->next->lnk.i != NULL)
			lnk = vn->next->lnk.i;

		if (lnk != NULL and DoLabel(vn, lnk))
			continue;

		ELABEL nPrev = GetVnodeLabel(vn->prev);
		if (nPrev != E_UNKNOWN and nPrev != E_SHARED3)
			SetVnodeLabel(vn, nPrev);
		else
            SetVnodeLabel(vn, GridInParea(&vn->g, other) ? E_INSIDE : E_OUTSIDE);
    } while ((vn = vn->next) != pline->head);
} // LabelIsected

local
void LabelPline(PLINE2 * pline, PAREA * other)
{
    if (GetPlineLabel(pline) == P_ISECTED)
        LabelIsected(pline, other);
    else
        pline->Flags = SETBITS(pline, P_MASK,
			PlineInParea(pline, other) ? P_INSIDE : P_OUTSIDE);
} // LabelPline

local
void LabelParea(PAREA * area, PAREA * other)
{
    PAREA * pa = area;
    do {
        for (PLINE2 * pline = pa->cntr; pline != NULL; pline = pline->next)
            LabelPline(pline, other);
    } while ((pa = pa->f) != area);
} // LabelParea

////////////////////// Collect ////////////////////////////////

enum DIRECTION { FORW, BACKW };

// EdgeRule
typedef bool	(*EDGERULE)(const VNODE2 *, DIRECTION *);
typedef bool	(*CNTRRULE)(const PLINE2  *, DIRECTION *);

static
bool EdgeRuleUn(const VNODE2 * vn, DIRECTION * dir)
{
	ELABEL nLabel = GetVnodeLabel(vn);
	if (nLabel == E_OUTSIDE or nLabel == E_SHARED1)
	{
		*dir = FORW;
		return true;
	}
	return false;
} // EdgeRuleUn

static
bool EdgeRuleIs(const VNODE2 * vn, DIRECTION * dir)
{
	ELABEL nLabel = GetVnodeLabel(vn);
	if (nLabel == E_INSIDE or nLabel == E_SHARED1)
	{
		*dir = FORW;
		return true;
	}
	return false;
} // EdgeRuleIs

static
bool EdgeRuleSb(const VNODE2 * vn, DIRECTION * dir)
{
	ELABEL nLabel = GetVnodeLabel(vn);
	if (IsFirst(vn))
	{
		if (nLabel == E_OUTSIDE or nLabel == E_SHARED2)
		{
			*dir = FORW;
			return true;
		}
	}
	else
	{
		if (nLabel == E_INSIDE or nLabel == E_SHARED2)
		{
			*dir = BACKW;
			return true;
		}
	}
	return false;
} // EdgeRuleSb

static
bool EdgeRuleXr(const VNODE2 * vn, DIRECTION * dir)
{
	ELABEL nLabel = GetVnodeLabel(vn);
	if		(nLabel == E_OUTSIDE)
	{
		*dir = FORW;
		return true;
	}
	else if (nLabel == E_INSIDE)
	{
		*dir = BACKW;
		return true;
	}
	return false;
} // EdgeRuleXr

static
bool CntrRuleUn(const PLINE2 * pline, DIRECTION * dir)
{
	PLABEL nLabel = GetPlineLabel(pline);
	if (nLabel == P_OUTSIDE)
	{
		*dir = FORW;
		return true;
	}
	return false;
} // CntrRuleUn

static
bool CntrRuleIs(const PLINE2 * pline, DIRECTION * dir)
{
	PLABEL nLabel = GetPlineLabel(pline);
	if (nLabel == P_INSIDE)
	{
		*dir = FORW;
		return true;
	}
	return false;
} // CntrRuleIs

static
bool CntrRuleSb(const PLINE2 * pline, DIRECTION * dir)
{
	PLABEL nLabel = GetPlineLabel(pline);
	if (IsFirst(pline->head))
	{
		if (nLabel == P_OUTSIDE)
		{
			*dir = FORW;
			return true;
		}
	}
	else
	{
		if (nLabel == P_INSIDE)
		{
			*dir = BACKW;
			return true;
		}
	}
	return false;
} // CntrRuleSb

static
bool CntrRuleXr(const PLINE2 * pline, DIRECTION * dir)
{
	PLABEL nLabel = GetPlineLabel(pline);
	if		(nLabel == P_OUTSIDE)
	{
		*dir = FORW;
		return true;
	}
	else if (nLabel == P_INSIDE)
	{
		*dir = BACKW;
		return true;
	}
	return false;
} // CntrRuleXr

local
VNODE2 * Jump(VNODE2 * cur, DIRECTION * cdir, EDGERULE eRule)
{
	VLINK *start = (*cdir == FORW) ? cur->lnk.i : cur->lnk.o;

	if (start != NULL)
		for (VLINK *n = start->p; n != start; n = n->p)
		{
			if (start->dx == n->dx and start->dy == n->dy)
				continue;
			if (n->IsIn() and eRule(n->vn->prev, cdir) or
			   !n->IsIn() and eRule(n->vn, cdir))
				return n->vn;
		}
	return cur;
} // Jump

static
void CollectVnode(VNODE2 * start, PLINE2 ** result, EDGERULE edgeRule, DIRECTION initdir)
{
    VNODE2*	V = start;
	VNODE2*	E = (initdir == FORW) ? start : start->prev;
    DIRECTION   dir  = initdir;

    do {
        PLINE2::Incl(result, V->g);

        SetMarked(E);

        // for SHARED edge mark its neighbour
        if (GetVnodeLabel(E) == E_SHARED1 or
			GetVnodeLabel(E) == E_SHARED2)
            SetMarked(E->lnk.o->shared);

		// go forward, try to jump
        V = Jump((dir == FORW) ? V->next : V->prev, &dir, edgeRule);
        E = (dir == FORW) ? V : V->prev;
		// assert(GetVnodeLabel(E) != E_SHARED3);
    } while (not IsMarked(E));
} // CollectVnode

static
void CollectPline(PLINE2 * pline, PAREA ** r, PLINE2 ** holes, PAREA::PBOPCODE nOpCode)
{
	PLABEL nLabel = GetPlineLabel(pline);
    if (nLabel == P_ISECTED)
    {
		static const EDGERULE edgeRule[] =
		{
			EdgeRuleUn,		//	PBO_UNITE,
			EdgeRuleIs,		//	PBO_ISECT,
			EdgeRuleSb,		//	PBO_SUB,
			EdgeRuleXr		//	PBO_XOR
		};
		VNODE2 *vn = pline->head;
		do {
			DIRECTION dir;
			if (not IsMarked(vn) and edgeRule[nOpCode](vn, &dir))
			{
				PLINE2 *p = NULL;
				CollectVnode((dir == FORW) ? vn : vn->next, &p, edgeRule[nOpCode], dir);
				if (p->Prepare())
					PLINE2::Put(p, r, holes);
				else
					PLINE2::Del(&p);
			}
		} while ((vn = vn->next) != pline->head);
    } // nLabel == P_ISECTED
    else
    {
		static const CNTRRULE cntrRule[] = 
		{
			CntrRuleUn,		//	PBO_UNITE,
			CntrRuleIs,		//	PBO_ISECT,
			CntrRuleSb,		//	PBO_SUB,
			CntrRuleXr		//	PBO_XOR
		};
		DIRECTION dir;
		if (cntrRule[nOpCode](pline, &dir))
		{
			PLINE2 * copy = pline->Copy();
			if (not copy->Prepare())
				PLINE2::Del(&copy);
			else
			{
				if (dir == BACKW)
					copy->Invert();
				PLINE2::Put(copy, r, holes);
			}
		}
    }
} // CollectPline

local
void CollectParea(PAREA * area, PAREA ** r, PLINE2 ** holes, PAREA::PBOPCODE nOpCode)
{
    PAREA * pa = area;
    do {
        for (PLINE2 * pline = pa->cntr; pline != NULL; pline = pline->next)
            CollectPline(pline, r, holes, nOpCode);
    } while ((pa = pa->f) != area);
} // CollectParea

static
void DoBoolean(PAREA * a, PAREA * b, PAREA ** r, PAREA::PBOPCODE nOpCode)
{
	SortDesc(a);
	SortDesc(b);
	LabelParea(a, b);
	LabelParea(b, a);
	PLINE2 * holes = NULL;
    CollectParea(a, r, &holes, nOpCode);
    CollectParea(b, r, &holes, nOpCode);

	try
	{
		PAREA::InsertHoles(r, &holes);
	}
	catch (int)
	{
		PLINE2::Del(&holes);
		PAREA::Del(r);
		throw;
	}
} // DoBoolean

//////////////////////////////////////////////////////////

local bool Left(const GRID2 & a, const GRID2 & b)
{ return (a.x < b.x) or (a.x == b.x and a.y < b.y); }

static
void Area2Segms(PAREA * area, SEGM2 ** pSegm)
{
	if (area == NULL)
		return;
	
	PAREA * pa = area;
	do {
		for (PLINE2 * pline = pa->cntr; pline != NULL; pline = pline->next)
		{
			VNODE2 * vn = pline->head;
			do {
				SEGM2 * s = *pSegm; (*pSegm)++;

				s->m_bRight = Left(vn->g, vn->next->g);
				if (s->m_bRight)
					s->r = (s->l = vn)->next;
				else
					s->l = (s->r = vn)->next;
			} while ((vn = vn->next) != pline->head);
		}
	} while ((pa = pa->f) != area);
} // Area2Segms

static
UINT32 VertCnt(const PAREA * area)
{
	if (area == NULL)
		return 0;
	
	UINT32 nCnt = 0;
	const PAREA * pa = area;
	do {
		for (const PLINE2 * pline = pa->cntr; pline != NULL; pline = pline->next)
		{
			const VNODE2 * vn = pline->head;
			do {
				nCnt++;
			} while ((vn = vn->next) != pline->head);
		}
	} while ((pa = pa->f) != area);
	return nCnt;
} // VertCnt

// static
void InitArea(PAREA * area, bool bFirst)
{
	PAREA * pa = area;
	do {
		for (PLINE2 * pline = pa->cntr; pline != NULL; pline = pline->next)
		{
            pline->Flags = SETBITS(pline, PLINE2::RESERVED, P_UNKNOWN);

			VNODE2 * vn = pline->head;
			do {
                vn->Flags = SETBITS(vn, VNODE2::RESERVED, E_UNKNOWN);
				SetFirst(vn, bFirst);
                vn->lnk.i = vn->lnk.o = NULL;
			} while ((vn = vn->next) != pline->head);
		}
	} while ((pa = pa->f) != area);
} // InitArea

static
void InsertNode(void ** _list, VNODE2 * vn, void * parm)
{
	if (vn->lnk.i != NULL) // already tied
	{
		assert(vn->lnk.o != NULL);
		return;
	}
	// tie into ring all segments which point (X,Y) was inserted into

	VLINK ** list = (VLINK **)_list;
	LNK_HEAP * pHeap = (LNK_HEAP *)parm;
	VLINK * i = pHeap->Get(true);
	VLINK * o = pHeap->Get(true);
	i->SetIn(true);
	i->SetValid(false);
	o->SetIn(false);
	o->SetValid(false);
	i->vn = o->vn = vn;

	if (*list == NULL)
		*list = o->n = o->p = o;
	else
		(((o->p = (*list)->p)->n = o)->n = *list)->p = o;

	(((i->p = o->p)->n = i)->n = o)->p = i;
	vn->lnk.i = i;
	vn->lnk.o = o;
} // InsertNode

local
void RecalcCount(PAREA * area)
{
	PAREA * pa = area;
	do {
		for (PLINE2 * pline = pa->cntr; pline != NULL; pline = pline->next)
		{
			VNODE2 * vn = pline->head;
			UINT32 nCount = 0;
			do {
				nCount++;
			} while ((vn = vn->next) != pline->head);
			pline->Count = nCount;
		}
	} while ((pa = pa->f) != area);
} // RecalcCount

int PAREA::Boolean0(PAREA * a, PAREA * b, PAREA ** r, PBOPCODE nOpCode)
{
	*r = NULL;

	assert(a->CheckDomain());
	assert(b->CheckDomain());

	SEGM2 * aSegms = NULL;
	int err = 0;
	try
	{
		InitArea(a, true);
		InitArea(b, false);

		POLYBOOL pb;
		{
			UINT32	nSegms = VertCnt(a) + VertCnt(b);
			aSegms = (SEGM2 *)calloc(nSegms, sizeof(SEGM2));
			if (aSegms == NULL)
				error(err_no_memory);
			SEGM2 * pSegm = aSegms;

			Area2Segms(a, &pSegm);
			Area2Segms(b, &pSegm);

			assert(pSegm == aSegms + nSegms);

			BOCTX bc(InsertNode, &pb.m_LinkHeap);
			bc.Sweep(aSegms, nSegms);
		}
		DoBoolean(a, b, r, nOpCode);
		RecalcCount(a);
		RecalcCount(b);
	}
	catch (int e)
	{
		err = e;
	}
	catch (const STD::bad_alloc &)
	{
		err = err_no_memory;
	}
	free(aSegms);
	return err;
} // PAREA::Boolean0

int PAREA::Boolean(const PAREA * _a, const PAREA * _b, PAREA ** r, PBOPCODE nOpCode)
{
	*r = NULL;

	if (_a == NULL and _b == NULL)
		return 0;

	if		(_b == NULL)
	{
		int err = 0;
		if (_a != NULL and (nOpCode == SB or nOpCode == XR or nOpCode == UN))
		{
			try
			{
				*r = _a->Copy();
			}
			catch (int e)
			{
				err = e;
			}
		}
		return err;
	}
	else if (_a == NULL)
	{
		int err = 0;
		if (nOpCode == XR or nOpCode == UN)
		{
			try
			{
				*r = _b->Copy();
			}
			catch (int e)
			{
				err = e;
			}
		}
		return err;
	}

	PAREA * a = NULL, * b = NULL;
	if (_a != NULL and (a = _a->Copy()) == NULL)
		return err_no_memory;
	if (_b != NULL and (b = _b->Copy()) == NULL)
	{
		PAREA::Del(&a);
		return err_no_memory;
	}
	int err = Boolean0(a, b, r, nOpCode);

	PAREA::Del(&a);
	PAREA::Del(&b);

	return err;
} // PAREA::Boolean

} // namespace POLYBOOLEAN

