//	pbtria.h - triangulation, for internal use only
//
//	This file is a part of PolyBoolean software library
//	(C) 1998-1999 Michael Leonov
//	Consult your license regarding permissions and restrictions
//

#ifndef _TRIANGUL_H_
#define _TRIANGUL_H_

#include "polybool.h"
#include "pbimpl.h"
#include "pbgeom.inl"
#include "ObjHeap.h"

namespace POLYBOOLEAN
{

struct YNODE;
struct XNODE;
struct QNODE;
struct MCHAIN;

struct V_DESC
{
    QNODE         *	root_n;
	QNODE         *	root_p;
    const VNODE2  *	vnext[4];	// next vertices for the 4 chains
    MCHAIN        *	mpos[4];	// position of the segm in the 4 chains
    unsigned int	nextfree;
};

inline const V_DESC * VN_D(const VNODE2 * vn) {
	return reinterpret_cast<const V_DESC*>(vn->v);
}

inline V_DESC * VN_D(VNODE2 * vn) {
	return reinterpret_cast<V_DESC*>(vn->v);
}

struct TRAP
{
    VNODE2       *l, *r;
    const GRID2 *hi, *lo;
    TRAP        *u0, *u1, *d0, *d1;
    QNODE       *sink;
    bool		bVisited;
};

struct YNODE
{
    const GRID2 *y;
    QNODE       *a, *b;
};

struct XNODE
{
    const VNODE2 *vn;
    QNODE       *r, *l;
};

/* Node attributes for every node in the query structure */
struct QNODE
{
	enum { Y, X, SINK } type;
    union
    {
        YNODE   yn;		// Y-node
        XNODE   xn;		// X-node
        TRAP    *tr;	// sink
    };
    QNODE    *parent;
};

/* Circularly linked list
   describing the monotone polygon */
struct MCHAIN
{
    VNODE2       *vn;
    MCHAIN      *n, *p;
    bool        marked;
};

typedef ObjStorageClass<TRAP, 16, err_no_memory> TRAP_HEAP;

class TRIAGLOBS
{
public:
	TRIAGLOBS()
	{
		m_area = NULL;
		m_rndv = NULL;
		m_mon = NULL;
		m_rc = NULL;
		m_tria_idx = m_mon_idx = m_choose_idx = m_nseg = 0;
	}
	~TRIAGLOBS()
	{
		free(m_rndv);
		free(m_mon);
		free((void *)m_rc);
	}

	// !!! should be called only once
	void Triangulate(PAREA * p);

	// called for every triangle
	void TriActor(const VNODE2 *v0, const VNODE2 *v1, const VNODE2 *v2);

protected:
	ObjStorageClass< QNODE, 16, err_no_memory>	m_qs;
	ObjStorageClass<V_DESC, 16, err_no_memory>	m_vd;
	ObjStorageClass<MCHAIN, 16, err_no_memory>	m_mc;
	TRAP_HEAP	m_trap;

    VNODE2   **	m_rndv;				// array of randomized nodes
	PAREA	  *	m_area;				// input PAREA
    UINT32		m_nseg;				// number of vertices in m_area

	MCHAIN   **	m_mon;				// array of monotone chains
	UINT32		m_mon_idx, m_choose_idx;

    const VNODE2 **	m_rc;			// reflex chain for monotone triangulation

	UINT32		m_tria_idx;			// index of currently processed triangle

	MCHAIN **newmon()
	{
		if (m_mon_idx >= m_nseg)
			error(err_bad_parm);
		return m_mon + m_mon_idx++;
	} // newmon

	MCHAIN* NEW_MCHAIN() {
		return m_mc.Get(true);
	}

	QNODE *	NEW_NODE() {
		return m_qs.Get(true);
	}

	TRAP *newtrap() {
		return m_trap.Get(true);
	} // newtrap

	void puttrap(TRAP *t)
	{
		m_trap.Put(t);
	} // puttrap

// Return the next segment in the generated random ordering of all the
// segments in S
	VNODE2 *choose_segment()
	{ return m_rndv[m_choose_idx++]; }

public:
	enum UpSide
	{
		S_LEFT,
		S_RIGHT
	};

	enum TraverseDir
	{
		TR_FROM_UP,
		TR_FROM_DN
	};

	enum MonSingleSide
	{
		TRI_L,
		TRI_R
	};

protected:
	TRAP  *	InsertVertex(bool fst, const QNODE *root, const GRID2 *v0, const GRID2 *v1);
	QNODE *	InitQueryStructure(VNODE2 *s);
	TRAP  *	SplitCurTrap(TRAP *t, const VNODE2 *s);
	void	MergeTrapezoids(const VNODE2 *s, TRAP *tfirst, TRAP *tlast, UpSide side);
	void	AddSegment(VNODE2 *s);
	void    ConstructTrapezoids();

	MCHAIN ** MakeNewMonotonePoly(MCHAIN ** mcur, VNODE2 *vp0, VNODE2 *vp1);
	void	TraverseCusps(MCHAIN **mcur, TRAP *t, TRAP *from, TRAP *d0, TRAP *d1, bool fst);
	void	OnlyCusp(MCHAIN **mcur, TRAP *t, TRAP *from, TraverseDir dir,
                    TRAP *u0, TRAP *u1, TRAP *d0, TRAP *d1, bool fst,
                    const GRID2 *chk1, const GRID2 *chk2);
	void	NoCusp(TRAP *t, MCHAIN **mcur, VNODE2 *vp0, VNODE2 *vp1, TraverseDir dir);
	void	TraversePolygon(MCHAIN **mcur, TRAP *t, TRAP *from, TraverseDir dir);
	void	TriangulateMonPolys();
	void	TriangulateSinglePolygon(MCHAIN * posmax, MonSingleSide side);
	void	MonotonateTrapezoids();
};

} // namespace POLYBOOLEAN

#endif // _TRIANGUL_H_

