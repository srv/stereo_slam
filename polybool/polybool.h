//	polybool.h - main PolyBoolean header
//
//	This file is a part of PolyBoolean software library
//	(C) 1998-1999 Michael Leonov
//	Consult your license regarding permissions and restrictions
//

#ifndef _POLYBOOL_H_
#define _POLYBOOL_H_

#include "pbdefs.h"
#include "pbgeom.h"

namespace POLYBOOLEAN
{

struct VLINK;
struct PAREA;

struct VNODE2
{
    VNODE2 *next, *prev;
    UINT32	Flags;

// Flags reserved for internal use
	enum {
		RESERVED = 0x00FFu
	};

	union
	{
		VECT2   p;
		GRID2	g;		// vertex coordinates
	};

    union	// temporary fields
    {
        struct
        {
            VLINK *i, *o;
        } lnk;

        int     i;
        void  * v;
        VNODE2* vn;
    };

	static VNODE2 * New(const GRID2 & g);

	void Incl(VNODE2 * after) {
		(((next = after->next)->prev = this)->prev = after)->next = this;
	}

	void Excl() {
		(prev->next = next)->prev = prev;
	}

}; // struct VNODE2

struct PLINE2
{
    PLINE2*	next;	// next contour
    VNODE2*	head;	// first vertex
    UINT32  Count;	// number of vertices
	UINT32  Flags;
	union
	{
		VECT2	vMin;
		GRID2	gMin;
	};
	union
	{
		VECT2	vMax;
		GRID2	gMax;
	};

	enum {
		RESERVED = 0x00FF,
		ORIENT   = 0x0100,
		DIR      = 0x0100,
		INV      = 0x0000
	};

	bool IsOuter() const
	{ return (Flags & ORIENT) == DIR; }

	static
	void    Del(PLINE2 ** c);

	static
	PLINE2 * New(const GRID2 & g);

	PLINE2 *	Copy(bool bMakeLinks = false) const;

	static
	void	Incl(PLINE2 ** pline, const GRID2 & g);

	//	calculate orientation and bounding box
	//	removes points lying on the same line
	//	returns if contour is valid, i.e. Count >= 3 and Area != 0
	bool    Prepare();
	double    Prepare2();

	//	invert contour
	void	Invert();

	// put pline either into area or holes depending on its orientation
	static
	void Put(PLINE2 * pline, PAREA ** area, PLINE2 ** holes);
};

/************************* PAREA ***************************/

struct PTRIA2
{
    VNODE2   *v0, *v1, *v2;
};

struct PAREA
{
    PAREA *	f, * b;
    PLINE2 *	cntr;
    PTRIA2 *	tria;
    UINT32	tnum;

	static PAREA * New();
	static void    Del(PAREA ** p);
	PAREA * Copy() const;

	void Excl() { (b->f = f)->b = b; }

	enum PBOPCODE
	{
		UN,	// union
		IS,	// intersection
		SB,	// difference
		XR	// symmetrical difference
	};

	static int	Boolean(const PAREA * a, const PAREA * b, PAREA ** r, PBOPCODE nOpCode);

// PolyBoolean0 operates destructively on a and b
	static int	Boolean0(PAREA * a, PAREA * b, PAREA ** r, PBOPCODE nOpCode);

	static int	Triangulate(PAREA * area);

	static void	InclParea(PAREA ** list, PAREA *a);
	static void InclPline(PAREA ** area, PLINE2 * pline);
	static void InsertHoles(PAREA ** area, PLINE2 ** holes);
#ifndef NDEBUG
	bool CheckDomain();	// check if coordinates are within 20 bit grid
#endif // NDEBUG
}; // struct PAREA

bool    GridInPline(const GRID2 * g, const PLINE2 * outer);
bool    GridInParea(const GRID2 * g, const PAREA * outer);
bool    PlineInPline(const PLINE2* p, const PLINE2 * outer);
bool    PlineInParea(const PLINE2* p, const PAREA * outer);


} // namespace POLYBOOLEAN

#endif // _POLYBOOL_H_

