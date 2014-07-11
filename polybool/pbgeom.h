//	pbgeom.h - basic geometric operations
//
//	This file is a part of PolyBoolean software library
//	(C) 1998-1999 Michael Leonov
//	Consult your license regarding permissions and restrictions
//

#ifndef _PBGEOM_H_
#define _PBGEOM_H_

#include "pbdefs.h"
#include <iso646.h>

namespace POLYBOOLEAN
{

struct GRID2
{
	INT32	x, y;

	static const GRID2	PosInfinity;
	static const GRID2	NegInfinity;

	// returns if t lies inside [a, b)
	static inline bool	AngleInside(const GRID2 & t, const GRID2 & a, const GRID2 & b);

	static inline INT64 Cross(const GRID2 & vp, const GRID2 & vc, const GRID2 & vn);
	static inline bool	AngleConvex(const GRID2 & vp, const GRID2 & vc, const GRID2 & vn);
	static inline bool	gt(const GRID2 & v1, const GRID2 & v2);
	static inline bool	ls(const GRID2 & v1, const GRID2 & v2);
	static inline bool	ge(const GRID2 & v1, const GRID2 & v2);
	static inline bool	le(const GRID2 & v1, const GRID2 & v2);
	static inline bool	IsLeft(const GRID2 & v, const GRID2 & v0, const GRID2 & v1);
	static inline const GRID2 *ymin(const GRID2 * v0, const GRID2 * v1);
	static inline const GRID2 *ymax(const GRID2 * v0, const GRID2 * v1);
};

inline GRID2 operator-(const GRID2 & v0)
{
	GRID2 v = {-v0.x, -v0.y};
	return v;
}

inline GRID2 & operator+=(GRID2 & v0, const GRID2 & v)
{
	v0.x += v.x;
	v0.y += v.y;
	return v0;
}

inline GRID2 & operator-=(GRID2 & v0, const GRID2 & v)
{
	v0.x -= v.x;
	v0.y -= v.y;
	return v0;
}

inline GRID2 & operator*=(GRID2 & v0, INT32 d)
{
	v0.x *= d;
	v0.y *= d;
	return v0;
}

inline GRID2 & operator/=(GRID2 & v0, INT32 d)
{
	v0.x /= d;
	v0.y /= d;
	return v0;
}

inline GRID2 operator+(const GRID2 & a, const GRID2 & b)
{
	GRID2 v = {a.x + b.x, a.y + b.y};
	return v;
}

inline GRID2 operator-(const GRID2 & a, const GRID2 & b)
{
	GRID2 v = {a.x - b.x, a.y - b.y};
	return v;
}

inline GRID2 operator*(const GRID2 & a, INT32 b)
{
	GRID2 v = {a.x * b, a.y * b};
	return v;
}

inline GRID2 operator*(INT32 b, const GRID2 & a)
{
	GRID2 v = {a.x * b, a.y * b};
	return v;
}

inline GRID2 operator/(const GRID2 & a, INT32 b)
{
	GRID2 v = {a.x / b, a.y / b};
	return v;
}

inline INT64 operator*(const GRID2 & a, const GRID2 & b)
{
	return a.x * b.x + a.y * b.y;
}

inline bool operator==(const GRID2 & a, const GRID2 & b)
{
	return (a.x == b.x and a.y == b.y);
}

inline bool operator!=(const GRID2 & a, const GRID2 & b)
{
	return (a.x != b.x or a.y != b.y);
}

inline INT64 operator%(const GRID2 & a, const GRID2 & b)
{
	return (INT64)a.x * b.y - (INT64)a.y * b.x;
}

} // namespace POLYBOOLEAN

#endif // _PBGEOM_H_

