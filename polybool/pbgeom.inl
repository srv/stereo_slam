//	pbgeom.inl - geometric operations for triangulation
//
//	This file is a part of PolyBoolean software library
//	(C) 1998-1999 Michael Leonov
//	Consult your license regarding permissions and restrictions
//

namespace POLYBOOLEAN
{

bool GRID2::AngleInside(const GRID2 & t, const GRID2 & a, const GRID2 & b)
{
	if ((a % b) >= 0)
	{
		if ((a % t) >= 0 and (t % b) > 0)
			return true;
	}
	else
	{
		if ((a % t) >= 0  or (t % b) > 0)
			return true;
	}
	return false;
} // GRID2::AngleInside

INT64 GRID2::Cross(const GRID2 & vp, const GRID2 & vc, const GRID2 & vn)
{
    return (vp - vc) % (vn - vc);
} // GRID2::Cross

bool GRID2::AngleConvex(const GRID2 & vp, const GRID2 & vc, const GRID2 & vn)
{
    return Cross(vp, vc, vn) < 0;
} // GRID2::AngleConvex

bool GRID2::gt(const GRID2 & v1, const GRID2 & v2)
{
    return (v1.y > v2.y or v1.y == v2.y and v1.x > v2.x);
} // GRID2::gt

bool GRID2::ls(const GRID2 & v1, const GRID2 & v2)
{
    return (v1.y < v2.y or v1.y == v2.y and v1.x < v2.x);
} // GRID2::ls

bool GRID2::ge(const GRID2 & v1, const GRID2 & v2)
{
    return (v1.y > v2.y or v1.y == v2.y and v1.x >= v2.x);
} // GRID2::ge

bool GRID2::le(const GRID2 & v1, const GRID2 & v2)
{
    return (v1.y < v2.y or v1.y == v2.y and v1.x <= v2.x);
} // GRID2::le

const GRID2 * GRID2::ymin(const GRID2 * v0, const GRID2 * v1)
{
    return ls(*v0, *v1) ? v0 : v1;
} // GRID2::ymin

const GRID2 * GRID2::ymax(const GRID2 * v0, const GRID2 * v1)
{
    return gt(*v0, *v1) ? v0 : v1;
} // GRID2::ymax

bool GRID2::IsLeft(const GRID2 & v, const GRID2 & v0, const GRID2 & v1)
{
    if (v1.y == v.y)
        return (v.x < v1.x);

    if (v0.y == v.y)
        return (v.x < v0.x);

    if (ls(v1, v0))
		return Cross(v0, v1, v) > 0;
	else
		return Cross(v1, v0, v) > 0;
} // GRID2::IsLeft

} // namespace POLYBOOLEAN

