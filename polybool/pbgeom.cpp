//	pbgeom.cpp - GRID2 static members
//
//	This file is a part of PolyBoolean software library
//	(C) 1998-1999 Michael Leonov
//	Consult your license regarding permissions and restrictions
//

#include "pbgeom.h"
#include "pbimpl.h"

namespace POLYBOOLEAN
{

const GRID2	GRID2::PosInfinity = { INT20_MAX, INT20_MAX };
const GRID2	GRID2::NegInfinity = { INT20_MIN, INT20_MIN };

} // namespace POLYBOOLEAN

