//	pbimpl.h - common definitions for all PolyBoolean source files
//
//	This file is a part of PolyBoolean software library
//	(C) 1998-1999 Michael Leonov
//	Consult your license regarding permissions and restrictions
//

#ifndef _PBIMPL_H_
#define _PBIMPL_H_

#ifdef _MSC_VER
#pragma warning(disable : 4018 4056 4100 4127 4146 4514 4663 4710 4711 4786 4800)
#define for		if (false) ; else for
#if _MSC_VER >= 1200
#pragma warning(push, 1)
#endif // _MSC_VER >= 1200
#endif // _MSC_VER

#include <assert.h>
#include <iso646.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <list>
#include <queue>
#include <vector>
namespace STD = std;

#ifdef _MSC_VER
#if _MSC_VER >= 1200
#pragma warning(pop)
#endif // _MSC_VER >= 1200
#endif

#include "pbdefs.h"

#define local		static inline

inline void error(int nCode) {
	throw nCode;
}

#endif // _PBIMPL_H_

