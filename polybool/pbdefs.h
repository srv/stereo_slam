//	pbdefs.h - common definitions for PolyBoolean
//
//	This file is a part of PolyBoolean software library
//	(C) 1998-1999 Michael Leonov
//	Consult your license regarding permissions and restrictions
//

#ifndef _PBDEFS_H_
#define _PBDEFS_H_

namespace POLYBOOLEAN
{

////////////// Begin of the platform specific section //////////////////

#ifdef _MSC_VER

typedef __int32				INT32;
typedef __int64				INT64;
typedef unsigned __int32	UINT32;

#else

// insert platform specific sized integer types here

typedef int					INT32;
typedef long long			INT64;
typedef unsigned int		UINT32;

#endif // _MSC_VER

////////////// End of the platform specific section //////////////////

// if you would like to use your own VECT2, simply put it here
// instead of the default struct VECT2 definition
struct VECT2
{
	double	x, y;
};

} // namespace POLYBOOLEAN

// ranges for the integer coordinates
#define INT20_MAX			+524287
#define INT20_MIN			-524288

// error codes thrown by the library 
enum {
	err_ok = 0,		// never thrown
	err_no_memory,	// not enough memory
	err_io,			// file I/O error
	err_bad_parm	// bad input data
};

#endif // _PBDEFS_H_

