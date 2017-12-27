/// FicTrac http://rjdmoore.net/fictrac/
/// \file       SharedPointers.h
/// \brief      Shared pointer macros.
/// \author     Saul Thurrowgood
/// \copyright  CC BY-NC-SA 3.0

#ifndef _SHARED_POINTERS_H
#define _SHARED_POINTERS_H 1

///
/// A shared pointer is typedef'd for each TypeName as TypeNamePtr.
///
/// Usage within a header file:
///    1. #include this file after other includes, but before class declaration
///    2. write SHARED_PTR(TypeName); for each forward declaration used in
///       the header file
///

#include <boost/shared_ptr.hpp>

///
/// Use this in cases where the forward or full declaration is already given,
/// or where that declaration is more complicated than "class TypeName"
/// i.e. you will provide it yourself.
///
#define SHARED_PTR_NO_DEC(TypeName) \
	typedef boost::shared_ptr<TypeName> TypeName ## Ptr

///
/// Assumes the type is a class so forward declares it as "class TypeName".
///
#define SHARED_PTR(TypeName) \
	class TypeName; \
	SHARED_PTR_NO_DEC(TypeName)

#endif // _SHARED_POINTERS_H
