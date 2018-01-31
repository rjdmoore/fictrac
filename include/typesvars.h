/// FicTrac http://rjdmoore.net/fictrac/
/// \file       TypesVars.h
/// \brief      Custom types and static variables used in FicTrac.
/// \author     Richard Moore, Saul Thurrowgood
/// \copyright  CC BY-NC-SA 3.0

#pragma once

#include "CmPoint.h"

typedef double CmReal;
typedef CmPointT<CmReal> CmPoint;

const CmReal CM_PI   = 3.14159265358979323846;
const CmReal CM_PI_2 = 1.57079632679489661923;
const CmReal CM_R2D  = 180.0 / CM_PI;
const CmReal CM_D2R  = CM_PI / 180.0;
