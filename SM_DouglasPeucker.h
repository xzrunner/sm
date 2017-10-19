#ifndef _SPATIAL_MATH_DOUGLAS_PEUCKER_H_
#define _SPATIAL_MATH_DOUGLAS_PEUCKER_H_

#include "SM_Vector.h"

#include <cu/cu_stl.h>

namespace sm
{

void douglas_peucker(const CU_VEC<vec2>& line, 
					 float precision, 
					 CU_VEC<vec2>& dst);

}

#endif // _SPATIAL_MATH_DOUGLAS_PEUCKER_H_