#ifndef _SPATIAL_MATH_CONVEX_HULL_H_
#define _SPATIAL_MATH_CONVEX_HULL_H_

#include "SM_Vector.h"

#include <cu/cu_stl.h>

namespace sm
{

void convex_hull(const CU_VEC<vec2>& points,
				 CU_VEC<vec2>& convex_hull);

}

#endif // _SPATIAL_MATH_CONVEX_HULL_H_