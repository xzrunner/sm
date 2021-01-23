#ifndef _SPATIAL_MATH_CONVEX_HULL_H_
#define _SPATIAL_MATH_CONVEX_HULL_H_

#include "SM_Vector.h"

#include <vector>

namespace sm
{

void convex_hull(const std::vector<vec2>& points,
				 std::vector<vec2>& convex_hull);

}

#endif // _SPATIAL_MATH_CONVEX_HULL_H_