#ifndef _SPATIAL_MATH_DOUGLAS_PEUCKER_H_
#define _SPATIAL_MATH_DOUGLAS_PEUCKER_H_

#include "SM_Vector.h"

#include <vector>

namespace sm
{

std::vector<vec2> douglas_peucker(const std::vector<vec2>& poly, float precision);

}

#endif // _SPATIAL_MATH_DOUGLAS_PEUCKER_H_