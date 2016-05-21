#ifndef _SPATIAL_MATH_DOUGLAS_PEUCKER_H_
#define _SPATIAL_MATH_DOUGLAS_PEUCKER_H_

#include "SM_Vector.h"

#include <vector>

namespace sm
{

void douglas_peucker(const std::vector<vec2>& line, 
					 float precision, 
					 std::vector<vec2>& dst);

}

#endif // _SPATIAL_MATH_DOUGLAS_PEUCKER_H_