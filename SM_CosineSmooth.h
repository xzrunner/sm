#ifndef _SPATIAL_MATH_COSINE_SMOOTH_H_
#define _SPATIAL_MATH_COSINE_SMOOTH_H_

#include "SM_Vector.h"

#include <vector>

namespace sm
{

void cosine_smooth(const std::vector<vec2>& src,
				   float sampling_width,
				   std::vector<vec2>& dst);

}

#endif // _SPATIAL_MATH_COSINE_SMOOTH_H_