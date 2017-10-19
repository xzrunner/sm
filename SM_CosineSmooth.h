#ifndef _SPATIAL_MATH_COSINE_SMOOTH_H_
#define _SPATIAL_MATH_COSINE_SMOOTH_H_

#include "SM_Vector.h"

#include <cu/cu_stl.h>

namespace sm
{

void cosine_smooth(const CU_VEC<vec2>& src, 
				   float sampling_width, 
				   CU_VEC<vec2>& dst);

}

#endif // _SPATIAL_MATH_COSINE_SMOOTH_H_