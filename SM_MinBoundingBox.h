#pragma once

#include "SM_Vector.h"

#include <cu/cu_stl.h>

namespace sm
{

class MinBoundingBox
{
public:

	// bounding:
	// 1  2
	// 0  3
	static bool Do(const CU_VEC<vec2>& points,
		vec2 bounding[4]);

private:
	static bool TextOtherDir(const CU_VEC<vec2>& points, 
		const vec2& start, const vec2& end, 
		float& min_area, vec2 bounding[4]);
	static void CalculateB(const CU_VEC<vec2>& points, float k, 
		float& b_min, float& b_max);

}; // MinBoundingBox

}