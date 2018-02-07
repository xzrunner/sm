#include "SM_CosineSmooth.h"
#include "sm_const.h"
#include "SM_Math.h"

#include <cmath>

namespace sm
{

void cosine_smooth(const CU_VEC<vec2>& src, 
				   float sampling_width, 
				   CU_VEC<vec2>& dst)
{
	dst.clear();

	if (src.size() < 2) return;

	for (int i = 0, n = src.size() - 1; i < n; ++i)
	{
		const vec2& p0 = src[i],
			        p1 = src[i+1];
		const int sampling_count = static_cast<int>(std::floor(std::abs(p1.x - p0.x) / sampling_width));
		const float dx = (p1.x - p0.x) / sampling_count;
		const float da = SM_PI / sampling_count;
		const float ymid = (p0.y + p1.y) * 0.5f;
		const float ampl = (p0.y - p1.y) * 0.5f;
		dst.push_back(p0);
		for (int j = 1; j < sampling_count; ++j)
		{
			vec2 pt;
			pt.x = p0.x + j * dx;
			pt.y = ymid + ampl * cos(da * j);
			dst.push_back(pt);
		}
	}
	dst.push_back(src.back());
}

}