#ifndef _SPATIAL_MATH_PROCESS_INL_
#define _SPATIAL_MATH_PROCESS_INL_

#include "SM_Calc.h"

namespace sm
{

inline
void rm_duplicate_nodes(const std::vector<vec2>& src, std::vector<vec2>& dst)
{
	if (src.size() > 1)
	{
		dst.reserve(src.size());

		dst.push_back(src[0]);
		vec2 last = src[0];
		for (size_t i = 1; i < src.size(); ++i)
		{
			if (dis_pos_to_pos(src[i], last) > 1.0f) {
				dst.push_back(src[i]);
			}
			last = src[i];
		}
	}
	else
	{
		dst = src;
	} 
}

inline
void trans_vertices(const mat4& mt, const std::vector<vec2>& src, std::vector<vec2>& dst)
{
	dst.clear();
	dst.reserve(src.size());
	for (int i = 0, n = src.size(); i < n; ++i) {
		dst.push_back(mt * src[i]);
	}
}

}

#endif // _SPATIAL_MATH_PROCESS_INL_