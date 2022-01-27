#include "SM_DouglasPeucker.h"
#include "SM_Calc.h"

#include <float.h>

namespace sm
{

static inline
void points_reduction(const std::vector<vec2>& line,
					  float precision,
					  std::vector<bool>& flag,
					  int begin,
					  int end)
{
	if (begin > end)
		return;

	if (begin == end)
	{
		flag[begin] = true;
		return;
	}

	if (end - begin < 2)
	{
		flag[begin] = true;
		flag[end] = true;
		return;
	}

	int saved_idx = 0;
	float max = - FLT_MAX;
	for (int i = begin + 1; i < end; ++i)
	{
		float dis = dis_pos_to_seg(line[i], line[begin], line[end]);
		if (dis > max)
		{
			max = dis;
			saved_idx = i;
		}
	}

	if (max < precision)
	{
		flag[begin] = true;
		flag[end] = true;
	}
	else
	{
		points_reduction(line, precision, flag, begin, saved_idx);
		points_reduction(line, precision, flag, saved_idx, end);
	}
}

std::vector<vec2> douglas_peucker(const std::vector<vec2>& poly, float precision)
{
	if (poly.empty()) {
		return {};
	}

	std::vector<vec2> dst;

	std::vector<bool> flag;
	flag.resize(poly.size(), false);
	points_reduction(poly, precision, flag, 0, static_cast<int>(poly.size() - 1));
	for (size_t i = 0, n = poly.size(); i < n; ++i) {
		if (flag[i]) dst.push_back(poly[i]);
	}

	return dst;
}

}