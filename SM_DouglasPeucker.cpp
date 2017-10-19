#include "SM_DouglasPeucker.h"
#include "SM_Calc.h"

#include <float.h>

namespace sm
{

static inline
void points_reduction(const CU_VEC<vec2>& line, 
					  float precision, 
					  CU_VEC<bool>& flag,
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

void douglas_peucker(const CU_VEC<vec2>& line, 
					 float precision, 
					 CU_VEC<vec2>& dst)
{
	if (line.empty()) {
		return;
	}

	dst.clear();

	CU_VEC<bool> flag;
	flag.resize(line.size(), false);
	points_reduction(line, precision, flag, 0, line.size() - 1);
	for (int i = 0, n = line.size(); i < n; ++i) {
		if (flag[i]) dst.push_back(line[i]);
	}
}

}