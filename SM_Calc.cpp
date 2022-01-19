#include "SM_Calc.h"

namespace sm
{

sm::vec3 calc_plane_mirror(const Plane& plane, const sm::vec3& pos)
{
	float len_s = plane.normal.LengthSquared();
	auto v1 = pos;
	float k = (-plane.normal.Dot(v1) - plane.dist) / len_s;
	auto v2 = plane.normal * k + v1;
	return v2 * 2 - v1;
}

bool intersect_segment_polyline(const vec2& s0, const vec2& s1, const std::vector<sm::vec2>& polyline, vec2* cross, size_t* idx)
{
	for (size_t i = 0, n = polyline.size(); i < n; ++i)
	{
		if (is_point_in_segment(polyline[i], s0, s1)) 
		{
			*cross = polyline[i];
			*idx = i;
			return true;
		} 
		else if (intersect_segment_segment(polyline[i], polyline[(i + 1) % n], s0, s1, cross))
		{
			*idx = i;
			return true;
		}
	}
	return false;
}

}