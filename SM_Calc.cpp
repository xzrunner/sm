#include "SM_Calc.h"

namespace sm
{

vec3 calc_plane_mirror(const Plane& plane, const vec3& pos)
{
	float len_s = plane.normal.LengthSquared();
	auto v1 = pos;
	float k = - plane.GetDistance(v1) / len_s;
	auto v2 = plane.normal * k + v1;
	return v2 * 2 - v1;
}

bool intersect_segment_rect(const vec2& s, const vec2& e, const rect& r, std::vector<vec2>& cross)
{
	unsigned char type_s, type_e;
	type_s = type_e = 0;
	if (s.x < r.xmin)      // left:  1000
		type_s |= 0x8; 
	else if (s.x > r.xmax) // right: 0100
		type_s |= 0x4; 
	if (s.y < r.ymin)      // down:  0001
		type_s |= 0x1; 
	else if (s.y > r.ymax) // up:    0010
		type_s |= 0x2; 

	if (e.x < r.xmin)      // left:  1000
		type_e |= 0x8; 
	else if (e.x > r.xmax) // right: 0100
		type_e |= 0x4; 
	if (e.y < r.ymin)      // down:  0001
		type_e |= 0x1; 
	else if (e.y > r.ymax) // up:    0010
		type_e |= 0x2; 

	unsigned char comp;
	comp = type_s & type_e;
	if (comp != 0)		// must be outside, so must intersect
		return false;
	comp = type_s | type_e;
	if (comp == 0)		// must be inside, so must not intersect
		return false;

	// test each edge
	if (comp & 0x8)		// 1000, left edge
	{
		float cross_y;
		cross_y = find_y_on_seg(s, e, r.xmin);
		if (cross_y >= r.ymin && cross_y <= r.ymax)
			cross.push_back(vec2(r.xmin, cross_y));
	}
	if (comp & 0x4)		// 0100, right edge
	{
		float cross_y;
		cross_y = find_y_on_seg(s, e, r.xmax);
		if (cross_y >= r.ymin && cross_y <= r.ymax)
			cross.push_back(vec2(r.xmax, cross_y));
	}
	if (comp & 0x1)		// 0001, down edge
	{
		float cross_x;
		cross_x = find_x_on_seg(s, e, r.ymin);
		if (cross_x >= r.xmin && cross_x <= r.xmax)
			cross.push_back(vec2(cross_x, r.ymin));
	}
	if (comp & 0x2)		// 0010, up edge
	{
		float cross_x;
		cross_x = find_x_on_seg(s, e, r.ymax);
		if (cross_x >= r.xmin && cross_x <= r.xmax)
			cross.push_back(vec2(cross_x, r.ymax));
	}

	return !cross.empty();
}

bool intersect_segment_polyline(const vec2& s0, const vec2& s1, const std::vector<vec2>& polyline, vec2* cross, size_t* idx)
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

void transform_plane(Plane& plane, const mat4& mt)
{
    auto& norm = plane.normal;

    sm::vec3 pos0;
    if (norm.x != 0)
    {
        pos0.x = -plane.dist / norm.x;
        pos0.y = 0;
        pos0.z = 0;
    }
    else if (norm.y != 0)
    {
        pos0.x = 0;
        pos0.y = -plane.dist / norm.y;
        pos0.z = 0;
    }
    else if (norm.z != 0)
    {
        pos0.x = 0;
        pos0.y = 0;
        pos0.z = -plane.dist / norm.z;
    }

    auto pos1 = pos0 + norm;

    pos0 = mt * pos0;
    pos1 = mt * pos1;
    plane.Build(pos1 - pos0, pos0);
}

}