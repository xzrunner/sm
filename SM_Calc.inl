#ifndef _SPATIAL_MATH_CALC_INL_
#define _SPATIAL_MATH_CALC_INL_

#include <float.h>

namespace sm
{

inline
float find_x_on_seg(const vec2& s, const vec2& e, float y)
{
	if (s.y == e.y)  {
		return (std::min)(s.x, e.x);
	} else {
		return (y - e.y) * (s.x - e.x) / (s.y - e.y) + e.x;
	}
}

inline
float find_y_on_seg(const vec2& s, const vec2& e, float x)
{
	if (s.x == e.x)  {
		return (std::min)(s.y, e.y);
	} else {
		return (x - e.x) * (s.y - e.y) / (s.x - e.x) + e.y;
	}
}

inline
vec2 rotate_vector(const vec2& v, float rad)
{
	vec2 ret;
	ret.x = v.x * cos(rad) - v.y * sin(rad);
	ret.y = v.x * sin(rad) + v.y * cos(rad);
	return ret;
}

inline
vec2 rotate_vector_right_angle(const vec2& v, bool turn_left)
{
	vec2 ret = v;
	if (turn_left)
	{
		ret.x = -v.y;
		ret.y = v.x;
	}
	else
	{
		ret.x = v.y;
		ret.y = -v.x;
	}
	return ret;
}

inline
float mat_trans_len(float len, const mat4& mat)
{
	vec2 p0 = mat * vec2(0, 0),
		 p1 = mat * vec2(len, 0);
	return (p0 - p1).Length();
}

inline
float get_line_angle(const vec2& s, const vec2& e)
{
	return atan2(e.y - s.y, e.x - s.x);
}

inline
bool is_acute_angle(const vec2& a, const vec2& center, const vec2& b) 
{
	float lac = (a - center).LengthSquared(),
		  lbc = (b - center).LengthSquared(),
		  lab = (a -b).LengthSquared();
	return lac + lbc - lab > 0;
}

inline
bool is_turn_left(const vec2& a, const vec2& center, const vec2& b) 
{
	return (center.x - a.x) * (b.y - center.y) - (b.x - center.x) * (center.y - a.y) > 0;
}

inline
bool is_turn_right(const vec2& a, const vec2& center, const vec2& b)
{
	return (center.x - a.x) * (b.y - center.y) - (b.x - center.x) * (center.y - a.y) < 0;
}

inline
float dis_pos_to_pos(const vec2& v0, const vec2& v1)
{
	return (v0 - v1).Length();
}

inline
float dis_pos_to_multi_pos(const vec2& pos, const std::vector<vec2>& multi_pos, int* nearest_idx)
{
	float nearest = FLT_MAX;
	int idx = -1;
	for (int i = 0, n = multi_pos.size(); i < n; ++i)
	{
		const float dis = dis_pos_to_pos(pos, multi_pos[i]);
		if (dis < nearest)
		{
			nearest = dis;
			idx = i;
		}
	}
	if (nearest_idx) {
		*nearest_idx = idx;
	}
	return nearest;
}

inline
float dis_pos_to_seg(const vec2& v, const vec2& s0, const vec2& s1)
{
	if (s0 == s1) {
		return dis_pos_to_pos(v, s0);
	} else if (!is_acute_angle(v, s0, s1)) {
		return dis_pos_to_pos(v, s0);
	} else if (!is_acute_angle(v, s1, s0)) {
		return dis_pos_to_pos(v, s1);
	} else {
		return fabs((s0.x - v.x) * (s1.y - v.y) - (s0.y - v.y) * (s1.x - v.x)) / dis_pos_to_pos(s0, s1);
	}
}

inline
float distance_aabb(const vec3& pos, const vec3& aabb_min, const vec3& aabb_max)
{
	vec3 center;
	center.x = (aabb_min.x + aabb_max.x) * 0.5f;
	center.y = (aabb_min.y + aabb_max.y) * 0.5f;
	center.z = (aabb_min.z + aabb_max.z) * 0.5f;

	vec3 extent;
	extent.x = (aabb_max.x - aabb_min.x) * 0.5f;
	extent.y = (aabb_max.y - aabb_min.y) * 0.5f;
	extent.z = (aabb_max.z - aabb_min.z) * 0.5f;

	vec3 nearest_vec;
	nearest_vec.x = std::max(0.0f, fabsf(pos.x - center.x) - extent.x);
	nearest_vec.y = std::max(0.0f, fabsf(pos.y - center.y) - extent.y);
	nearest_vec.z = std::max(0.0f, fabsf(pos.z - center.z) - extent.z);

	return nearest_vec.Length();
}

inline
bool intersect_line_line(const vec2& s0, const vec2& e0, const vec2& s1, const vec2& e1, vec2* cross) 
{
	// If they are parallel ?
	float denominator_x = (e1.y - s1.y) * (e0.x - s0.x) - (e0.y - s0.y) * (e1.x - s1.x),
		  denominator_y = (e1.x - s1.x) * (e0.y - s0.y) - (e0.x - s0.x) * (e1.y - s1.y);
	if (fabs(denominator_x) < SM_LARGE_EPSILON || fabs(denominator_y) < SM_LARGE_EPSILON)
	{
		cross->x = cross->y = FLT_MAX;
		return false;
	} else {
		cross->x = ( (e0.x * s0.y - s0.x * e0.y) * (e1.x - s1.x) - (e1.x * s1.y - s1.x * e1.y) * (e0.x - s0.x) ) / denominator_x;
		cross->y = ( (e0.y * s0.x - s0.y * e0.x) * (e1.y - s1.y) - (e1.y * s1.x - s1.y * e1.x) * (e0.y - s0.y) ) / denominator_y;
		return true;
	}
}

}

#endif // _SPATIAL_MATH_CALC_INL_