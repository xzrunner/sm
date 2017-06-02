#ifndef _SPATIAL_MATH_CALC_INL_
#define _SPATIAL_MATH_CALC_INL_

#include "SM_Test.h"
#include "SM_MatrixFix.h"
#include "SM_Matrix2D.h"
#include "SM_Math.h"

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
	if (rad == 0) {
		return v;
	}

	vec2 ret;
	float s = sm::sin(rad),
		  c = sm::cos(rad);
	ret.x = v.x * c - v.y * s;
	ret.y = v.x * s + v.y * c;
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
float mat_trans_len(float len, const Matrix2D& mat)
{
	vec2 p0 = mat * vec2(0, 0),
		 p1 = mat * vec2(len, 0);
	return (p0 - p1).Length();
}

inline
float mat_trans_len(float len, const MatrixFix& mat)
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
float get_angle(const vec2& center, const vec2& pa, const vec2& pb)
{
	const float a = dis_pos_to_pos(center, pa),
		        b = dis_pos_to_pos(center, pb),
		        c = dis_pos_to_pos(pa, pb);

	float cos_val = (a * a + b * b - c * c) / (2 * a * b);
	cos_val = std::max(std::min(cos_val, 1.0f), -1.0f);

	return acos(cos_val);

	// 	float angle = acos(cosVal);
	// 	return is_turn_right(pa, center, pb) ? angle : -angle;
}

inline
float get_angle_in_direction(const vec2& center, const vec2& start, const vec2& end)
{
	float angle = get_angle(center, start, end);
	const float cross = (start - center).Cross(end - start);
	if (cross < 0) angle = -angle;
	return angle;
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
float dis_square_pos_to_pos(const vec2& v0, const vec2& v1)
{
	return (v0 - v1).LengthSquared();	
}

inline
float dis_pos_to_multi_pos(const vec2& pos, const std::vector<vec2>& multi_pos, int* nearest_idx)
{
	float nearest = FLT_MAX;
	int idx = -1;
	for (size_t i = 0, n = multi_pos.size(); i < n; ++i)
	{
		const float dis = dis_pos_to_pos(pos, multi_pos[i]);
		if (dis < nearest)
		{
			nearest = dis;
			idx = (int)i;
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

inline
bool intersect_segment_segment(const vec2& s0, const vec2& e0, const vec2& s1, const vec2& e1, vec2* cross)
{
	bool line_intersect = intersect_line_line(s0, e0, s1, e1, cross);
	if (line_intersect) {
		return is_segment_intersect_segment(s0, e0, s1, e1);
	} else {
		return false;
	}
}

inline
int get_foot_of_perpendicular(const vec2& s, const vec2& e, const vec2& out, vec2* foot) 
{
	const float dx = e.x - s.x, dy = e.y - s.y;
	const float dx_square = dx * dx, dy_square = dy * dy;

	if (dx_square + dy_square < FLT_EPSILON)
	{
		*foot = s;
		return -1;
	}

	if (fabs(s.x - e.x) > fabs(s.y - e.y))
	{
		foot->x = (dx_square * out.x + dy_square * s.x + dx * dy * (out.y - s.y)) / (dx_square + dy_square);
		if (s.x == e.x)
		{
			foot->y = out.y;
		}
		else
		{
			foot->y = find_y_on_seg(s, e, foot->x);
		}
	}
	else
	{
		foot->y = (dy_square * out.y + dx_square * s.y + dx * dy * (out.x - s.x)) / (dx_square + dy_square);
		if (s.y == e.y)
		{
			foot->x = out.x;
		}
		else
		{
			foot->x = find_x_on_seg(s, e, foot->y);
		}
	}

	if (is_between(s.x, e.x, foot->x) && is_between(s.y, e.y, foot->y)) {
		return 0;
	} else {
		return -1;
	}
}

inline
vec2 get_tri_gravity_center(const vec2& p0, const vec2& p1, const vec2& p2)
{
	return vec2((p0.x + p1.x + p2.x) / 3, (p0.y + p1.y + p2.y) / 3);
}

}

#endif // _SPATIAL_MATH_CALC_INL_