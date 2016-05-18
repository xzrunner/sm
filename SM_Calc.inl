#ifndef _SPATIAL_MATH_CALC_INL_
#define _SPATIAL_MATH_CALC_INL_

namespace sm
{

inline
float find_x_on_seg(const sm::vec2& s, const sm::vec2& e, float y)
{
	if (s.y == e.y)  {
		return (std::min)(s.x, e.x);
	} else {
		return (y - e.y) * (s.x - e.x) / (s.y - e.y) + e.x;
	}
}

inline
float find_y_on_seg(const sm::vec2& s, const sm::vec2& e, float x)
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

}

#endif // _SPATIAL_MATH_CALC_INL_