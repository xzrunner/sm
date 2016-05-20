#ifndef _SPATIAL_MATH_TEST_INL_
#define _SPATIAL_MATH_TEST_INL_

//#include "SM_Calc.h"
#include "sm_const.h"

#include <float.h>

#include <algorithm>

namespace sm
{

inline
bool is_point_at_line_left(const vec2& v, const vec2& s, const vec2& e)
{
	return (v.y - s.y) * (e.x - s.x) - (v.x - s.x) * (e.y - s.y) > FLT_EPSILON;
}

inline
bool is_point_in_rect(const vec2& v, const rect& r)
{
	return v.x > r.xmin && v.x < r.xmax
		&& v.y > r.ymin && v.y < r.ymax;
}

inline
bool is_point_in_area(const vec2& v, const std::vector<vec2>& area)
{
	bool odd_nodes = false;
	for (size_t i = 0, n = area.size(), j = n - 1; i < n; ++i)
	{
		if ((area[i].y < v.y && area[j].y >= v.y ||
			 area[j].y < v.y && area[i].y >= v.y) &&
			(area[i].x <= v.x || area[j].x <= v.x))
		{
			odd_nodes ^= (area[i].x + (v.y - area[i].y) / (area[j].y - area[i].y) * (area[j].x - area[i].x) < v.x);
		}
		j = i;
	}
	return odd_nodes;
}

inline
bool is_point_in_circle(const vec2& v, const vec2& center, float radius)
{
	return (v - center).LengthSquared() < radius * radius;
}

inline
bool is_point_in_convex(const vec2& pos, const std::vector<vec2>& convex) 
{
	if (convex.size() < 3) {
		return false;
	}

	size_t count = 0;
	for (size_t i = 0, n = convex.size(); i < n; ++i)
	{
		vec2 s = convex[i], 
			     e = i == convex.size() - 1 ? convex[0] : convex[i + 1];
		if (is_point_at_line_left(pos, s, e)) {
			++count;
		}
	}
	return count == convex.size() || count == 0;
}

inline
bool is_rect_contain_point(const rect& r, const vec2& v)
{
	return v.x >= r.xmin && v.x <= r.xmax
		&& v.y >= r.ymin && v.y <= r.ymax;
}

inline
bool is_rect_contain_rect(const rect& r0, const rect& r1)
{
	return r1.xmin >= r0.xmin && r1.xmax <= r0.xmax 
		&& r1.ymin >= r0.ymin && r1.ymax <= r0.ymax;
}

inline
bool is_rect_intersect_rect(const rect& r0, const rect& r1)
{
	return !(r0.xmin >= r1.xmax || r0.xmax <= r1.xmin || r0.ymin >= r1.ymax || r0.ymax <= r1.ymin);
}

inline
bool is_ray_intersect_triangle(const vec3& ray_ori, const vec3& ray_dir, 
							   const vec3& tri0, const vec3& tri1, 
							   const vec3& tri2, vec3& out)
{
		// Idea: Tomas Moeller and Ben Trumbore
	// in Fast, Minimum Storage Ray/Triangle Intersection 
	
	// Find vectors for two edges sharing vert0
	vec3 edge1 = tri1 - tri0,
		 edge2 = tri2 - tri0;

	// Begin calculating determinant - also used to calculate U parameter
	vec3 pvec = ray_dir.Cross(edge2);

	// If determinant is near zero, ray lies in sm_plane of triangle
	float det = edge1.Dot(pvec);

	// *** Culling branch ***
	/*if (det < FLT_EPSILON)
		return NULL;

	// Calculate distance from vert0 to ray origin
	struct sm_vec3 tvec;
	sm_vec3_vector(&tvec, rayOrig, &vert0);

	// Calculate U parameter and test bounds
	float u = sm_vec3_dot(&tvec, &pvec);
	if (u < 0 || u > det) 
		return NULL;

	// Prepare to test V parameter
	struct sm_vec3 qvec;
	sm_vec3_cross(&qvec, &tvec, &edge1);

	// Calculate V parameter and test bounds
	float v = sm_vec3_dot(rayDir, &qvec);
	if (v < 0 || u + v > det) 
		return NULL;

	// Calculate t, scale parameters, ray intersects triangle
	float t = sm_vec3_dot(&edge2, &qvec) / det;*/

	// *** Non-culling branch ***
	if (det > -FLT_EPSILON && det < FLT_EPSILON) {
		return false;
	}
	float inv_det = 1.0f / det;

	// Calculate distance from vert0 to ray origin
	vec3 tvec = ray_ori - tri0;

	// Calculate U parameter and test bounds
	float u = tvec.Dot(pvec) * inv_det;
	if (u < 0.0f || u > 1.0f) {
		return false;
	}

	// Prepare to test V parameter
	vec3 qvec = tvec.Cross(tri1);

	// Calculate V parameter and test bounds
	float v = ray_dir.Dot(qvec) * inv_det;
	if (v < 0.0f || u + v > 1.0f) {
		return false;
	}

	// Calculate t, ray intersects triangle
	float t = tri2.Dot(qvec) * inv_det;

	// Calculate intersection point and test ray length and direction
	out.x = ray_ori.x + ray_dir.x * t;
	out.y = ray_ori.y + ray_dir.y * t;
	out.z = ray_ori.z + ray_dir.z * t;

	vec3 vec = out - ray_ori;
	if (vec.Dot(ray_dir) < 0 || 
		vec.Length() > ray_dir.Length()) {
		return false;
	}

	return true;
}

inline
bool is_ray_intersect_aabb(const vec3& ray_ori, const vec3& ray_dir, 
						   const vec3& aabb_min, const vec3& aabb_max)
{
	// SLAB based optimized ray/AABB intersection routine
	// Idea taken from http://ompf.org/ray/

	float l1 = (aabb_min.x - ray_ori.x) / ray_dir.x;
	float l2 = (aabb_max.x - ray_ori.x) / ray_dir.x;
	float lmin = std::min(l1, l2);
	float lmax = std::max(l1, l2);

	l1 = (aabb_min.y - ray_ori.y) / ray_dir.y;
	l2 = (aabb_max.y - ray_ori.y) / ray_dir.y;
	lmin = std::max(std::min(l1, l2), lmin);
	lmax = std::min(std::max(l1, l2), lmax);

	l1 = (aabb_min.z - ray_ori.z) / ray_dir.z;
	l2 = (aabb_max.z - ray_ori.z) / ray_dir.z;
	lmin = std::max(std::min(l1, l2), lmin);
	lmax = std::min(std::max(l1, l2), lmax);

	if ((lmax >= 0.0f) & (lmax >= lmin)) {
		// Consider length
		vec3 ray_dst(ray_ori.x + ray_dir.x , ray_ori.y + ray_dir.y , ray_ori.z + ray_dir.z),
			 ray_min(std::min(ray_dst.x, ray_ori.x), std::min(ray_dst.y, ray_ori.y), std::min(ray_dst.z, ray_ori.z)),
			 ray_max(std::max(ray_dst.x, ray_ori.x), std::max(ray_dst.y, ray_ori.y), std::max(ray_dst.z, ray_ori.z));
		return 
			(ray_min.x < aabb_max.x) && (ray_max.x > aabb_min.x) &&
			(ray_min.y < aabb_max.y) && (ray_max.y > aabb_min.y) &&
			(ray_min.z < aabb_max.z) && (ray_max.z > aabb_min.z);
	} else {
		return false;
	}
}

}

#endif // _SPATIAL_MATH_TEST_INL_