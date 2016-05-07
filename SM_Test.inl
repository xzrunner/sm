#ifndef _SPATIAL_MATH_TEST_INL_
#define _SPATIAL_MATH_TEST_INL_

#include <float.h>

#include <algorithm>

namespace sm
{

bool intersection_ray_triangle(const vec3& ray_ori, const vec3& ray_dir, 
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
	/*if( det < FLT_EPSILON )
		return NULL;

	// Calculate distance from vert0 to ray origin
	struct sm_vec3 tvec;
	sm_vec3_vector(&tvec, rayOrig, &vert0);

	// Calculate U parameter and test bounds
	float u = sm_vec3_dot(&tvec, &pvec);
	if (u < 0 || u > det ) 
		return NULL;

	// Prepare to test V parameter
	struct sm_vec3 qvec;
	sm_vec3_cross(&qvec, &tvec, &edge1);

	// Calculate V parameter and test bounds
	float v = sm_vec3_dot(rayDir, &qvec);
	if (v < 0 || u + v > det ) 
		return NULL;

	// Calculate t, scale parameters, ray intersects triangle
	float t = sm_vec3_dot(&edge2, &qvec ) / det;*/

	// *** Non-culling branch ***
	if( det > -FLT_EPSILON && det < FLT_EPSILON ) {
		return false;
	}
	float inv_det = 1.0f / det;

	// Calculate distance from vert0 to ray origin
	vec3 tvec = ray_ori - tri0;

	// Calculate U parameter and test bounds
	float u = tvec.Dot(pvec) * inv_det;
	if( u < 0.0f || u > 1.0f ) {
		return false;
	}

	// Prepare to test V parameter
	vec3 qvec = tvec.Cross(tri1);

	// Calculate V parameter and test bounds
	float v = ray_dir.Dot(qvec) * inv_det;
	if( v < 0.0f || u + v > 1.0f ) {
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

bool intersection_ray_aabb(const vec3& ray_ori, const vec3& ray_dir, 
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
	lmin = std::max(std::min( l1, l2 ), lmin);
	lmax = std::min(std::max( l1, l2 ), lmax);

	l1 = (aabb_min.z - ray_ori.z) / ray_dir.z;
	l2 = (aabb_max.z - ray_ori.z) / ray_dir.z;
	lmin = std::max(std::min( l1, l2 ), lmin);
	lmax = std::min(std::max( l1, l2 ), lmax);

	if( (lmax >= 0.0f) & (lmax >= lmin) ) {
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

#endif // _SPATIAL_MATH_TEST_INL_