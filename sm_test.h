#ifndef _SPATIAL_MATH_TEST_H_
#define _SPATIAL_MATH_TEST_H_

#include "SM_Vector.h"

namespace sm
{

bool intersection_ray_triangle(const vec3& ray_ori, const vec3& ray_dir,
							   const vec3& tri0, const vec3& tri1, 
							   const vec3& tri2, vec3& out);

bool intersection_ray_aabb(const vec3& ray_ori, const vec3& ray_dir, 
						   const vec3& aabb_min, const vec3& aabb_max);

float distance_aabb(const vec3& pos, const vec3& aabb_min, const vec3& aabb_max);

}

#include "SM_Test.inl"

#endif // _SPATIAL_MATH_TEST_H_