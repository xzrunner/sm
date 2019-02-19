#pragma once

#include "SM_Cube.h"
#include "SM_Ray.h"
#include "SM_Vector.h"
#include "SM_Quaternion.h"
#include "SM_Matrix.h"
#include "SM_Plane.h"

namespace sm
{

bool ray_ray_intersect(const Ray& ray0, const Ray& ray1, vec3* cross);
bool line_line_intersect(const sm::vec3& p1, const sm::vec3& p2,
	                     const sm::vec3& p3, const sm::vec3& p4,
	                     sm::vec3* pa, sm::vec3* pb, float* mua, float* mub);

bool ray_aabb_intersect(const cube& aabb, const Ray& ray, vec3* cross);
bool ray_obb_intersect(const cube& aabb, const vec3& pos, const Quaternion& angle, const vec3& scale, const Ray& ray, vec3* cross);

bool ray_plane_intersect(const Ray& ray, const Plane& plane, vec3* cross);
bool ray_plane_intersect_both_faces(const Ray& ray, const Plane& plane, vec3* cross);

bool ray_triangle_intersect(const mat4& mat, const vec3& v0, const vec3& v1, const vec3& v2, const Ray& ray, vec3* cross);
bool ray_triangle_intersect_both_faces(const mat4& mat, const vec3& v0, const vec3& v1, const vec3& v2, const Ray& ray, vec3* cross);

bool ray_polygon_intersect(const mat4& mat, const vec3* polygon, size_t polygon_n, const Ray& ray, vec3* cross);
bool ray_polygon_intersect_both_faces(const mat4& mat, const vec3* polygon, size_t polygon_n, const Ray& ray, vec3* cross);

}