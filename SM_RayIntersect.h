#pragma once

#include "SM_Cube.h"
#include "SM_Ray.h"
#include "SM_Vector.h"
#include "SM_Quaternion.h"
#include "SM_Matrix.h"
#include "SM_Plane.h"

#include <vector>

namespace sm
{


bool ray_aabb_intersect(const cube& aabb, const Ray& ray, vec3* cross);
bool ray_obb_intersect(const cube& aabb, const vec3& pos, const Quaternion& angle, const vec3& scale, const Ray& ray, vec3* cross);

bool ray_plane_intersect(const Ray& ray, const Plane& plane, vec3* cross);

bool ray_triangle_intersect(const mat4& mat, const vec3& v0, const vec3& v1, const vec3& v2, const Ray& ray, vec3* cross);
bool ray_polygon_intersect(const mat4& mat, const std::vector<vec3>& polygon, const Ray& ray, vec3* cross);

}