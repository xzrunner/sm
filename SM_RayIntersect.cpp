#include "SM_RayIntersect.h"

namespace sm
{

#define NUMDIM	3
#define RIGHT	0
#define LEFT	1
#define MIDDLE	2

// This code from GraphicsGems's RayBox.c
// https://github.com/erich666/GraphicsGems/blob/master/gems/RayBox.c
bool ray_aabb_intersect(const cube& aabb, const Ray& ray, vec3* coord)
{
	vec3 cross;

	char quadrant[3];
	float candidate_plane[3];
	bool inside = true;

	/* Find candidate planes; this loop can be avoided if
   	rays cast all from the eye(assume perpsective view) */
	for (int i = 0; i < 3; ++i)
	{
		if (ray.origin[i] < aabb.Min()[i]) {
			quadrant[i] = LEFT;
			candidate_plane[i] = aabb.Min()[i];
			inside = false;
		} else if (ray.origin[i] > aabb.Max()[i]) {
			quadrant[i] = RIGHT;
			candidate_plane[i] = aabb.Max()[i];
			inside = false;
		} else {
			quadrant[i] = MIDDLE;
		}
	}

	/* Ray origin inside bounding box */
	if (inside) {
		cross = ray.origin;
		return true;
	}

	float max_t[3];
	/* Calculate T distances to candidate planes */
	for (int i = 0; i < 3; ++i) {
		if (quadrant[i] != MIDDLE && ray.dir[i] != 0) {
			max_t[i] = (candidate_plane[i]-ray.origin[i])/ray.dir[i];
		} else {
			max_t[i] = -1;
		}
	}

	/* Get largest of the maxT's for final choice of intersection */
	int which_plane = 0;
	for (int i = 1; i < 3; ++i) {
		if (max_t[which_plane] < max_t[i]) {
			which_plane = i;
		}
	}

	/* Check final candidate actually inside box */
	if (max_t[which_plane] < 0) {
		return false;
	}
	for (int i = 0; i < 3; ++i) {
		if (which_plane != i) {
			cross[i] = ray.origin[i] + max_t[which_plane] * ray.dir[i];
			if (cross[i] < aabb.Min()[i] || cross[i] > aabb.Max()[i]) {
				return false;
			}
		} else {
			cross[i] = candidate_plane[i];
		}
	}

	if (coord) {
		*coord = cross;
	}

	return true;
}

bool ray_obb_intersect(const cube& aabb, const vec3& pos, const Quaternion& angle,
	                   const vec3& scale, const Ray& ray, vec3* coord)
{
	mat4 rot_mat(-angle);

	vec3 start = rot_mat * (ray.origin - pos);
	vec3 dir = rot_mat * ray.dir;
	Ray ray_fix(start, dir);

	auto aabb_scaled = aabb;
	aabb_scaled.Scale(scale);

	return ray_aabb_intersect(aabb_scaled, ray_fix, coord);
}

// Code from glm/gtx/intersect.h intersectRayTriangle
bool ray_triangle_intersect(const mat4& mat, const vec3& v0, const vec3& v1,
	                        const vec3& v2, const Ray& ray, vec3* coord)
{
	auto _v0 = mat * v0;
	auto _v1 = mat * v1;
	auto _v2 = mat * v2;

	auto e1 = _v1 - _v0;
	auto e2 = _v2 - _v0;

	auto p = ray.dir.Cross(e2);;
	auto a = e1.Dot(p);

	auto epsilon = std::numeric_limits<float>::epsilon();
	if (a < epsilon) {
		return false;
	}

	float f = 1.0f / a;

	auto s = ray.origin - _v0;
	coord->x = f * s.Dot(p);
	if (coord->x < 0.0f) {
		return false;
	}
	if (coord->x > 1.0f) {
		return false;
	}

	auto q = s.Cross(e1);
	coord->y = f * ray.dir.Dot(q);
	if (coord->y < 0.0f) {
		return false;
	}
	if (coord->y + coord->x > 1.0f) {
		return false;
	}

	coord->z = f * e2.Dot(q);

	return coord->z >= 0.0f;
}

bool ray_polygon_intersect(const mat4& mat, const std::vector<vec3>& polygon, const Ray& ray, vec3* coord)
{
	for (int i = 1, n = polygon.size(); i < n - 1; ++i) {
		if (ray_triangle_intersect(mat, polygon[0], polygon[i], polygon[i + 1], ray, coord)) {
			return true;
		}
	}
	return false;
}

}