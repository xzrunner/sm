#include "SM_RayIntersect.h"

namespace sm
{

bool ray_ray_intersect(const Ray& ray0, const Ray& ray1, vec3* cross)
{
	auto& da = ray0.dir;
	auto& db = ray1.dir;
	auto  dc = ray1.origin - ray0.origin;
	auto cross_ab = da.Cross(db);
	if (fabs(dc.Dot(cross_ab)) > SM_LARGE_EPSILON) {
		return false;
	}
	float d = cross_ab.LengthSquared();
	// collinear
	if (d < std::numeric_limits<float>::epsilon()) {
		return false;
	}
	auto s = dc.Cross(db).Dot(cross_ab) / d;
	auto t = dc.Cross(da).Dot(cross_ab) / d;
	//if (s >= 0.0f && s <= 1.0f) {
	//	*cross = ray0.origin + ray0.dir * s;
	//	return true;
	//}
	if (s >= 0 && t >= 0) {
		*cross = ray0.origin + ray0.dir * s;
		return true;
	}
	return false;
}

// code from http://paulbourke.net/geometry/pointlineplane/lineline.c
bool line_line_intersect(const vec3& p1, const vec3& p2,
	                     const vec3& p3, const vec3& p4,
	                     vec3* pa, vec3* pb, float* mua, float* mub)
{
	vec3 p13,p43,p21;
	float d1343,d4321,d1321,d4343,d2121;
	float numer,denom;

	float EPS = std::numeric_limits<float>::epsilon();

	p13.x = p1.x - p3.x;
	p13.y = p1.y - p3.y;
	p13.z = p1.z - p3.z;
	p43.x = p4.x - p3.x;
	p43.y = p4.y - p3.y;
	p43.z = p4.z - p3.z;
	if (fabs(p43.x) < EPS && fabs(p43.y) < EPS && fabs(p43.z) < EPS)
		return false;
	p21.x = p2.x - p1.x;
	p21.y = p2.y - p1.y;
	p21.z = p2.z - p1.z;
	if (fabs(p21.x) < EPS && fabs(p21.y) < EPS && fabs(p21.z) < EPS)
		return false;

	d1343 = p13.x * p43.x + p13.y * p43.y + p13.z * p43.z;
	d4321 = p43.x * p21.x + p43.y * p21.y + p43.z * p21.z;
	d1321 = p13.x * p21.x + p13.y * p21.y + p13.z * p21.z;
	d4343 = p43.x * p43.x + p43.y * p43.y + p43.z * p43.z;
	d2121 = p21.x * p21.x + p21.y * p21.y + p21.z * p21.z;

	denom = d2121 * d4343 - d4321 * d4321;
	if (fabs(denom) < EPS)
		return false;
	numer = d1343 * d4321 - d1321 * d4343;

	*mua = numer / denom;
	*mub = (d1343 + d4321 * (*mua)) / d4343;

	pa->x = p1.x + *mua * p21.x;
	pa->y = p1.y + *mua * p21.y;
	pa->z = p1.z + *mua * p21.z;
	pb->x = p3.x + *mub * p43.x;
	pb->y = p3.y + *mub * p43.y;
	pb->z = p3.z + *mub * p43.z;

	return true;
}

bool ray_line_intersect(const Ray& ray, const vec3& p0, const vec3& p1, vec3* cross)
{
	auto da = ray.dir;
	auto db = p1 - p0;
	auto dc = p0 - ray.origin;
	auto cross_ab = da.Cross(db);
	float zz = fabs(dc.Dot(cross_ab));
	if (fabs(dc.Dot(cross_ab)) > 0.01f) {
		return false;
	}

	float d = cross_ab.LengthSquared();
	// collinear
	if (d < std::numeric_limits<float>::epsilon()) {
		return false;
	}
	auto t = dc.Cross(da).Dot(cross_ab) / d;
	if (t >= 0.0f && t <= 1.0f) {
		*cross = ray.origin + ray.dir * t;
		return true;
	}
	return false;
}

#define NUMDIM	3
#define RIGHT	0
#define LEFT	1
#define MIDDLE	2

// This code from GraphicsGems's RayBox.c
// https://github.com/erich666/GraphicsGems/blob/master/gems/RayBox.c
bool ray_aabb_intersect(const cube& aabb, const Ray& ray, vec3* _cross)
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

	if (_cross) {
		*_cross = cross;
	}

	return true;
}

bool ray_obb_intersect(const cube& aabb, const vec3& pos, const Quaternion& angle,
	                   const vec3& scale, const Ray& ray, vec3* cross)
{
	mat4 rot_mat(-angle);

	vec3 start = rot_mat * (ray.origin - pos);
	vec3 dir = rot_mat * ray.dir;
	Ray ray_fix(start, dir);

	auto aabb_scaled = aabb;
	aabb_scaled.Scale(scale);

	return ray_aabb_intersect(aabb_scaled, ray_fix, cross);
}

bool ray_plane_intersect(const Ray& ray, const Plane& plane, vec3* cross)
{
	float d = ray.dir.Dot(plane.normal);
	if (d < -std::numeric_limits<float>::epsilon()) {
		float dist = -(ray.origin.Dot(plane.normal) + plane.dist) / d;
		*cross = ray.origin + ray.dir * dist;
		return true;
	}
	return false;
}

bool ray_plane_intersect_both_faces(const Ray& ray, const Plane& plane, vec3* cross)
{
	float d = ray.dir.Dot(plane.normal);
	if (d < -std::numeric_limits<float>::epsilon() ||
		d > std::numeric_limits<float>::epsilon()) {
		float dist = -(ray.origin.Dot(plane.normal) + plane.dist) / d;
		*cross = ray.origin + ray.dir * dist;
		return true;
	}
	return false;
}

// Code from glm/gtx/intersect.h intersectRayTriangle
bool ray_triangle_intersect(const mat4& mat, const vec3& v0, const vec3& v1,
	                        const vec3& v2, const Ray& ray, vec3* cross)
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

	vec2 bary;

	float f = 1.0f / a;

	auto s = ray.origin - _v0;
	bary.x = f * s.Dot(p);
	if (bary.x < 0.0f) {
		return false;
	}
	if (bary.x > 1.0f) {
		return false;
	}

	auto q = s.Cross(e1);
	bary.y = f * ray.dir.Dot(q);
	if (bary.y < 0.0f) {
		return false;
	}
	if (bary.y + bary.x > 1.0f) {
		return false;
	}

	*cross = _v0 + e1 * bary.x + e2 * bary.y;

	return f * e2.Dot(q) >= 0.0f;
}

// Code from glm/gtx/intersect.h intersectRayTriangle
bool ray_triangle_intersect_both_faces(const mat4& mat, const vec3& v0, const vec3& v1,
	                                   const vec3& v2, const Ray& ray, vec3* cross)
{
	auto _v0 = mat * v0;
	auto _v1 = mat * v1;
	auto _v2 = mat * v2;

	auto e1 = _v1 - _v0;
	auto e2 = _v2 - _v0;

	auto p = ray.dir.Cross(e2);;
	auto a = e1.Dot(p);

	vec2 bary;

	float f = 1.0f / a;

	auto s = ray.origin - _v0;
	bary.x = f * s.Dot(p);
	if (bary.x < 0.0f) {
		return false;
	}
	if (bary.x > 1.0f) {
		return false;
	}

	auto q = s.Cross(e1);
	bary.y = f * ray.dir.Dot(q);
	if (bary.y < 0.0f) {
		return false;
	}
	if (bary.y + bary.x > 1.0f) {
		return false;
	}

	*cross = _v0 + e1 * bary.x + e2 * bary.y;

	return f * e2.Dot(q) >= 0.0f;
}

bool ray_polygon_intersect(const mat4& mat, const vec3* polygon, size_t polygon_n, const Ray& ray, vec3* cross)
{
	for (size_t i = 1; i < polygon_n - 1; ++i) {
		if (ray_triangle_intersect(mat, polygon[0], polygon[i], polygon[i + 1], ray, cross)) {
			return true;
		}
	}
	return false;
}

bool ray_polygon_intersect_both_faces(const mat4& mat, const vec3* polygon, size_t polygon_n, const Ray& ray, vec3* cross)
{
	for (size_t i = 1; i < polygon_n - 1; ++i) {
		if (ray_triangle_intersect_both_faces(mat, polygon[0], polygon[i], polygon[i + 1], ray, cross)) {
			return true;
		}
	}
	return false;
}

}