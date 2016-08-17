#include "SM_Test.h"
#include "SM_Calc.h"

namespace sm
{

bool is_rect_intersect_segment(const rect& r, const vec2& s, const vec2& e)
{
	unsigned char type_s, type_e;
	type_s = type_e = 0;
	if (s.x < r.xmin)      // left:  1000
		type_s |= 0x8; 
	else if (s.x > r.xmax) // right: 0100
		type_s |= 0x4; 
	if (s.y < r.ymin)      // down:  0001
		type_s |= 0x1; 
	else if (s.y > r.ymax) // up:    0010
		type_s |= 0x2; 

	if (e.x < r.xmin)      // left:  1000
		type_e |= 0x8; 
	else if (e.x > r.xmax) // right: 0100
		type_e |= 0x4; 
	if (e.y < r.ymin)      // down:  0001
		type_e |= 0x1; 
	else if (e.y > r.ymax) // up:    0010
		type_e |= 0x2; 

	unsigned char comp;
	comp = type_s & type_e;
	if (comp != 0)		// must be outside, so must intersect
		return false;
	comp = type_s | type_e;
	if (comp == 0)		// must be inside, so must not intersect
		return true;

	// test each edge
	if (comp & 0x8)		// 1000, left edge
	{
		float cross_y;
		cross_y = find_y_on_seg(s, e, r.xmin);
		if (cross_y >= r.ymin && cross_y <= r.ymax)
			return true;
	}
	if (comp & 0x4)		// 0100, right edge
	{
		float cross_y;
		cross_y = find_y_on_seg(s, e, r.xmax);
		if (cross_y >= r.ymin && cross_y <= r.ymax)
			return true;
	}
	if (comp & 0x1)		// 0001, down edge
	{
		float cross_x;
		cross_x = find_x_on_seg(s, e, r.ymin);
		if (cross_x >= r.xmin && cross_x <= r.xmax)
			return true;
	}
	if (comp & 0x2)		// 0010, up edge
	{
		float cross_x;
		cross_x = find_x_on_seg(s, e, r.ymax);
		if (cross_x >= r.xmin && cross_x <= r.xmax)
			return true;
	}

	return false;
}

static inline
void project_convex(const std::vector<vec2>& c, float angle, float* min, float* max)
{
	*min = FLT_MAX;
	*max = -FLT_MAX;
	for (int i = 0, n = c.size(); i < n; ++i) {
		vec2 v = rotate_vector(c[i], angle);
		if (v.x < *min) {
			*min = v.x;
		}
		if (v.x > *max) {
			*max = v.x;
		}
	}
}

static inline
bool is_project_intersect(float min0, float max0, float min1, float max1)
{
	return !(max1 <= min0 || min1 >= max0);
}

static inline
bool is_convex_intersect_convex(const std::vector<vec2>& c0, const std::vector<vec2>& c1, float angle)
{
	float min0, max0, min1, max1;
	project_convex(c0, angle, &min0, &max0);
	project_convex(c1, angle, &min1, &max1);
	return is_project_intersect(min0, max0, min1, max1);
}

static inline
bool is_convex_intersect_convex(const std::vector<vec2>& c0, const std::vector<vec2>& c1, const vec2& v0, const vec2& v1)
{
	float angle = SM_PI * 0.5f - atan2(v1.y - v0.y, v1.x - v0.x);
	return is_convex_intersect_convex(c0, c1, angle);
}

static inline
bool is_convex_intersect_convex(const std::vector<vec2>& c0, const std::vector<vec2>& c1, const std::vector<vec2>& proj)
{
	for (int i = 0, n = c0.size() - 1; i < n; ++i) {
		if (!is_convex_intersect_convex(c0, c1, proj[i], proj[i+1])) {
			return false;
		}				
	}
	if (!is_convex_intersect_convex(c0, c1, proj[c0.size() - 1], proj[0])) {
		return false;
	}
	return true;
}

bool is_convex_intersect_convex(const std::vector<vec2>& c0, const std::vector<vec2>& c1)
{
	if (c0.size() < 3 || c1.size() < 3) {
		return false;
	}
	if (!is_convex_intersect_convex(c0, c1, c0) ||
		!is_convex_intersect_convex(c0, c1, c1)) {
		return false;
	} else {
		return true;
	}
}

}