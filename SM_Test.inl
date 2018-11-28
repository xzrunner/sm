#ifndef _SPATIAL_MATH_TEST_INL_
#define _SPATIAL_MATH_TEST_INL_

//#include "SM_Calc.h"
#include "sm_const.h"

#include <float.h>

#include <algorithm>

namespace sm
{

inline
bool is_between(float bound0, float bound1, float test)
{
	if (bound0 < bound1) {
		return test < bound1 + FLT_EPSILON && test > bound0 - FLT_EPSILON;
	} else {
		return test < bound0 + FLT_EPSILON && test > bound1 - FLT_EPSILON;
	}
}

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
bool is_point_on_rect(const vec2& v, const rect& r)
{
	return v.x == r.xmin || v.y == r.xmax || v.y == r.ymin || v.y == r.ymax;
}

inline
bool is_point_in_area(const vec2& v, const CU_VEC<vec2>& area)
{
	bool odd_nodes = false;
	for (int i = 0, n = area.size(), j = n - 1; i < n; ++i)
	{
		if (((area[i].y < v.y && area[j].y >= v.y) ||
			 (area[j].y < v.y && area[i].y >= v.y)) &&
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
bool is_point_in_convex(const vec2& pos, const CU_VEC<vec2>& convex)
{
	return is_point_in_convex(pos, &convex[0], convex.size());
}

inline
bool is_point_in_convex(const vec2& pos, const vec2* convex, int num)
{
	if (num < 3) {
		return false;
	}

	int count = 0;
	for (int i = 0; i < num; ++i)
	{
		vec2 s = convex[i],
			 e = i == num - 1 ? convex[0] : convex[i + 1];
		if (is_point_at_line_left(pos, s, e)) {
			++count;
		}
	}
	return count == num || count == 0;
}

inline
bool is_point_intersect_polyline(const vec2& point, const CU_VEC<vec2>& polyline)
{
	rect r(point, SM_LARGE_EPSILON, SM_LARGE_EPSILON);
	return is_rect_intersect_polyline(r, polyline, true);
}

inline
bool is_segment_intersect_segment(const vec2& s0, const vec2& e0, const vec2& s1, const vec2& e1)
{
	return is_point_at_line_left(s0, s1, e1) != is_point_at_line_left(e0, s1, e1)
		&& is_point_at_line_left(s1, s0, e0) != is_point_at_line_left(e1, s0, e0);
}

inline
bool is_two_line_parallel(const vec2& s0, const vec2& e0, const vec2& s1, const vec2& e1)
{
	float denominatorX = (e1.y - s1.y) * (e0.x - s0.x) - (e0.y - s0.y) * (e1.x - s1.x),
		denominatorY = (e1.x - s1.x) * (e0.y - s0.y) - (e0.x - s0.x) * (e1.y - s1.y);
	return fabs(denominatorX) < FLT_EPSILON || fabs(denominatorY) < FLT_EPSILON;
}

inline
bool is_rect_contain_point(const rect& r, const vec2& v)
{
	return v.x >= r.xmin && v.x <= r.xmax
		&& v.y >= r.ymin && v.y <= r.ymax;
}

inline
bool is_rect_contain_rect(const rect& outer, const rect& inner)
{
	return inner.xmin >= outer.xmin && inner.xmax <= outer.xmax
		&& inner.ymin >= outer.ymin && inner.ymax <= outer.ymax;
}

inline
bool is_rect_intersect_rect(const rect& r0, const rect& r1)
{
	return !(r0.xmin >= r1.xmax || r0.xmax <= r1.xmin || r0.ymin >= r1.ymax || r0.ymax <= r1.ymin);
}

inline
bool is_rect_intersect_polyline(const rect& r, const CU_VEC<vec2>& poly, bool loop)
{
	if (poly.size() < 2) return false;

	for (size_t i = 0, n = poly.size() - 1; i < n; ++i)
	{
		if (is_rect_intersect_segment(r, poly[i], poly[i+1]))
			return true;
	}

	if (loop && is_rect_intersect_segment(r, poly[poly.size() - 1], poly[0]))
		return true;

	return false;
}

inline
bool is_rect_intersect_polygon(const rect& rect, const CU_VEC<vec2>& poly)
{
	if (poly.size() < 3) {
		return false;
	}

	if (is_point_in_area(rect.Center(), poly) || is_point_in_rect(poly[0], rect)) {
		return true;
	}

	CU_VEC<vec2> poly2;
	poly2.push_back(vec2(rect.xmin, rect.ymin));
	poly2.push_back(vec2(rect.xmax, rect.ymin));
	poly2.push_back(vec2(rect.xmax, rect.ymax));
	poly2.push_back(vec2(rect.xmin, rect.ymax));

	return is_polygon_intersect_polygon(poly, poly2);
}

}

#endif // _SPATIAL_MATH_TEST_INL_