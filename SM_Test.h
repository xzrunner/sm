#ifndef _SPATIAL_MATH_TEST_H_
#define _SPATIAL_MATH_TEST_H_

#include "SM_Vector.h"
#include "SM_Rect.h"

#include <vector>

namespace sm
{

/**
 *  @brief
 *    float
 */

/**
 *  @brief
 *    To check if test at middle of bound0 and bound1
 */
bool is_between(float bound0, float bound1, float test);

/**
 *  @brief
 *    point
 */

/**
 *  @note
 *    It can't handle the point on segment.
 *	  Before use it must make sure the point is not on the segment.
 */
bool is_point_at_line_left(const vec2& v, const vec2& s, const vec2& e);
bool is_point_in_rect(const vec2& v, const rect& r);
bool is_point_on_rect(const vec2& v, const rect& r);
bool is_point_in_area(const vec2& v, const std::vector<vec2>& area);
bool is_point_in_circle(const vec2& v, const vec2& center, float radius);
bool is_point_in_convex(const vec2& pos, const std::vector<vec2>& convex);
bool is_point_in_convex(const vec2& pos, const vec2* convex, size_t num);
bool is_point_intersect_polyline(const vec2& point, const std::vector<vec2>& polyline);

/**
 *  @brief
 *    segment
 */
bool is_segment_intersect_segment(const vec2& s0, const vec2& e0, const vec2& s1, const vec2& e1);
bool is_segment_intersect_polyline(const vec2& s, const vec2& e, const std::vector<vec2>& poly);

bool is_two_line_parallel(const vec2& s0, const vec2& e0, const vec2& s1, const vec2& e1);

/**
 *  @brief
 *    rect
 */
bool is_rect_contain_point(const rect& r, const vec2& v);
bool is_rect_contain_rect(const rect& outer, const rect& inner);
bool is_rect_intersect_rect(const rect& r0, const rect& r1);
bool is_rect_intersect_segment(const rect& r, const vec2& s, const vec2& e);
bool is_rect_intersect_polyline(const rect& r, const std::vector<vec2>& poly, bool loop);
bool is_rect_intersect_polygon(const rect& r, const std::vector<vec2>& poly);

/**
 *  @brief
 *    convex
 */
bool is_polygon_convex(const std::vector<vec2>& poly);
bool is_convex_intersect_convex(const std::vector<vec2>& c0, const std::vector<vec2>& c1);

/**
 *  @brief
 *    polygon
 */
bool is_polygon_intersect_polygon(const std::vector<vec2>& poly0, const std::vector<vec2>& poly1);
bool is_polygon_in_polygon(const std::vector<vec2>& in, const std::vector<vec2>& out);
bool is_polygon_clockwise(const std::vector<vec2>& poly);

}

#include "SM_Test.inl"

#endif // _SPATIAL_MATH_TEST_H_