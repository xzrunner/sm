#ifndef _SPATIAL_MATH_CALC_H_
#define _SPATIAL_MATH_CALC_H_

#include "SM_Vector.h"
#include "SM_Plane.h"

#include <cu/cu_stl.h>

#include <stddef.h>

namespace sm
{

class Matrix2D;
class MatrixFix;

float find_x_on_seg(const vec2& s, const vec2& e, float y);
float find_y_on_seg(const vec2& s, const vec2& e, float x);

vec2 rotate_vector(const vec2& v, float rad);
vec2 rotate_vector_right_angle(const vec2& v, bool turn_left);

float mat_trans_len(float len, const Matrix2D& mat);
float mat_trans_len(float len, const MatrixFix& mat);

float get_line_angle(const vec2& s, const vec2& e);
float get_angle(const vec2& center, const vec2& pa, const vec2& pb);
float get_angle(const vec3& center, const vec3& pa, const vec3& pb);
float get_angle_in_direction(const vec2& center, const vec2& start, const vec2& end);

/**
 *  @brief
 *    To check if angle a-center-b is acute.
 */
bool is_acute_angle(const vec2& a, const vec2& center, const vec2& b);

/**
 *  @brief
 *    To check angle a-center-b turn left or right.
 */
bool is_turn_left(const vec2& a, const vec2& center, const vec2& b);
bool is_turn_right(const vec2& a, const vec2& center, const vec2& b);

/**
 *  @brief
 *    distance position to ...
 */
float dis_pos_to_pos(const vec2& v0, const vec2& v1);
float dis_square_pos_to_pos(const vec2& v0, const vec2& v1);
float dis_pos_to_multi_pos(const vec2& pos, const CU_VEC<vec2>& multi_pos, int* nearest_idx = NULL);
float dis_pos_to_seg(const vec2& v, const vec2& s0, const vec2& s1);
float dis_pos_to_polyline(const vec2& pos, const CU_VEC<vec2>& polyline, int* nearest_idx = NULL);
float dis_pos_to_polygon(const vec2& pos, const CU_VEC<vec2>& polygon, int* nearest_idx = NULL);

float dis_pos3_to_pos3(const vec3& v0, const vec3& v1);
float dis_square_pos3_to_pos3(const vec3& v0, const vec3& v1);
float distance_aabb(const vec3& pos, const vec3& aabb_min, const vec3& aabb_max);

/**
 *  @brief
 *    Get the cross point of two segment.
 *    If they are not crossed, direct return false withnot compute the cross point.
 */
bool intersect_line_line(const vec2& s0, const vec2& e0, const vec2& s1, const vec2& e1, vec2* cross);
bool intersect_segment_segment(const vec2& s0, const vec2& e0, const vec2& s1, const vec2& e1, vec2* cross);

/**
 *  @brief
 *    Get the foot of out at line(s, e).
 *    Is return -1 the foot is outside the line(s, e), return 0 the foot on the line(s, e).
 */
int get_foot_of_perpendicular(const vec2& s, const vec2& e, const vec2& out, vec2* foot);
int get_foot_of_perpendicular(const vec3& s, const vec3& e, const vec3& out, vec3* foot);

/**
 *  @brief
 *    triangle
 */
vec2 get_tri_gravity_center(const vec2& p0, const vec2& p1, const vec2& p2);

/**
*  @brief
*    area
*/
float get_polygon_area(const CU_VEC<sm::vec2>& polygon);
float get_polygon_area(const CU_VEC<sm::vec3>& polygon);
float get_triangle_area(const sm::vec2& p0, const sm::vec2& p1, const sm::vec2& p2);

sm::vec3 calc_unit_normal(const sm::vec3& a, const sm::vec3& b, const sm::vec3& c);

/**
*  @brief
*    perimeter
*/
float get_polygon_perimeter(const CU_VEC<sm::vec2>& poly);

/**
*  @brief
*    Get the cross point of three planes.
*/
bool intersect_planes(const Plane& p0, const Plane& p1, const Plane& p2, vec3* cross);

}

#include "SM_Calc.inl"

#endif // _SPATIAL_MATH_CALC_H_