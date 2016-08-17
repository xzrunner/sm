#ifndef _SPATIAL_MATH_CALC_H_
#define _SPATIAL_MATH_CALC_H_

#include "SM_Vector.h"
#include "SM_Matrix.h"

#include <vector>

#include <stddef.h>

namespace sm
{

float find_x_on_seg(const vec2& s, const vec2& e, float y);
float find_y_on_seg(const vec2& s, const vec2& e, float x);

vec2 rotate_vector(const vec2& v, float rad);
vec2 rotate_vector_right_angle(const vec2& v, bool turn_left);

float mat_trans_len(float len, const mat4& mat);

float get_line_angle(const vec2& s, const vec2& e);

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
float dis_pos_to_multi_pos(const vec2& pos, const std::vector<vec2>& multi_pos, int* nearest_idx = NULL);
float dis_pos_to_seg(const vec2& v, const vec2& s0, const vec2& s1);

float distance_aabb(const vec3& pos, const vec3& aabb_min, const vec3& aabb_max);

/**
 *  @brief
 *    Get the cross point of two segment.
 *    If they are not crossed, direct return false withnot compute the cross point.
 */
bool intersect_line_line(const vec2& s0, const vec2& e0, const vec2& s1, const vec2& e1, vec2* cross);

}

#include "SM_Calc.inl"

#endif // _SPATIAL_MATH_CALC_H_