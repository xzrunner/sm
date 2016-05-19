#ifndef _SPATIAL_MATH_CALC_H_
#define _SPATIAL_MATH_CALC_H_

#include "SM_Vector.h"
#include "SM_Matrix.h"

namespace sm
{

float find_x_on_seg(const vec2& s, const vec2& e, float y);
float find_y_on_seg(const vec2& s, const vec2& e, float x);

vec2 rotate_vector(const vec2& v, float rad);

float mat_trans_len(float len, const mat4& mat);

float distance_aabb(const vec3& pos, const vec3& aabb_min, const vec3& aabb_max);

}

#include "SM_Calc.inl"

#endif // _SPATIAL_MATH_CALC_H_