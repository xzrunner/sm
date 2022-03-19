#ifndef _SPATIAL_MATH_MATH_H_
#define _SPATIAL_MATH_MATH_H_

namespace sm
{

float sin(float x);
float cos(float x);

float sin_fast(float x);
float cos_fast(float x);

int next_p2(int a);
bool is_power_of_two(int x);

float f16_to_f32(unsigned short value);

}

#include "SM_Math.inl"

#endif // _SPATIAL_MATH_MATH_H_