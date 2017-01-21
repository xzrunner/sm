#ifndef _SPATIAL_MATH_FIX_MATH_INL_
#define _SPATIAL_MATH_FIX_MATH_INL_

#include <stdio.h>

namespace sm
{

#ifdef _MSC_VER
#define roundf(num) ((num > 0.0) ? floor(num + 0.5) : ceil(num - 0.5))
#endif // _MSC_VER

inline
MatrixFix MatrixFix::operator * (const MatrixFix& b) const
{
	MatrixFix ret;
	int* m = ret.x;
	const int* m1 = x;
	const int* m2 = b.x;
	m[0] = (m1[0] * m2[0] + m1[1] * m2[2]) / SCALE;
	m[1] = (m1[0] * m2[1] + m1[1] * m2[3]) / SCALE;
	m[2] = (m1[2] * m2[0] + m1[3] * m2[2]) / SCALE;
	m[3] = (m1[2] * m2[1] + m1[3] * m2[3]) / SCALE;
	m[4] = (m1[4] * m2[0] + m1[5] * m2[2]) / SCALE + m2[4];
	m[5] = (m1[4] * m2[1] + m1[5] * m2[3]) / SCALE + m2[5];
	return ret;
}

inline
vec2 MatrixFix::operator * (const vec2& v) const
{
	vec2 ret;
	ret.x = (v.x * x[0] + v.y * x[2]) * SCALE_INV + x[4] * TRANSLATE_SCALE_INV;
	ret.y = (v.x * x[1] + v.y * x[3]) * SCALE_INV + x[5] * TRANSLATE_SCALE_INV;
	return ret;
}

inline
void MatrixFix::Identity()
{
	x[0] = SCALE;
	x[1] = 0;
	x[2] = 0;
	x[3] = SCALE;
	x[4] = 0;
	x[5] = 0;
}

}

#endif // _SPATIAL_MATH_FIX_MATH_INL_