#ifndef spatial_math_math_inl
#define spatial_math_math_inl

#include "sm_const.h"

// http://lab.polygonal.de/2007/07/18/fast-and-accurate-sinecosine-approximation/

static inline
float sin_low_precision(float x)
{
	//always wrap input angle to -PI..PI
	while (x < -SM_PI) {
		x += SM_TWO_PI;
	}
	while (x > SM_PI) {
		x -= SM_TWO_PI;
	}

	//compute sine
	if (x < 0)
		return 1.27323954 * x + .405284735 * x * x;
	else
		return 1.27323954 * x - 0.405284735 * x * x;
}

static inline
float cos_low_precision(float x)
{
	//compute cosine: sin(x + PI/2) = cos(x)
	x += 1.57079632;

	if (x < 0)
		return 1.27323954 * x + 0.405284735 * x * x;
	else
		return 1.27323954 * x - 0.405284735 * x * x;
}

static inline
float sin_high_precision(float x)
{
	//always wrap input angle to -PI..PI
	while (x < -SM_PI) {
		x += SM_TWO_PI;
	}
	while (x > SM_PI) {
		x -= SM_TWO_PI;
	}

	//compute sine
	float sin;
	if (x < 0)
	{
		sin = 1.27323954 * x + .405284735 * x * x;

		if (sin < 0)
			sin = .225 * (sin *-sin - sin) + sin;
		else
			sin = .225 * (sin * sin - sin) + sin;
	}
	else
	{
		sin = 1.27323954 * x - 0.405284735 * x * x;

		if (sin < 0)
			sin = .225 * (sin *-sin - sin) + sin;
		else
			sin = .225 * (sin * sin - sin) + sin;
	}
	return sin;
}

static inline
float cos_high_precision(float x)
{
	//compute cosine: sin(x + PI/2) = cos(x)
	x += 1.57079632;

	float cos;
	if (x < 0)
	{
		cos = 1.27323954 * x + 0.405284735 * x * x;

		if (cos < 0)
			cos = .225 * (cos *-cos - cos) + cos;
		else
			cos = .225 * (cos * cos - cos) + cos;
	}
	else
	{
		cos = 1.27323954 * x - 0.405284735 * x * x;

		if (cos < 0)
			cos = .225 * (cos *-cos - cos) + cos;
		else
			cos = .225 * (cos * cos - cos) + cos;
	}
	return cos;
}

//inline
//float sm_sin(float x)
//{
//	return sin_low_precision(x);
////	return sin_high_precision(x);
//}
//
//inline
//float sm_cos(float x)
//{
//	return cos_low_precision(x);
////	return cos_high_precision(x);
//}

#endif // spatial_math_math_inl