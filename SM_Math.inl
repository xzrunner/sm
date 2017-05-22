// http://lab.polygonal.de/2007/07/18/fast-and-accurate-sinecosine-approximation/

#ifndef _SPATIAL_MATH_MATH_INL_
#define _SPATIAL_MATH_MATH_INL_

#include "sm_const.h"

namespace sm
{

static inline 
float prepare_angle(float x) {
	//always wrap input angle to -PI..PI
	while (x < -SM_PI) {
		x += SM_TWO_PI;
	}
	while (x > SM_PI) {
		x -= SM_TWO_PI;
	}
	return x;
}

static inline
float sin_low_precision(float x)
{
	x = prepare_angle(x);

	double sin;
	if (x < 0) {
		sin = 1.27323954 * x + .405284735 * x * x;
	} else {
		sin = 1.27323954 * x - 0.405284735 * x * x;
	}
	return static_cast<float>(sin);
}

static inline
float cos_low_precision(float x)
{
	//compute cosine: sin(x + PI/2) = cos(x)
	x += 1.57079632f;

	x = prepare_angle(x);

	double cos;
	if (x < 0) {
		cos = 1.27323954 * x + 0.405284735 * x * x;
	} else {
		cos = 1.27323954 * x - 0.405284735 * x * x;
	}
	return static_cast<float>(cos);
}

static inline
float sin_high_precision(float x)
{
	x = prepare_angle(x);

	double sin;
	if (x < 0)
	{
		sin = 1.27323954 * x + .405284735 * x * x;
		if (sin < 0) {
			sin = .225 * (sin *-sin - sin) + sin;
		} else {
			sin = .225 * (sin * sin - sin) + sin;
		}
	}
	else
	{
		sin = 1.27323954 * x - 0.405284735 * x * x;
		if (sin < 0) {
			sin = .225 * (sin *-sin - sin) + sin;
		} else {
			sin = .225 * (sin * sin - sin) + sin;
		}
	}
	return static_cast<float>(sin);
}

static inline
float cos_high_precision(float x)
{
	//compute cosine: sin(x + PI/2) = cos(x)
	x += 1.57079632f;

	x = prepare_angle(x);

	double cos;
	if (x < 0)
	{
		cos = 1.27323954 * x + 0.405284735 * x * x;
		if (cos < 0) {
			cos = .225 * (cos *-cos - cos) + cos;
		} else {
			cos = .225 * (cos * cos - cos) + cos;
		}
	}
	else
	{
		cos = 1.27323954 * x - 0.405284735 * x * x;
		if (cos < 0) {
			cos = .225 * (cos *-cos - cos) + cos;
		} else {
			cos = .225 * (cos * cos - cos) + cos;
		}
	}
	return static_cast<float>(cos);
}

inline
float sin(float x)
{
#ifdef SM_SIN_COS_LOW_PRECISION
	return sin_low_precision(x);
#elif defined SM_SIN_COS_HIGH_PRECISION
	return sin_high_precision(x);
#else
	return sinf(x);
#endif
}

inline
float cos(float x)
{
#ifdef SM_SIN_COS_LOW_PRECISION
	return cos_low_precision(x);
#elif defined SM_SIN_COS_HIGH_PRECISION
	return cos_high_precision(x);
#else
	return cosf(x);
#endif
}

inline
float sin_fast(float x)
{
	return sin_low_precision(x);
}

inline
float cos_fast(float x)
{
	return cos_low_precision(x);
}

}

#endif // _SPATIAL_MATH_MATH_INL_