#ifdef __cplusplus
extern "C"
{
#endif

#ifndef spatial_math_math_h
#define spatial_math_math_h

#include "sm_const.h"

inline
float sm_sin(float x)
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

inline
float sm_cos(float x)
{
	//compute cosine: sin(x + PI/2) = cos(x)
	x += 1.57079632;

	if (x < 0)
		return 1.27323954 * x + 0.405284735 * x * x;
	else
		return 1.27323954 * x - 0.405284735 * x * x;
}

#endif // spatial_math_math_h

//#include "sm_math.inl"

#ifdef __cplusplus
}
#endif
