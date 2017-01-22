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
	double sin;
	if (x < 0)
		sin = 1.27323954 * x + .405284735 * x * x;
	else
		sin = 1.27323954 * x - 0.405284735 * x * x;
	return static_cast<float>(sin);
}

inline
float sm_cos(float _x)
{
	//compute cosine: sin(x + PI/2) = cos(x)
	double x = _x + 1.57079632;

	double cos;
	if (x < 0)
		cos = 1.27323954 * x + 0.405284735 * x * x;
	else
		cos = 1.27323954 * x - 0.405284735 * x * x;
	return static_cast<float>(cos);
}

#endif // spatial_math_math_h

//#include "sm_math.inl"

#ifdef __cplusplus
}
#endif
