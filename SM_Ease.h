// https://github.com/fogleman/ease

#ifndef _SPATIAL_MATH_EASE_H_
#define _SPATIAL_MATH_EASE_H_

#include "sm_const.h"

#include <math.h>

namespace sm
{

inline
float linear(float t) 
{
	return t;
}

inline
float in_quad(float t) 
{
	return t * t;
}

inline
float out_quad(float t) 
{
	return -t * (t - 2);
}

inline
float in_out_quad(float t) 
{
	if (t < 0.5f) {
		return 2 * t * t;
	} else {
		t = 2 * t - 1;
		return -0.5f * (t*(t-2) - 1);
	}
}

inline
float in_cubic(float t) 
{
	return t * t * t;
}

inline
float out_cubic(float t) 
{
	t -= 1;
	return t*t*t + 1;
}

inline
float in_out_cubic(float t) 
{
	t *= 2;
	if (t < 1) {
		return 0.5f * t * t * t;
	} else {
		t -= 2;
		return 0.5f * (t*t*t + 2);
	}
}

inline
float in_quart(float t) 
{
	return t * t * t * t;
}

inline
float out_quart(float t) 
{
	t -= 1;
	return -(t*t*t*t - 1);
}

inline
float in_out_quart(float t) 
{
	t *= 2;
	if (t < 1) {
		return 0.5f * t * t * t * t;
	} else {
		t -= 2;
		return -0.5f * (t*t*t*t - 2);
	}
}

inline
float in_quint(float t) 
{
	return t * t * t * t * t;
}

inline
float out_quint(float t) 
{
	t -= 1;
	return t*t*t*t*t + 1;
}

inline
float in_out_quint(float t) 
{
	t *= 2;
	if (t < 1) {
		return 0.5f * t * t * t * t * t;
	} else {
		t -= 2;
		return 0.5f * (t*t*t*t*t + 2);
	}
}

inline
float in_sine(float t) 
{
	return -1 * cos(t * SM_PI / 2) + 1;
}

inline
float out_sine(float t) 
{
	return sin(t * SM_PI / 2);
}

inline
float in_out_sine(float t) 
{
	return -0.5f * (cos(SM_PI * t) - 1);
}

inline
float in_expo(float t) 
{
	if (t == 0) {
		return 0;
	} else {
		return pow(2, 10*(t-1));
	}
}

inline
float out_expo(float t) 
{
	if (t == 1) {
		return 1;
	} else {
		return 1 - pow(2, -10*t);
	}
}

inline
float in_out_expo(float t) 
{
	if (t == 0) {
		return 0;
	} else if (t == 1) {
		return 1;
	} else {
		if (t < 0.5f) {
			return 0.5f * pow(2, (20*t)-10);
		} else {
			return 1 - 0.5f * pow(2, (-20*t)+10);
		}
	}
}

inline
float in_circ(float t) 
{
	return -1 * (sqrt(1-t*t) - 1);
}

inline
float out_circ(float t) 
{
	t -= 1;
	return sqrt(1 - (t * t));
}

inline
float in_out_circ(float t) 
{
	t *= 2;
	if (t < 1) {
		return -0.5f * (sqrt(1-t*t) - 1);
	} else {
		t = t - 2;
		return 0.5f * (sqrt(1-t*t) + 1);
	}
}

inline
float in_elastic_function(float t)
{
	float p = t;
	t -= 1;
	return -1 * (pow(2, 10*t) * sin((t-p/4)*(2*SM_PI)/p));
}

inline
float out_elastic_function(float t)
{
	float p = t;
	return pow(2, -10*t) * sin((t-p/4)*(2*SM_PI/p)) + 1;
}

inline
float in_out_elastic_function(float t)
{
	float p = t;
	t *= 2;
	if (t < 1) {
		t -= 1;
		return -0.5f * (pow(2, 10*t) * sin((t-p/4)*2*SM_PI/p));
	} else {
		t -= 1;
		return pow(2, -10*t)*sin((t-p/4)*2*SM_PI/p)*0.5f + 1;
	}
}

inline
float in_elastic(float t) 
{
	return in_elastic_function(0.5f);
}

inline
float out_elastic(float t) 
{
	return out_elastic_function(0.5f);
}

inline
float in_out_elastic(float t) 
{
	return in_out_elastic_function(0.5f);
}

inline
float in_back(float t) 
{
	float s = 1.70158f;
	return t * t * ((s+1)*t - s);
}

inline
float out_back(float t) 
{
	float s = 1.70158f;
	t -= 1;
	return t*t*((s+1)*t+s) + 1;
}

inline
float in_out_back(float t) 
{
	float s = 1.70158f;
	t *= 2;
	if (t < 1) {
		s *= 1.525f;
		return 0.5f * (t * t * ((s+1)*t - s));
	} else {
		t -= 2;
		s *= 1.525f;
		return 0.5f * (t*t*((s+1)*t+s) + 2);
	}
}

inline
float out_bounce(float t) 
{
	if (t < 4/11.0f) {
		return (121 * t * t) / 16.0f;
	} else if (t < 8/11.0f) {
		return (363 / 40.0f * t * t) - (99 / 10.0f * t) + 17/5.0f;
	} else if (t < 9/10.0f) {
		return (4356 / 361.0f * t * t) - (35442 / 1805.0f * t) + 16061/1805.0f;
	} else {
		return (54 / 5.0f * t * t) - (513 / 25.0f * t) + 268/25.0f;
	}
}

inline
float in_bounce(float t) 
{
	return 1 - out_bounce(1-t);
}

inline
float in_out_bounce(float t) 
{
	if (t < 0.5f) {
		return in_bounce(2*t) * 0.5f;
	} else {
		return out_bounce(2*t-1)*0.5f + 0.5f;
	}
}

inline
float in_square(float t) 
{
	if (t < 1) {
		return 0;
	} else {
		return 1;
	}
}

inline
float out_square(float t) 
{
	if (t > 0) {
		return 1;
	} else {
		return 0;
	}
}

inline
float in_out_square(float t) 
{
	if (t < 0.5f) {
		return 0;
	} else {
		return 1;
	}
}

}

#endif // _SPATIAL_MATH_EASE_H_