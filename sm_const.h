#ifdef __cplusplus
extern "C"
{
#endif

#ifndef spatial_math_const_h
#define spatial_math_const_h

#ifdef __cplusplus
static const float SM_PI = 3.1415926f;
#else
#define SM_PI 3.1415926f
#endif // __cplusplus
static const float SM_TWO_PI = 2 * SM_PI;

static const float SM_DEG_TO_RAD = SM_PI / 180.0f;
static const float SM_RAD_TO_DEG = 180.0f / SM_PI;

static const float SM_LARGE_EPSILON = 0.0001f;

#endif // spatial_math_const_h

#ifdef __cplusplus
}
#endif