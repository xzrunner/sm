#ifdef __cplusplus
extern "C"
{
#endif

#ifndef spatial_math_quaternion_h
#define spatial_math_quaternion_h

#include <math.h>

struct sm_quaternion {
	float x, y, z, w;
};

static inline struct sm_quaternion *
sm_quaternion_mul(struct sm_quaternion *q, const struct sm_quaternion *a, const struct sm_quaternion *b) {
	float x = a->y * b->z - a->z * b->y + b->x * a->w + a->x * b->w;
	float y = a->z * b->x - a->x * b->z + b->y * a->w + a->y * b->w;
	float z = a->x * b->y - a->y * b->x + b->z * a->w + a->z * b->w;
	float w = a->w * b->w - (a->x * b->x + a->y * b->y + a->z * b->z);

	q->x = x;
	q->y = y;
	q->z = z;
	q->w = w;

	return q;
}

static inline struct sm_quaternion *
sm_quaternion_init(struct sm_quaternion *q, float x, float y, float z) {
	struct sm_quaternion roll = { sinf( x * 0.5f ), 0, 0, cosf( x * 0.5f ) };
	struct sm_quaternion pitch = { 0, sinf( y * 0.5f ), 0, cosf( y * 0.5f ) };
	struct sm_quaternion yaw = { 0, 0, sinf( z * 0.5f ), cosf( z * 0.5f ) };

	// Order: y * x * z
	sm_quaternion_mul(q, &pitch, &roll);
	sm_quaternion_mul(q, q, &yaw);

	return q;
}

static inline struct sm_quaternion *
sm_quaternion_slerp(struct sm_quaternion *q, const struct sm_quaternion *a, const struct sm_quaternion *b, float t) {
	float cosTheta = a->x * b->x + a->y * b->y + a->z * b->z + a->w * b->w;
	if (cosTheta < 0) {
		cosTheta = -cosTheta; 
		q->x = -b->x; q->y = -b->y;
		q->z = -b->z; q->w = -b->w;
	} else {
		*q = *b;
	}
	float scale0 = 1 - t, scale1 = t;
	if( (1 - cosTheta) > 0.001f ) {
		// use spherical interpolation
		float theta = acosf( cosTheta );
		float sinTheta = sinf( theta );
		scale0 = sinf( (1 - t) * theta ) / sinTheta;
		scale1 = sinf( t * theta ) / sinTheta;
	}

	q->x = a->x * scale0 + q->x * scale1;
	q->y = a->y * scale0 + q->y * scale1;
	q->z = a->z * scale0 + q->z * scale1;
	q->w = a->w * scale0 + q->w * scale1;

	return q;
}

static inline struct sm_quaternion *
sm_quaternion_nslerp(struct sm_quaternion *q, const struct sm_quaternion *a, const struct sm_quaternion *b, float t) {
	// Normalized linear sm_quaternion interpolation
	// Note: NLERP is faster than SLERP and commutative but does not yield constant velocity

	float cosTheta = a->x * b->x + a->y * b->y + a->z * b->z + a->w * b->w;

	if( cosTheta < 0 ) {
		q->x = a->x + (-b->x - a->x) * t;
		q->y = a->y + (-b->y - a->y) * t;
		q->z = a->z + (-b->z - a->z) * t;
		q->w = a->w + (-b->w - a->w) * t;
	} else {
		q->x = a->x + (b->x - a->x) * t;
		q->y = a->y + (b->y - a->y) * t;
		q->z = a->z + (b->z - a->z) * t;
		q->w = a->w + (b->w - a->w) * t;
	}

	float invLen = 1.0f / sqrtf( q->x * q->x + q->y * q->y + q->z * q->z + q->w * q->w );

	q->x *= invLen;
	q->y *= invLen;
	q->z *= invLen;
	q->w *= invLen;

	return q;
}

static inline struct sm_quaternion *
sm_quaternion_inverted(struct sm_quaternion * q) {
	float len = q->x * q->x + q->y * q->y + q->z * q->z + q->w * q->w;
	if( len > 0 ) {
		float invLen = - 1.0f / len;
		q->x *= invLen;
		q->y *= invLen;
		q->z *= invLen;
		q->w *= invLen;
		q->w = -q->w;
	} else {
		q->x = q->y = q->z = q->w = 0;
	}
	return q;
}

#endif // spatial_math_quaternion_h

#ifdef __cplusplus
}
#endif