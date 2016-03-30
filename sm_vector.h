#ifdef __cplusplus
extern "C"
{
#endif

#ifndef spatial_math_vector_h
#define spatial_math_vector_h

#include <math.h>

struct sm_ivec2 {
	int x, y;
};

struct sm_vec2 {
	float x, y;
};

struct sm_vec3 {
	float x, y, z;
};

struct sm_vec4 {
	float x, y, z, w;
};


static inline float *
sm_vec3_array(struct sm_vec3 *v) {
	return (float *)v;
}

static inline float *
sm_vec4_array(struct sm_vec4 *v) {
	return (float *)v;
}

static inline float
sm_vec3_dot(const struct sm_vec3 *a, const struct sm_vec3 *b) {
	return a->x * b->x + a->y * b->y + a->z * b->z;
}

static inline struct sm_vec3 *
sm_vec3_cross(struct sm_vec3 *v, const struct sm_vec3 *a, const struct sm_vec3 *b) {
	float x = a->y * b->z - a->z * b->y;
	float y = a->z * b->x - a->x * b->z;
	float z = a->x * b->y - a->y * b->x;

	v->x = x;
	v->y = y;
	v->z = z;

	return v;
}

static inline struct sm_vec3 *
sm_vec3_vector(struct sm_vec3 *v, const struct sm_vec3 *p1, const struct sm_vec3 *p2) {
	v->x = p1->x - p2->x;
	v->y = p1->y - p2->y;
	v->z = p1->z - p2->z;

	return v;
}

static inline float
sm_vec3_length(const struct sm_vec3 *v) {
	return sqrtf(v->x * v->x + v->y * v->y + v->z * v->z );
}

static inline struct sm_vec3 *
sm_vec3_normalize(struct sm_vec3 *v) {
	float invLen = 1.0f / sm_vec3_length(v);
	v->x *= invLen;
	v->y *= invLen;
	v->z *= invLen;

	return v;
}

static inline struct sm_vec3 *
sm_vec3_to_rotation(struct sm_vec3 *v, const struct sm_vec3 *r) {
	// Assumes that the unrotated view vector is (0, 0, -1)
	v->x = v->y = v->z = 0;
	if (r->y != 0) {
		v->x = atan2f( r->y, sqrtf( r->x*r->x + r->z*r->z ) );
	}
	if (r->x != 0 || r->z != 0) {
		v->y = atan2f( -r->x, -r->z );
	}

	return v;
}

static inline struct sm_vec3 *
sm_vec3_lerp(struct sm_vec3 *v, const struct sm_vec3 *a, const struct sm_vec3 *b, float f) {
	float x = a->x + (b->x - a->x) * f;
	float y = a->y + (b->y - a->y) * f;
	float z = a->z + (b->z - a->z) * f;

	v->x = x;
	v->y = y;
	v->z = z;

	return v;
}

#endif // spatial_math_vector_h

#ifdef __cplusplus
}
#endif