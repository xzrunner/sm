#ifdef __cplusplus
extern "C"
{
#endif

#ifndef spatial_math_plane_h
#define spatial_math_plane_h

#include "sm_vector.h"

struct sm_plane {
	struct sm_vec3 normal;
	float dist;
};

static inline struct sm_plane *
sm_plane_init(struct sm_plane *p, const struct sm_vec3 *normal, float d ) {
	p->normal = *normal;
	// normalize
	float invLen = 1.0f / sm_vec3_length(normal);
	p->normal.x *= invLen;
	p->normal.y *= invLen;
	p->normal.z *= invLen;
	p->dist = d * invLen;

	return p;
}

static inline struct sm_plane *
sm_plane_init_dot3(struct sm_plane *p, const struct sm_vec3 *v0, const struct sm_vec3 *v1, const struct sm_vec3 *v2) {
	struct sm_vec3 a,b;
	sm_vec3_vector(&a, v1, v0);
	sm_vec3_vector(&b, v2, v0);

	sm_vec3_cross(&p->normal, &a, &b);
	sm_vec3_normalize(&p->normal);
	p->dist = -sm_vec3_dot(&p->normal, v0);

	return p;
}

static inline float
sm_plane_dist(const struct sm_plane *p, const struct sm_vec3 *v) {
	float d = sm_vec3_dot(&p->normal, v);
	return d + p->dist;
}

#endif // spatial_math_plane_h

#ifdef __cplusplus
}
#endif