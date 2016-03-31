#ifdef __cplusplus
extern "C"
{
#endif

#ifndef spatial_math_matrix_h
#define spatial_math_matrix_h

#include "sm_quaternion.h"
#include "sm_vector.h"

union sm_mat3 {
	float c[3][3];
	float x[9];
};

union sm_mat4 {
	float c[4][4];
	float x[16];
};

#define C m->c

static inline union sm_mat4 *
sm_mat4_identity(union sm_mat4 * m) {
	C[0][0] = 1; C[1][0] = 0; C[2][0] = 0; C[3][0] = 0;
	C[0][1] = 0; C[1][1] = 1; C[2][1] = 0; C[3][1] = 0;
	C[0][2] = 0; C[1][2] = 0; C[2][2] = 1; C[3][2] = 0;
	C[0][3] = 0; C[1][3] = 0; C[2][3] = 0; C[3][3] = 1;

	return m;
}

static inline union sm_mat4 *
sm_mat4_from_quaternion(union sm_mat4 *m, const struct sm_quaternion *q) {
	// Calculate coefficients
	float x2 = q->x + q->x, y2 = q->y + q->y, z2 = q->z + q->z;
	float xx = q->x * x2,  xy = q->x * y2,  xz = q->x * z2;
	float yy = q->y * y2,  yz = q->y * z2,  zz = q->z * z2;
	float wx = q->w * x2,  wy = q->w * y2,  wz = q->w * z2;

	C[0][0] = 1 - (yy + zz);  C[1][0] = xy - wz;	
	C[2][0] = xz + wy;        C[3][0] = 0;
	C[0][1] = xy + wz;        C[1][1] = 1 - (xx + zz);
	C[2][1] = yz - wx;        C[3][1] = 0;
	C[0][2] = xz - wy;        C[1][2] = yz + wx;
	C[2][2] = 1 - (xx + yy);  C[3][2] = 0;
	C[0][3] = 0;              C[1][3] = 0;
	C[2][3] = 0;              C[3][3] = 1;

	return m;
}

static inline union sm_mat4 *
sm_mat4_transmat(union sm_mat4 *m, float x, float y, float z) {
	sm_mat4_identity(m);
	C[3][0] = x;
	C[3][1] = y;
	C[3][2] = z;

	return m;
}

static inline union sm_mat4 *
sm_mat4_trans(union sm_mat4 *m, float x, float y, float z) {
	C[3][0] += x;
	C[3][1] += y;
	C[3][2] += z;

	return m;
}

static inline union sm_mat4 *
sm_mat4_scalemat(union sm_mat4 *m, float x, float y, float z) {
	sm_mat4_identity(m);
	C[0][0] = x;
	C[1][1] = y;
	C[2][2] = z;

	return m;
}

static inline union sm_mat4 *
sm_mat4_scale(union sm_mat4 *m, float x, float y, float z) {
	C[0][0] *= x;
	C[0][1] *= y;
	C[0][2] *= z;

	C[1][0] *= x;
	C[1][1] *= y;
	C[1][2] *= z;

	C[2][0] *= x;
	C[2][1] *= y;
	C[2][2] *= z;

	return m;
}

static inline union sm_mat4 *
sm_mat4_rotmat(union sm_mat4 *m, float x, float y, float z) {
	// Rotation order: YXZ [* Vector]
	struct sm_quaternion q;
	sm_quaternion_init(&q, x, y, z);

	return sm_mat4_from_quaternion(m, &q);
}

static inline union sm_mat4 *
sm_mat4_rotxmat(union sm_mat4* mat, float degrees) {
	float radians = degrees * 3.14159f / 180.0f;
	float s = sin(radians);
	float c = cos(radians);
	mat->c[0][0] = 1; mat->c[0][1] = 0; mat->c[0][2] = 0; mat->c[0][3] = 0;
	mat->c[1][0] = 0; mat->c[1][1] = c; mat->c[1][2] = s; mat->c[1][3] = 0;
	mat->c[2][0] = 0; mat->c[2][1] =-s; mat->c[2][2] = c; mat->c[2][3] = 0;
	mat->c[3][0] = 0; mat->c[3][1] = 0; mat->c[3][2] = 0; mat->c[3][3] = 1;
    return mat;
}

static inline union sm_mat4 *
sm_mat4_rot_axis(union sm_mat4 *m, const struct sm_vec3 *axis, float angle) {
	float t = sinf( angle * 0.5f);
	float x = axis->x * t;
	float y = axis->y * t;
	float z = axis->z * t;
	struct sm_quaternion q = {	x,y,z, cosf( angle * 0.5f ) };

	return sm_mat4_from_quaternion(m, &q);
}

static inline union sm_mat4 *
sm_mat4_perspective(union sm_mat4 *m, float l, float r, float b, float t, float n, float f) {
	sm_mat4_identity(m);
	float *mx = m->x;

	mx[0] = 2 * n / (r - l);
	mx[5] = 2 * n / (t - b);
	mx[8] = (r + l) / (r - l);
	mx[9] = (t + b) / (t - b);
	mx[10] = -(f + n) / (f - n);
	mx[11] = -1;
	mx[14] = -2 * f * n / (f - n);
	mx[15] = 0;

	return m;
}

static inline union sm_mat4 *
sm_mat4_ortho(union sm_mat4 *m, float l, float r, float b, float t, float n, float f ) {
	sm_mat4_identity(m);
	float *mx = m->x;

	mx[0] = 2 / (r - l);
	mx[5] = 2 / (t - b);
	mx[10] = -2 / (f - n);
	mx[12] = -(r + l) / (r - l);
	mx[13] = -(t + b) / (t - b);
	mx[14] = -(f + n) / (f - n);

	return m;
}

static inline union sm_mat4 *
sm_mat4_fastmul43(union sm_mat4 *m, const union sm_mat4 *m1, const union sm_mat4 *m2) {
	// Note: m may not be the same as m1 or m2

	const float *m1x = m1->x;
	const float *m2x = m2->x;
	float *mx = m->x;

	mx[0] = m1x[0] * m2x[0] + m1x[4] * m2x[1] + m1x[8] * m2x[2];
	mx[1] = m1x[1] * m2x[0] + m1x[5] * m2x[1] + m1x[9] * m2x[2];
	mx[2] = m1x[2] * m2x[0] + m1x[6] * m2x[1] + m1x[10] * m2x[2];
	mx[3] = 0.0f;

	mx[4] = m1x[0] * m2x[4] + m1x[4] * m2x[5] + m1x[8] * m2x[6];
	mx[5] = m1x[1] * m2x[4] + m1x[5] * m2x[5] + m1x[9] * m2x[6];
	mx[6] = m1x[2] * m2x[4] + m1x[6] * m2x[5] + m1x[10] * m2x[6];
	mx[7] = 0.0f;

	mx[8] = m1x[0] * m2x[8] + m1x[4] * m2x[9] + m1x[8] * m2x[10];
	mx[9] = m1x[1] * m2x[8] + m1x[5] * m2x[9] + m1x[9] * m2x[10];
	mx[10] = m1x[2] * m2x[8] + m1x[6] * m2x[9] + m1x[10] * m2x[10];
	mx[11] = 0.0f;

	mx[12] = m1x[0] * m2x[12] + m1x[4] * m2x[13] + m1x[8] * m2x[14] + m1x[12] * m2x[15];
	mx[13] = m1x[1] * m2x[12] + m1x[5] * m2x[13] + m1x[9] * m2x[14] + m1x[13] * m2x[15];
	mx[14] = m1x[2] * m2x[12] + m1x[6] * m2x[13] + m1x[10] * m2x[14] + m1x[14] * m2x[15];
	mx[15] = 1.0f;

	return m;
}

static inline union sm_mat4 *
sm_mat4_mul(union sm_mat4 *m, const union sm_mat4 *m1, const union sm_mat4 *m2) {
	union sm_mat4 mf;
	const float *m1x = m1->x;
	const float *m2x = m2->x;

	mf.x[0] = m1x[0] * m2x[0] + m1x[4] * m2x[1] + m1x[8] * m2x[2] + m1x[12] * m2x[3];
	mf.x[1] = m1x[1] * m2x[0] + m1x[5] * m2x[1] + m1x[9] * m2x[2] + m1x[13] * m2x[3];
	mf.x[2] = m1x[2] * m2x[0] + m1x[6] * m2x[1] + m1x[10] * m2x[2] + m1x[14] * m2x[3];
	mf.x[3] = m1x[3] * m2x[0] + m1x[7] * m2x[1] + m1x[11] * m2x[2] + m1x[15] * m2x[3];

	mf.x[4] = m1x[0] * m2x[4] + m1x[4] * m2x[5] + m1x[8] * m2x[6] + m1x[12] * m2x[7];
	mf.x[5] = m1x[1] * m2x[4] + m1x[5] * m2x[5] + m1x[9] * m2x[6] + m1x[13] * m2x[7];
	mf.x[6] = m1x[2] * m2x[4] + m1x[6] * m2x[5] + m1x[10] * m2x[6] + m1x[14] * m2x[7];
	mf.x[7] = m1x[3] * m2x[4] + m1x[7] * m2x[5] + m1x[11] * m2x[6] + m1x[15] * m2x[7];

	mf.x[8] = m1x[0] * m2x[8] + m1x[4] * m2x[9] + m1x[8] * m2x[10] + m1x[12] * m2x[11];
	mf.x[9] = m1x[1] * m2x[8] + m1x[5] * m2x[9] + m1x[9] * m2x[10] + m1x[13] * m2x[11];
	mf.x[10] = m1x[2] * m2x[8] + m1x[6] * m2x[9] + m1x[10] * m2x[10] + m1x[14] * m2x[11];
	mf.x[11] = m1x[3] * m2x[8] + m1x[7] * m2x[9] + m1x[11] * m2x[10] + m1x[15] * m2x[11];

	mf.x[12] = m1x[0] * m2x[12] + m1x[4] * m2x[13] + m1x[8] * m2x[14] + m1x[12] * m2x[15];
	mf.x[13] = m1x[1] * m2x[12] + m1x[5] * m2x[13] + m1x[9] * m2x[14] + m1x[13] * m2x[15];
	mf.x[14] = m1x[2] * m2x[12] + m1x[6] * m2x[13] + m1x[10] * m2x[14] + m1x[14] * m2x[15];
	mf.x[15] = m1x[3] * m2x[12] + m1x[7] * m2x[13] + m1x[11] * m2x[14] + m1x[15] * m2x[15];

	*m = mf;

	return m;
}

static inline union sm_mat4 *
sm_mat4_rot(union sm_mat4 *m, float x, float y, float z) {
	// Rotation order: YXZ [* Vector]
	struct sm_quaternion q;
	sm_quaternion_init(&q, x, y, z);

	union sm_mat4 tmp;
	sm_mat4_from_quaternion(&tmp, &q);
	return sm_mat4_mul(m, &tmp, m);
}

// vector * matrix

static inline struct sm_vec3 *
sm_vec3_mul(struct sm_vec3 *v, const union sm_mat4 *m) {
	float x = v->x * C[0][0] + v->y * C[1][0] + v->z * C[2][0] + C[3][0];
	float y = v->x * C[0][1] + v->y * C[1][1] + v->z * C[2][1] + C[3][1];
	float z = v->x * C[0][2] + v->y * C[1][2] + v->z * C[2][2] + C[3][2];

	v->x = x;
	v->y = y;
	v->z = z;

	return v;
}

static inline struct sm_vec4 *
sm_vec4_mul(struct sm_vec4 *v, const union sm_mat4 *m) {
	float x = v->x * C[0][0] + v->y * C[1][0] + v->z * C[2][0] + v->w * C[3][0];
	float y = v->x * C[0][1] + v->y * C[1][1] + v->z * C[2][1] + v->w * C[3][1];
	float z = v->x * C[0][2] + v->y * C[1][2] + v->z * C[2][2] + v->w * C[3][2];
	float w = v->x * C[0][3] + v->y * C[1][3] + v->z * C[2][3] + v->w * C[3][3];

	v->x = x;
	v->y = y;
	v->z = z;
	v->w = w;
	return v;
}

static inline struct sm_vec3 *
sm_vec3_mul33(struct sm_vec3 *v, const union sm_mat4 *m) {
	float x = v->x * C[0][0] + v->y * C[1][0] + v->z * C[2][0];
	float y = v->x * C[0][1] + v->y * C[1][1] + v->z * C[2][1];
	float z = v->x * C[0][2] + v->y * C[1][2] + v->z * C[2][2];

	v->x = x;
	v->y = y;
	v->z = z;

	return v;
}

static inline union sm_mat4 *
sm_mat4_transposed(union sm_mat4 *m) {
	int x,y;
	for (y = 0; y < 4; ++y ) {
		for(x = y + 1; x < 4; ++x ) {
			float tmp = C[x][y];
			C[x][y] = C[y][x];
			C[y][x] = tmp;
		}
	}

	return m;
}

static inline float 
sm_mat4_determinant(const union sm_mat4 *m) {
	return 
		C[0][3]*C[1][2]*C[2][1]*C[3][0] - C[0][2]*C[1][3]*C[2][1]*C[3][0] - C[0][3]*C[1][1]*C[2][2]*C[3][0] + C[0][1]*C[1][3]*C[2][2]*C[3][0] +
		C[0][2]*C[1][1]*C[2][3]*C[3][0] - C[0][1]*C[1][2]*C[2][3]*C[3][0] - C[0][3]*C[1][2]*C[2][0]*C[3][1] + C[0][2]*C[1][3]*C[2][0]*C[3][1] +
		C[0][3]*C[1][0]*C[2][2]*C[3][1] - C[0][0]*C[1][3]*C[2][2]*C[3][1] - C[0][2]*C[1][0]*C[2][3]*C[3][1] + C[0][0]*C[1][2]*C[2][3]*C[3][1] +
		C[0][3]*C[1][1]*C[2][0]*C[3][2] - C[0][1]*C[1][3]*C[2][0]*C[3][2] - C[0][3]*C[1][0]*C[2][1]*C[3][2] + C[0][0]*C[1][3]*C[2][1]*C[3][2] +
		C[0][1]*C[1][0]*C[2][3]*C[3][2] - C[0][0]*C[1][1]*C[2][3]*C[3][2] - C[0][2]*C[1][1]*C[2][0]*C[3][3] + C[0][1]*C[1][2]*C[2][0]*C[3][3] +
		C[0][2]*C[1][0]*C[2][1]*C[3][3] - C[0][0]*C[1][2]*C[2][1]*C[3][3] - C[0][1]*C[1][0]*C[2][2]*C[3][3] + C[0][0]*C[1][1]*C[2][2]*C[3][3];
}

static inline union sm_mat4 *
sm_mat4_inverted(union sm_mat4 *dst, const union sm_mat4 *m) {
	float d = sm_mat4_determinant(m);
	if( d == 0 ) {
		*dst = *m;
		return dst;
	}
	d = 1.0f / d;

	dst->c[0][0] = d * (C[1][2]*C[2][3]*C[3][1] - C[1][3]*C[2][2]*C[3][1] + C[1][3]*C[2][1]*C[3][2] - C[1][1]*C[2][3]*C[3][2] - C[1][2]*C[2][1]*C[3][3] + C[1][1]*C[2][2]*C[3][3]);
	dst->c[0][1] = d * (C[0][3]*C[2][2]*C[3][1] - C[0][2]*C[2][3]*C[3][1] - C[0][3]*C[2][1]*C[3][2] + C[0][1]*C[2][3]*C[3][2] + C[0][2]*C[2][1]*C[3][3] - C[0][1]*C[2][2]*C[3][3]);
	dst->c[0][2] = d * (C[0][2]*C[1][3]*C[3][1] - C[0][3]*C[1][2]*C[3][1] + C[0][3]*C[1][1]*C[3][2] - C[0][1]*C[1][3]*C[3][2] - C[0][2]*C[1][1]*C[3][3] + C[0][1]*C[1][2]*C[3][3]);
	dst->c[0][3] = d * (C[0][3]*C[1][2]*C[2][1] - C[0][2]*C[1][3]*C[2][1] - C[0][3]*C[1][1]*C[2][2] + C[0][1]*C[1][3]*C[2][2] + C[0][2]*C[1][1]*C[2][3] - C[0][1]*C[1][2]*C[2][3]);
	dst->c[1][0] = d * (C[1][3]*C[2][2]*C[3][0] - C[1][2]*C[2][3]*C[3][0] - C[1][3]*C[2][0]*C[3][2] + C[1][0]*C[2][3]*C[3][2] + C[1][2]*C[2][0]*C[3][3] - C[1][0]*C[2][2]*C[3][3]);
	dst->c[1][1] = d * (C[0][2]*C[2][3]*C[3][0] - C[0][3]*C[2][2]*C[3][0] + C[0][3]*C[2][0]*C[3][2] - C[0][0]*C[2][3]*C[3][2] - C[0][2]*C[2][0]*C[3][3] + C[0][0]*C[2][2]*C[3][3]);
	dst->c[1][2] = d * (C[0][3]*C[1][2]*C[3][0] - C[0][2]*C[1][3]*C[3][0] - C[0][3]*C[1][0]*C[3][2] + C[0][0]*C[1][3]*C[3][2] + C[0][2]*C[1][0]*C[3][3] - C[0][0]*C[1][2]*C[3][3]);
	dst->c[1][3] = d * (C[0][2]*C[1][3]*C[2][0] - C[0][3]*C[1][2]*C[2][0] + C[0][3]*C[1][0]*C[2][2] - C[0][0]*C[1][3]*C[2][2] - C[0][2]*C[1][0]*C[2][3] + C[0][0]*C[1][2]*C[2][3]);
	dst->c[2][0] = d * (C[1][1]*C[2][3]*C[3][0] - C[1][3]*C[2][1]*C[3][0] + C[1][3]*C[2][0]*C[3][1] - C[1][0]*C[2][3]*C[3][1] - C[1][1]*C[2][0]*C[3][3] + C[1][0]*C[2][1]*C[3][3]);
	dst->c[2][1] = d * (C[0][3]*C[2][1]*C[3][0] - C[0][1]*C[2][3]*C[3][0] - C[0][3]*C[2][0]*C[3][1] + C[0][0]*C[2][3]*C[3][1] + C[0][1]*C[2][0]*C[3][3] - C[0][0]*C[2][1]*C[3][3]);
	dst->c[2][2] = d * (C[0][1]*C[1][3]*C[3][0] - C[0][3]*C[1][1]*C[3][0] + C[0][3]*C[1][0]*C[3][1] - C[0][0]*C[1][3]*C[3][1] - C[0][1]*C[1][0]*C[3][3] + C[0][0]*C[1][1]*C[3][3]);
	dst->c[2][3] = d * (C[0][3]*C[1][1]*C[2][0] - C[0][1]*C[1][3]*C[2][0] - C[0][3]*C[1][0]*C[2][1] + C[0][0]*C[1][3]*C[2][1] + C[0][1]*C[1][0]*C[2][3] - C[0][0]*C[1][1]*C[2][3]);
	dst->c[3][0] = d * (C[1][2]*C[2][1]*C[3][0] - C[1][1]*C[2][2]*C[3][0] - C[1][2]*C[2][0]*C[3][1] + C[1][0]*C[2][2]*C[3][1] + C[1][1]*C[2][0]*C[3][2] - C[1][0]*C[2][1]*C[3][2]);
	dst->c[3][1] = d * (C[0][1]*C[2][2]*C[3][0] - C[0][2]*C[2][1]*C[3][0] + C[0][2]*C[2][0]*C[3][1] - C[0][0]*C[2][2]*C[3][1] - C[0][1]*C[2][0]*C[3][2] + C[0][0]*C[2][1]*C[3][2]);
	dst->c[3][2] = d * (C[0][2]*C[1][1]*C[3][0] - C[0][1]*C[1][2]*C[3][0] - C[0][2]*C[1][0]*C[3][1] + C[0][0]*C[1][2]*C[3][1] + C[0][1]*C[1][0]*C[3][2] - C[0][0]*C[1][1]*C[3][2]);
	dst->c[3][3] = d * (C[0][1]*C[1][2]*C[2][0] - C[0][2]*C[1][1]*C[2][0] + C[0][2]*C[1][0]*C[2][1] - C[0][0]*C[1][2]*C[2][1] - C[0][1]*C[1][0]*C[2][2] + C[0][0]*C[1][1]*C[2][2]);

	return dst;
}

static inline struct sm_vec3 *
sm_mat4_gettrans(const union sm_mat4 *m, struct sm_vec3 *trans) {
	// Getting translation is trivial
	trans->x = C[3][0];
	trans->y = C[3][1];
	trans->z = C[3][2];

	return trans;
}

static inline struct sm_vec3 *
sm_mat4_getscale(const union sm_mat4 *m, struct sm_vec3 *scale) {
	// Scale is length of columns
	scale->x = sqrtf( C[0][0] * C[0][0] + C[0][1] * C[0][1] + C[0][2] * C[0][2] );
	scale->y = sqrtf( C[1][0] * C[1][0] + C[1][1] * C[1][1] + C[1][2] * C[1][2] );
	scale->z = sqrtf( C[2][0] * C[2][0] + C[2][1] * C[2][1] + C[2][2] * C[2][2] );

	return scale;
}

static inline void
sm_mat4_decompose(const union sm_mat4 *m, struct sm_vec3 *trans, struct sm_vec3 *rot, struct sm_vec3 *scale ) {
	sm_mat4_gettrans(m, trans);
	sm_mat4_getscale(m, scale);

	if( scale->x == 0 || scale->y == 0 || scale->z == 0 ) {
		rot->x = 0;
		rot->y = 0;
		rot->z = 0;
		return;
	}

	// Detect negative scale with determinant and flip one arbitrary axis
	if( sm_mat4_determinant(m) < 0) 
		scale->x = -scale->x;

	// Combined rotation matrix YXZ
	//
	// Cos[y]*Cos[z]+Sin[x]*Sin[y]*Sin[z]   Cos[z]*Sin[x]*Sin[y]-Cos[y]*Sin[z]  Cos[x]*Sin[y]	
	// Cos[x]*Sin[z]                        Cos[x]*Cos[z]                       -Sin[x]
	// -Cos[z]*Sin[y]+Cos[y]*Sin[x]*Sin[z]  Cos[y]*Cos[z]*Sin[x]+Sin[y]*Sin[z]  Cos[x]*Cos[y]

	rot->x = asinf( -C[2][1] / scale->z );

	// Special case: Cos[x] == 0 (when Sin[x] is +/-1)
	float f = fabsf( C[2][1] / scale->z );

	if( f > 0.999f && f < 1.001f ) {
		// Pin arbitrarily one of y or z to zero
		// Mathematical equivalent of gimbal lock
		rot->y = 0;

		// Now: Cos[x] = 0, Sin[x] = +/-1, Cos[y] = 1, Sin[y] = 0
		// => m[0][0] = Cos[z] and m[1][0] = Sin[z]
		rot->z = atan2f( -C[1][0] / scale->y, C[0][0] / scale->x );
	} else {
		// Standard case
		rot->y = atan2f( C[2][0] / scale->z, C[2][2] / scale->z );
		rot->z = atan2f( C[0][1] / scale->x, C[1][1] / scale->y );
	}
}

static union sm_mat3*
sm_mat4_to_mat3(union sm_mat3* m3, const union sm_mat4 *m) {
	m3->x[0] = C[0][0]; m3->x[1] = C[0][1]; m3->x[2] = C[0][2];
	m3->x[3] = C[1][0]; m3->x[4] = C[1][1]; m3->x[5] = C[1][2];
	m3->x[6] = C[2][0]; m3->x[7] = C[2][1]; m3->x[8] = C[2][2];
	return m3;
}

#undef C

#endif // spatial_math_matrix_h

#ifdef __cplusplus
}
#endif