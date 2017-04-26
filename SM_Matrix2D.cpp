#include "SM_Matrix2D.h"
#include "SM_Math.h"

#include <string.h>

namespace sm
{

Matrix2D::Matrix2D()
{
	Identity();
}

Matrix2D::Matrix2D(const Matrix2D& mt)
{
	memcpy(x, mt.x, sizeof(x));
}

Matrix2D& Matrix2D::operator = (const Matrix2D& mt)
{
	memcpy(x, mt.x, sizeof(x));
	return *this;
}

static inline Matrix2D
inverse_scale(const Matrix2D& m) 
{
	Matrix2D ret;
	if (m.x[0] == 0 || m.x[3] == 0) {
		return ret;
	}

	ret.x[0] = 1 / m.x[0];
	ret.x[1] = 0;
	ret.x[2] = 0;
	ret.x[3] = 1 / m.x[3];
	ret.x[4] = - m.x[4] * ret.x[0];
	ret.x[5] = - m.x[5] * ret.x[3];

	return ret;
}

static inline Matrix2D
inverse_rot(const Matrix2D& m) 
{
	Matrix2D ret;
	if (m.x[1] == 0 || m.x[2] == 0) {
		return ret;
	}

	ret.x[0] = 0;
	ret.x[1] = 1 / m.x[2];
	ret.x[2] = 1 / m.x[1];
	ret.x[3] = 0;
	ret.x[4] = - m.x[5] * ret.x[2];
	ret.x[5] = - m.x[4] * ret.x[1];

	return ret;
}

Matrix2D Matrix2D::Inverted() const
{
	if (x[1] == 0 && x[2] == 0) {
		return inverse_scale(*this);
	}
	if (x[0] == 0 && x[3] == 0) {
		return inverse_rot(*this);
	}

	Matrix2D ret;
	int t = x[0] * x[3] - x[1] * x[2] ;
	if (t == 0) {
		return ret;
	}

	ret.x[0] =   x[3] / t;
	ret.x[1] = - x[1] / t;
	ret.x[2] = - x[2] / t;
	ret.x[3] =   x[0] / t;
	ret.x[4] = - (x[4] * ret.x[0] + x[5] * ret.x[2]);
	ret.x[5] = - (x[4] * ret.x[1] + x[5] * ret.x[3]);

	return ret;
}

void Matrix2D::Shear(float kx, float ky)
{
	Matrix2D m = *this;
	x[0] = m.x[0] + m.x[1] * ky;
	x[1] = m.x[0] * kx + m.x[1];
	x[2] = m.x[2] + m.x[3] * ky;
	x[3] = m.x[2] * kx + m.x[3];
	x[4] = m.x[4] + m.x[5] * ky;
	x[5] = m.x[4] * kx + m.x[5];
}

void Matrix2D::Scale(float sx, float sy)
{
	if (sx != 1) {
		x[0] *= sx;
		x[2] *= sx;
		x[4] *= sx;
	}
	if (sy != 1) {
		x[1] *= sy;
		x[3] *= sy;
		x[5] *= sy;
	}
}

void Matrix2D::Rotate(float rotate)
{
	if (rotate == 0) {
		return;
	}

	int cosd = cosf(rotate);
	int sind = sinf(rotate);

	int m0_cosd = x[0] * cosd;
	int m0_sind = x[0] * sind;
	int m1_cosd = x[1] * cosd;
	int m1_sind = x[1] * sind;
	int m2_cosd = x[2] * cosd;
	int m2_sind = x[2] * sind;
	int m3_cosd = x[3] * cosd;
	int m3_sind = x[3] * sind;
	int m4_cosd = x[4] * cosd;
	int m4_sind = x[4] * sind;
	int m5_cosd = x[5] * cosd;
	int m5_sind = x[5] * sind;

	x[0] = m0_cosd - m1_sind;
	x[1] = m0_sind + m1_cosd;
	x[2] = m2_cosd - m3_sind;
	x[3] = m2_sind + m3_cosd;
	x[4] = m4_cosd - m5_sind;
	x[5] = m4_sind + m5_cosd;
}

void Matrix2D::Translate(float _x, float _y)
{
	x[4] += _x;
	x[5] += _y;
}

void Matrix2D::SetTransformation(float _x, float _y, float angle, float sx, float sy, float ox, float oy, float kx, float ky)
{
	float c = sm::cos(angle), s = sm::sin(angle);
	// matrix multiplication carried out on paper:
	// |1   x| |c -s  | |sx     | | 1 ky  | |1   -ox|
	// |  1 y| |s  c  | |   sy  | |kx  1  | |  1 -oy|
	//  move     rotate   scale     skew      origin
	x[0]  = c * sx - ky * s * sy; // = a
	x[1]  = s * sx + ky * c * sy; // = b
	x[2]  = kx * c * sx - s * sy; // = c
	x[3]  = kx * s * sx + c * sy; // = d
	x[4] = _x - ox * x[0] - oy * x[2];
	x[5] = _y - ox * x[1] - oy * x[3];
}

}
