#ifndef _SPATIAL_MATH_MATRIX_2D_H_
#define _SPATIAL_MATH_MATRIX_2D_H_

#include "SM_Vector.h"

namespace sm
{

class Matrix2D
{
public:
	float x[6];

public:
	Matrix2D();
	Matrix2D(const Matrix2D& mt);
	Matrix2D& operator = (const Matrix2D& mt);

	Matrix2D operator * (const Matrix2D& b) const;
	vec2 operator * (const vec2& v) const;

	void Identity();

	Matrix2D Inverted() const;

	void Shear(float kx, float ky);
	void Scale(float sx, float sy);
	void Rotate(float angle);
	void Translate(float x, float y);

	void SetTransformation(float x, float y, float angle, float sx, float sy, float ox, float oy, float kx, float ky);

	static void Mul(const Matrix2D& m0, const Matrix2D& m1, Matrix2D& out);

}; // Matrix2D

#define SM_MAT2D_MUL(m0, m1, out)                   \
	out[0] = m0[0] * m1[0] + m0[1] * m1[2];         \
	out[1] = m0[0] * m1[1] + m0[1] * m1[3];         \
	out[2] = m0[2] * m1[0] + m0[3] * m1[2];         \
	out[3] = m0[2] * m1[1] + m0[3] * m1[3];         \
	out[4] = m0[4] * m1[0] + m0[5] * m1[2] + m1[4]; \
	out[5] = m0[4] * m1[1] + m0[5] * m1[3] + m1[5]; \

}

#include "SM_Matrix2D.inl"

#endif // _SPATIAL_MATH_MATRIX_2D_H_