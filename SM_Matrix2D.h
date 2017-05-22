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

}

#include "SM_Matrix2D.inl"

#endif // _SPATIAL_MATH_MATRIX_2D_H_