#ifndef _SPATIAL_MATH_MATRIX_2D_H_
#define _SPATIAL_MATH_MATRIX_2D_H_

#include "SM_Matrix.h"

namespace sm
{

class Matrix2D
{
public:
	float x[6];

public:
	Matrix2D();
	Matrix2D(const mat4& mt);
	Matrix2D(const Matrix2D& mt);
	Matrix2D& operator = (const Matrix2D& mt);
	
	vec2 operator * (const vec2& v) const;

	void Identity();

	mat4 ToMat4() const;

}; // Matrix2D

}

#include "SM_Matrix2D.h"

#endif // _SPATIAL_MATH_MATRIX_2D_H_