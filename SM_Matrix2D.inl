#include "SM_Matrix2D.h"

namespace sm
{

inline
Matrix2D Matrix2D::operator * (const Matrix2D& b) const
{
	Matrix2D ret;
	float* m = ret.x;
	const float* m1 = x;
	const float* m2 = b.x;
	m[0] = m1[0] * m2[0] + m1[1] * m2[2];
	m[1] = m1[0] * m2[1] + m1[1] * m2[3];
	m[2] = m1[2] * m2[0] + m1[3] * m2[2];
	m[3] = m1[2] * m2[1] + m1[3] * m2[3];
	m[4] = m1[4] * m2[0] + m1[5] * m2[2] + m2[4];
	m[5] = m1[4] * m2[1] + m1[5] * m2[3] + m2[5];
	return ret;
}

inline
vec2 Matrix2D::operator * (const vec2& v) const
{
	vec2 ret;
	ret.x = (v.x * x[0] + v.y * x[2]) + x[4];
	ret.y = (v.x * x[1] + v.y * x[3]) + x[5];
	return ret;
}

inline
void Matrix2D::Identity()
{
	x[0] = 1;
	x[1] = 0;
	x[2] = 0;
	x[3] = 1;
	x[4] = 0;
	x[5] = 0;
}

}