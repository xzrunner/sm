#include "SM_Matrix2D.h"

#include <string.h>

namespace sm
{

inline
Matrix2D Matrix2D::operator * (const Matrix2D& b) const
{
	Matrix2D ret;
	Mul(*this, b, ret);
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

// static float IDENTITY[6] = {
// 	1, 0, 0, 1, 0, 0
// };
// 
// static bool _is_identity(const Matrix2D& m)
// {
// 	return memcmp(m.x, IDENTITY, sizeof(IDENTITY)) == 0;
// }

//inline
//void Matrix2D::Mul(const Matrix2D& _m0, const Matrix2D& _m1, Matrix2D& out)
//{
//// 	if (_is_identity(_m0)) {
//// 		out = _m1;
//// 		return;
//// 	} else if (_is_identity(_m1)) {
//// 		out = _m0;
//// 		return;
//// 	} else {
//		float* m = out.x;
//		const float* m0 = _m0.x;
//		const float* m1 = _m1.x;
//		m[0] = m0[0] * m1[0] + m0[1] * m1[2];
//		m[1] = m0[0] * m1[1] + m0[1] * m1[3];
//		m[2] = m0[2] * m1[0] + m0[3] * m1[2];
//		m[3] = m0[2] * m1[1] + m0[3] * m1[3];
//		m[4] = m0[4] * m1[0] + m0[5] * m1[2] + m1[4];
//		m[5] = m0[4] * m1[1] + m0[5] * m1[3] + m1[5];
////	}	
//}

 static float MAT1001[4] = {
 	1, 0, 0, 1
 };
 
 static bool _is_mat1001(const float* m)
 {
 	return memcmp(m, MAT1001, sizeof(MAT1001)) == 0;
 }

inline
void Matrix2D::Mul(const Matrix2D& _m0, const Matrix2D& _m1, Matrix2D& out)
{
	float* m = out.x;
	const float* m0 = _m0.x;
	const float* m1 = _m1.x;

	if (_is_mat1001(m0)) {
		m[0] = m1[0];
		m[1] = m1[1];
		m[2] = m1[2];
		m[3] = m1[3];
		m[4] = m0[4] * m1[0] + m0[5] * m1[2] + m1[4];
		m[5] = m0[4] * m1[1] + m0[5] * m1[3] + m1[5];
	} else if (_is_mat1001(m1)) {
		m[0] = m0[0];
		m[1] = m0[1];
		m[2] = m0[2];
		m[3] = m0[3];
		m[4] = m0[4] + m1[4];
		m[5] = m0[5] + m1[5];
	} else {
		m[0] = m0[0] * m1[0] + m0[1] * m1[2];
		m[1] = m0[0] * m1[1] + m0[1] * m1[3];
		m[2] = m0[2] * m1[0] + m0[3] * m1[2];
		m[3] = m0[2] * m1[1] + m0[3] * m1[3];
		m[4] = m0[4] * m1[0] + m0[5] * m1[2] + m1[4];
		m[5] = m0[4] * m1[1] + m0[5] * m1[3] + m1[5];
	}
}

}