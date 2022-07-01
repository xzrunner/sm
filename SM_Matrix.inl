#ifndef _SPATIAL_MATH_MATRIX_INL_
#define _SPATIAL_MATH_MATRIX_INL_

#include "sm_const.h"
#include "SM_Quaternion.h"
#include "SM_Math.h"

namespace sm
{

/************************************************************************/
/* Matrix2                                                              */
/************************************************************************/

template <typename T>
Matrix2<T>::Matrix2()
{
	c[0][0] = 1; c[0][1] = 0;
	c[1][0] = 0; c[1][1] = 1;
}

template <typename T>
bool Matrix2<T>::operator == (const Matrix2<T>& b) const
{
	return memcmp(x, b.x, sizeof(x)) == 0;
}

template <typename T>
bool Matrix2<T>::operator != (const Matrix2<T>& b) const
{
	return !(*this == b);
}

/************************************************************************/
/* Matrix3                                                              */
/************************************************************************/

template <typename T>
Matrix3<T>::Matrix3()
{
	c[0][0] = 1; c[0][1] = 0; c[0][2] = 0;
	c[1][0] = 0; c[1][1] = 1; c[1][2] = 0;
	c[2][0] = 0; c[2][1] = 0; c[2][2] = 1;
}

template <typename T>
Matrix3<T>::Matrix3(const Matrix4<T>& m)
{
    c[0][0] = m.c[0][0]; c[1][0] = m.c[1][0]; c[2][0] = m.c[2][0];
    c[0][1] = m.c[0][1]; c[1][1] = m.c[1][1]; c[2][1] = m.c[2][1];
    c[0][2] = m.c[0][2]; c[1][2] = m.c[1][2]; c[2][2] = m.c[2][2];
}

template <typename T>
Matrix3<T>::Matrix3(const Vector3<T>& x, const Vector3<T>& y, const Vector3<T>& z)
{
	c[0][0] = x.x; c[0][1] = x.y; c[0][2] = x.z;
	c[1][0] = y.x; c[1][1] = y.y; c[1][2] = y.z;
	c[2][0] = z.x; c[2][1] = z.y; c[2][2] = z.z;
}

template <typename T>
bool Matrix3<T>::operator == (const Matrix3<T>& b) const
{
    return memcmp(x, b.x, sizeof(x)) == 0;
}

template <typename T>
bool Matrix3<T>::operator != (const Matrix3<T>& b) const
{
    return !(*this == b);
}

template <typename T>
T Matrix3<T>::Determinant() const
{
    return c[0][0] * c[1][1] * c[2][2] +
           c[0][1] * c[1][2] * c[2][0] +
           c[0][2] * c[1][0] * c[2][1] -
           c[0][2] * c[1][1] * c[2][0] -
           c[0][1] * c[1][0] * c[2][2] -
           c[0][0] * c[1][2] * c[2][1];
}

/************************************************************************/
/* Matrix4                                                              */
/************************************************************************/

template <typename T>
Matrix4<T>::Matrix4()
{
	Identity();
}

template <typename T>
Matrix4<T>::Matrix4(const Matrix3<T>& m)
{
	c[0][0] = m.c[0][0]; c[0][1] = m.c[0][1]; c[0][2] = m.c[0][2]; c[0][3] = 0;
	c[1][0] = m.c[1][0]; c[1][1] = m.c[1][1]; c[1][2] = m.c[1][2]; c[1][3] = 0;
	c[2][0] = m.c[2][0]; c[2][1] = m.c[2][1]; c[2][2] = m.c[2][2]; c[2][3] = 0;
	c[3][0] = 0;         c[3][1] = 0;         c[3][2] = 0;         c[3][3] = 1;
}

template <typename T>
Matrix4<T>::Matrix4(const QuaternionT<T>& q)
{
	// Calculate coefficients
	T x2 = q.x + q.x, y2 = q.y + q.y, z2 = q.z + q.z;
	T xx = q.x * x2,  xy = q.x * y2,  xz = q.x * z2;
	T yy = q.y * y2,  yz = q.y * z2,  zz = q.z * z2;
	T wx = q.w * x2,  wy = q.w * y2,  wz = q.w * z2;

	c[0][0] = 1 - (yy + zz);  c[1][0] = xy - wz;
	c[2][0] = xz + wy;        c[3][0] = 0;
	c[0][1] = xy + wz;        c[1][1] = 1 - (xx + zz);
	c[2][1] = yz - wx;        c[3][1] = 0;
	c[0][2] = xz - wy;        c[1][2] = yz + wx;
	c[2][2] = 1 - (xx + yy);  c[3][2] = 0;
	c[0][3] = 0;              c[1][3] = 0;
	c[2][3] = 0;              c[3][3] = 1;
}

template <typename T>
Matrix4<T>& Matrix4<T>::operator = (const Matrix4<T>& b)
{
    memcpy(&x[0], &b.x[0], sizeof(x));
    return *this;
}

template <typename T>
bool Matrix4<T>::operator == (const Matrix4<T>& b) const
{
	return memcmp(x, b.x, sizeof(x)) == 0;
}

template <typename T>
bool Matrix4<T>::operator != (const Matrix4<T>& b) const
{
	return !(*this == b);
}

template <typename T>
Matrix4<T> Matrix4<T>::operator * (const Matrix4<T>& m) const
{
	Matrix4 mf;

	mf.x[0] = x[0] * m.x[0] + x[4] * m.x[1] + x[8] * m.x[2] + x[12] * m.x[3];
	mf.x[1] = x[1] * m.x[0] + x[5] * m.x[1] + x[9] * m.x[2] + x[13] * m.x[3];
	mf.x[2] = x[2] * m.x[0] + x[6] * m.x[1] + x[10] * m.x[2] + x[14] * m.x[3];
	mf.x[3] = x[3] * m.x[0] + x[7] * m.x[1] + x[11] * m.x[2] + x[15] * m.x[3];

	mf.x[4] = x[0] * m.x[4] + x[4] * m.x[5] + x[8] * m.x[6] + x[12] * m.x[7];
	mf.x[5] = x[1] * m.x[4] + x[5] * m.x[5] + x[9] * m.x[6] + x[13] * m.x[7];
	mf.x[6] = x[2] * m.x[4] + x[6] * m.x[5] + x[10] * m.x[6] + x[14] * m.x[7];
	mf.x[7] = x[3] * m.x[4] + x[7] * m.x[5] + x[11] * m.x[6] + x[15] * m.x[7];

	mf.x[8] = x[0] * m.x[8] + x[4] * m.x[9] + x[8] * m.x[10] + x[12] * m.x[11];
	mf.x[9] = x[1] * m.x[8] + x[5] * m.x[9] + x[9] * m.x[10] + x[13] * m.x[11];
	mf.x[10] = x[2] * m.x[8] + x[6] * m.x[9] + x[10] * m.x[10] + x[14] * m.x[11];
	mf.x[11] = x[3] * m.x[8] + x[7] * m.x[9] + x[11] * m.x[10] + x[15] * m.x[11];

	mf.x[12] = x[0] * m.x[12] + x[4] * m.x[13] + x[8] * m.x[14] + x[12] * m.x[15];
	mf.x[13] = x[1] * m.x[12] + x[5] * m.x[13] + x[9] * m.x[14] + x[13] * m.x[15];
	mf.x[14] = x[2] * m.x[12] + x[6] * m.x[13] + x[10] * m.x[14] + x[14] * m.x[15];
	mf.x[15] = x[3] * m.x[12] + x[7] * m.x[13] + x[11] * m.x[14] + x[15] * m.x[15];

	return mf;
}

template <typename T>
Vector2<T> Matrix4<T>::operator * (const Vector2<T>& v) const
{
	T x = c[0][0] * v.x + c[1][0] * v.y + c[3][0];
	T y = c[0][1] * v.x + c[1][1] * v.y + c[3][1];
	return Vector2<T>(x, y);
}

template <typename T>
Vector3<T> Matrix4<T>::operator * (const Vector3<T>& v) const
{
	T x = c[0][0] * v.x + c[1][0] * v.y + c[2][0] * v.z + c[3][0];
	T y = c[0][1] * v.x + c[1][1] * v.y + c[2][1] * v.z + c[3][1];
	T z = c[0][2] * v.x + c[1][2] * v.y + c[2][2] * v.z + c[3][2];
	T w = c[0][3] * v.x + c[1][3] * v.y + c[2][3] * v.z + c[3][3];
	return Vector3<T>(x / w, y / w, z / w);
}

template <typename T>
Vector4<T> Matrix4<T>::operator * (const Vector4<T>& v) const
{
	T x = c[0][0] * v.x + c[1][0] * v.y + c[2][0] * v.z + c[3][0] * v.w;
	T y = c[0][1] * v.x + c[1][1] * v.y + c[2][1] * v.z + c[3][1] * v.w;
	T z = c[0][2] * v.x + c[1][2] * v.y + c[2][2] * v.z + c[3][2] * v.w;
	T w = c[0][3] * v.x + c[1][3] * v.y + c[2][3] * v.z + c[3][3] * v.w;
	return Vector4<T>(x, y, z, w);
}

template <typename T>
void Matrix4<T>::Identity()
{
	c[0][0] = 1; c[0][1] = 0; c[0][2] = 0; c[0][3] = 0;
	c[1][0] = 0; c[1][1] = 1; c[1][2] = 0; c[1][3] = 0;
	c[2][0] = 0; c[2][1] = 0; c[2][2] = 1; c[2][3] = 0;
	c[3][0] = 0; c[3][1] = 0; c[3][2] = 0; c[3][3] = 1;
}

template <typename T>
void Matrix4<T>::Translate(T x, T y, T z)
{
    *this = Translated(x, y, z) * *this;
}

template <typename T>
void Matrix4<T>::Rotate(T x, T y, T z)
{
	*this = Rotated(x, y, z) * *this;
}

template <typename T>
void Matrix4<T>::Scale(T x, T y, T z)
{
    *this = Scaled(x, y, z) * *this;
}

template <typename T>
void Matrix4<T>::Shear(T kx, T ky)
{
    *this = Sheared(kx, ky) * *this;
}

template <typename T>
void Matrix4<T>::RotateZ(T degrees)
{
    *this = RotatedZ(degrees) * *this;
}

template <typename T>
void Matrix4<T>::SetTransformation(T x, T y, T angle, T sx, T sy, T ox, T oy, T kx, T ky)
{
	Identity();
	T c = cos(angle), s = sin(angle);
	// matrix multiplication carried out on paper:
	// |1     x| |c -s    | |sx       | | 1 ky    | |1     -ox|
	// |  1   y| |s  c    | |   sy    | |kx  1    | |  1   -oy|
	// |    1  | |     1  | |      1  | |      1  | |    1    |
	// |      1| |       1| |        1| |        1| |       1 |
	//   move      rotate      scale       skew       origin
	this->x[10] = this->x[15] = 1.0f;
	this->x[0]  = c * sx - ky * s * sy; // = a
	this->x[1]  = s * sx + ky * c * sy; // = b
	this->x[4]  = kx * c * sx - s * sy; // = c
	this->x[5]  = kx * s * sx + c * sy; // = d
	this->x[12] = x - ox * this->x[0] - oy * this->x[4];
	this->x[13] = y - ox * this->x[1] - oy * this->x[5];
}

template <typename T>
void Matrix4<T>::SetTransformation(const Vector3<T>& scale,
	                               const Vector3<T>& rotation_origin,
	                               const Vector4<T>& rotation_quaternion,
	                               const Vector3<T>& translation)
{
	Identity();

	// scale
	c[0][0] = scale.x;
	c[1][1] = scale.x;
	c[2][2] = scale.x;

	// rotate
	c[3][0] -= rotation_origin.x;
	c[3][1] -= rotation_origin.y;
	c[3][2] -= rotation_origin.z;

    *this = (Matrix4<T>(QuaternionT<T>(
        rotation_quaternion.x,
        rotation_quaternion.y,
        rotation_quaternion.z,
        rotation_quaternion.w
        ))) * *this;

	c[3][0] += rotation_origin.x;
	c[3][1] += rotation_origin.y;
	c[3][2] += rotation_origin.z;

	// translate
	c[3][0] += translation.x;
	c[3][1] += translation.y;
	c[3][2] += translation.z;
}

template <typename T>
Matrix4<T> Matrix4<T>::FastMul43(const Matrix4<T>& b)
{
	// Note: m may not be the same as m1 or m2

	Matrix4 m;

	m.x[0] = x[0] * b.x[0] + x[4] * b.x[1] + x[8] * b.x[2];
	m.x[1] = x[1] * b.x[0] + x[5] * b.x[1] + x[9] * b.x[2];
	m.x[2] = x[2] * b.x[0] + x[6] * b.x[1] + x[10] * b.x[2];
	m.x[3] = 0.0f;

	m.x[4] = x[0] * b.x[4] + x[4] * b.x[5] + x[8] * b.x[6];
	m.x[5] = x[1] * b.x[4] + x[5] * b.x[5] + x[9] * b.x[6];
	m.x[6] = x[2] * b.x[4] + x[6] * b.x[5] + x[10] * b.x[6];
	m.x[7] = 0.0f;

	m.x[8] = x[0] * b.x[8] + x[4] * b.x[9] + x[8] * b.x[10];
	m.x[9] = x[1] * b.x[8] + x[5] * b.x[9] + x[9] * b.x[10];
	m.x[10] = x[2] * b.x[8] + x[6] * b.x[9] + x[10] * b.x[10];
	m.x[11] = 0.0f;

	m.x[12] = x[0] * b.x[12] + x[4] * b.x[13] + x[8] * b.x[14] + x[12] * b.x[15];
	m.x[13] = x[1] * b.x[12] + x[5] * b.x[13] + x[9] * b.x[14] + x[13] * b.x[15];
	m.x[14] = x[2] * b.x[12] + x[6] * b.x[13] + x[10] * b.x[14] + x[14] * b.x[15];
	m.x[15] = 1.0f;

	return m;
}

template <typename T>
Matrix4<T> Matrix4<T>::Transposed() const
{
    Matrix4<T> dst(*this);
	for (int y = 0; y < 4; ++y ) {
		for(int x = y + 1; x < 4; ++x ) {
			T tmp = dst.c[x][y];
            dst.c[x][y] = dst.c[y][x];
            dst.c[y][x] = tmp;
		}
	}
    return dst;
}

template <typename T>
T Matrix4<T>::Determinant() const
{
	return
		c[0][3]*c[1][2]*c[2][1]*c[3][0] - c[0][2]*c[1][3]*c[2][1]*c[3][0] - c[0][3]*c[1][1]*c[2][2]*c[3][0] + c[0][1]*c[1][3]*c[2][2]*c[3][0] +
		c[0][2]*c[1][1]*c[2][3]*c[3][0] - c[0][1]*c[1][2]*c[2][3]*c[3][0] - c[0][3]*c[1][2]*c[2][0]*c[3][1] + c[0][2]*c[1][3]*c[2][0]*c[3][1] +
		c[0][3]*c[1][0]*c[2][2]*c[3][1] - c[0][0]*c[1][3]*c[2][2]*c[3][1] - c[0][2]*c[1][0]*c[2][3]*c[3][1] + c[0][0]*c[1][2]*c[2][3]*c[3][1] +
		c[0][3]*c[1][1]*c[2][0]*c[3][2] - c[0][1]*c[1][3]*c[2][0]*c[3][2] - c[0][3]*c[1][0]*c[2][1]*c[3][2] + c[0][0]*c[1][3]*c[2][1]*c[3][2] +
		c[0][1]*c[1][0]*c[2][3]*c[3][2] - c[0][0]*c[1][1]*c[2][3]*c[3][2] - c[0][2]*c[1][1]*c[2][0]*c[3][3] + c[0][1]*c[1][2]*c[2][0]*c[3][3] +
		c[0][2]*c[1][0]*c[2][1]*c[3][3] - c[0][0]*c[1][2]*c[2][1]*c[3][3] - c[0][1]*c[1][0]*c[2][2]*c[3][3] + c[0][0]*c[1][1]*c[2][2]*c[3][3];
}

template <typename T>
Matrix4<T> Matrix4<T>::Inverted() const
{
	Matrix4<T> dst;

	T d = Determinant();
	if( d == 0 ) {
		return dst;
	}
	d = 1.0f / d;

	dst.c[0][0] = d * (c[1][2]*c[2][3]*c[3][1] - c[1][3]*c[2][2]*c[3][1] + c[1][3]*c[2][1]*c[3][2] - c[1][1]*c[2][3]*c[3][2] - c[1][2]*c[2][1]*c[3][3] + c[1][1]*c[2][2]*c[3][3]);
	dst.c[0][1] = d * (c[0][3]*c[2][2]*c[3][1] - c[0][2]*c[2][3]*c[3][1] - c[0][3]*c[2][1]*c[3][2] + c[0][1]*c[2][3]*c[3][2] + c[0][2]*c[2][1]*c[3][3] - c[0][1]*c[2][2]*c[3][3]);
	dst.c[0][2] = d * (c[0][2]*c[1][3]*c[3][1] - c[0][3]*c[1][2]*c[3][1] + c[0][3]*c[1][1]*c[3][2] - c[0][1]*c[1][3]*c[3][2] - c[0][2]*c[1][1]*c[3][3] + c[0][1]*c[1][2]*c[3][3]);
	dst.c[0][3] = d * (c[0][3]*c[1][2]*c[2][1] - c[0][2]*c[1][3]*c[2][1] - c[0][3]*c[1][1]*c[2][2] + c[0][1]*c[1][3]*c[2][2] + c[0][2]*c[1][1]*c[2][3] - c[0][1]*c[1][2]*c[2][3]);
	dst.c[1][0] = d * (c[1][3]*c[2][2]*c[3][0] - c[1][2]*c[2][3]*c[3][0] - c[1][3]*c[2][0]*c[3][2] + c[1][0]*c[2][3]*c[3][2] + c[1][2]*c[2][0]*c[3][3] - c[1][0]*c[2][2]*c[3][3]);
	dst.c[1][1] = d * (c[0][2]*c[2][3]*c[3][0] - c[0][3]*c[2][2]*c[3][0] + c[0][3]*c[2][0]*c[3][2] - c[0][0]*c[2][3]*c[3][2] - c[0][2]*c[2][0]*c[3][3] + c[0][0]*c[2][2]*c[3][3]);
	dst.c[1][2] = d * (c[0][3]*c[1][2]*c[3][0] - c[0][2]*c[1][3]*c[3][0] - c[0][3]*c[1][0]*c[3][2] + c[0][0]*c[1][3]*c[3][2] + c[0][2]*c[1][0]*c[3][3] - c[0][0]*c[1][2]*c[3][3]);
	dst.c[1][3] = d * (c[0][2]*c[1][3]*c[2][0] - c[0][3]*c[1][2]*c[2][0] + c[0][3]*c[1][0]*c[2][2] - c[0][0]*c[1][3]*c[2][2] - c[0][2]*c[1][0]*c[2][3] + c[0][0]*c[1][2]*c[2][3]);
	dst.c[2][0] = d * (c[1][1]*c[2][3]*c[3][0] - c[1][3]*c[2][1]*c[3][0] + c[1][3]*c[2][0]*c[3][1] - c[1][0]*c[2][3]*c[3][1] - c[1][1]*c[2][0]*c[3][3] + c[1][0]*c[2][1]*c[3][3]);
	dst.c[2][1] = d * (c[0][3]*c[2][1]*c[3][0] - c[0][1]*c[2][3]*c[3][0] - c[0][3]*c[2][0]*c[3][1] + c[0][0]*c[2][3]*c[3][1] + c[0][1]*c[2][0]*c[3][3] - c[0][0]*c[2][1]*c[3][3]);
	dst.c[2][2] = d * (c[0][1]*c[1][3]*c[3][0] - c[0][3]*c[1][1]*c[3][0] + c[0][3]*c[1][0]*c[3][1] - c[0][0]*c[1][3]*c[3][1] - c[0][1]*c[1][0]*c[3][3] + c[0][0]*c[1][1]*c[3][3]);
	dst.c[2][3] = d * (c[0][3]*c[1][1]*c[2][0] - c[0][1]*c[1][3]*c[2][0] - c[0][3]*c[1][0]*c[2][1] + c[0][0]*c[1][3]*c[2][1] + c[0][1]*c[1][0]*c[2][3] - c[0][0]*c[1][1]*c[2][3]);
	dst.c[3][0] = d * (c[1][2]*c[2][1]*c[3][0] - c[1][1]*c[2][2]*c[3][0] - c[1][2]*c[2][0]*c[3][1] + c[1][0]*c[2][2]*c[3][1] + c[1][1]*c[2][0]*c[3][2] - c[1][0]*c[2][1]*c[3][2]);
	dst.c[3][1] = d * (c[0][1]*c[2][2]*c[3][0] - c[0][2]*c[2][1]*c[3][0] + c[0][2]*c[2][0]*c[3][1] - c[0][0]*c[2][2]*c[3][1] - c[0][1]*c[2][0]*c[3][2] + c[0][0]*c[2][1]*c[3][2]);
	dst.c[3][2] = d * (c[0][2]*c[1][1]*c[3][0] - c[0][1]*c[1][2]*c[3][0] - c[0][2]*c[1][0]*c[3][1] + c[0][0]*c[1][2]*c[3][1] + c[0][1]*c[1][0]*c[3][2] - c[0][0]*c[1][1]*c[3][2]);
	dst.c[3][3] = d * (c[0][1]*c[1][2]*c[2][0] - c[0][2]*c[1][1]*c[2][0] + c[0][2]*c[1][0]*c[2][1] - c[0][0]*c[1][2]*c[2][1] - c[0][1]*c[1][0]*c[2][2] + c[0][0]*c[1][1]*c[2][2]);

	return dst;
}

template <typename T>
QuaternionT<T> Matrix4<T>::ToQuaternion() const
{
	QuaternionT<T> q;
	auto& a = this->c;
	float trace = a[0][0] + a[1][1] + a[2][2]; // I removed + 1.0f; see discussion with Ethan
	if( trace > 0 ) {// I changed M_EPSILON to 0
		float s = 0.5f / sqrtf(trace+ 1.0f);
		q.w = 0.25f / s;
		q.x = ( a[1][2] - a[2][1] ) * s;
		q.y = ( a[2][0] - a[0][2] ) * s;
		q.z = ( a[0][1] - a[1][0] ) * s;
	} else {
		if ( a[0][0] > a[1][1] && a[0][0] > a[2][2] ) {
			float s = 2.0f * sqrtf( 1.0f + a[0][0] - a[1][1] - a[2][2]);
			q.w = (a[1][2] - a[2][1] ) / s;
			q.x = 0.25f * s;
			q.y = (a[1][0] + a[0][1] ) / s;
			q.z = (a[2][0] + a[0][2] ) / s;
		} else if (a[1][1] > a[2][2]) {
			float s = 2.0f * sqrtf( 1.0f + a[1][1] - a[0][0] - a[2][2]);
			q.w = (a[2][0] - a[0][2] ) / s;
			q.x = (a[1][0] + a[0][1] ) / s;
			q.y = 0.25f * s;
			q.z = (a[2][1] + a[1][2] ) / s;
		} else {
			float s = 2.0f * sqrtf( 1.0f + a[2][2] - a[0][0] - a[1][1] );
			q.w = (a[0][1] - a[1][0] ) / s;
			q.x = (a[2][0] + a[0][2] ) / s;
			q.y = (a[2][1] + a[1][2] ) / s;
			q.z = 0.25f * s;
		}
	}
	return q;
}

template <typename T>
Vector3<T> Matrix4<T>::GetTranslate() const
{
	return Vector3<T>(c[3][0], c[3][1], c[3][2]);
}

template <typename T>
Vector3<T> Matrix4<T>::GetScale() const
{
	Vector3<T> scale;
	scale.x = sqrtf(c[0][0] * c[0][0] + c[0][1] * c[0][1] + c[0][2] * c[0][2]);
	scale.y = sqrtf(c[1][0] * c[1][0] + c[1][1] * c[1][1] + c[1][2] * c[1][2]);
	scale.z = sqrtf(c[2][0] * c[2][0] + c[2][1] * c[2][1] + c[2][2] * c[2][2]);
	return scale;
}

template <typename T>
void Matrix4<T>::Decompose(Vector3<T>& trans, Vector3<T>& rot, Vector3<T>& scale) const
{
	trans = GetTranslate();
	scale = GetScale();

	if( scale.x == 0 || scale.y == 0 || scale.z == 0 ) {
		rot.x = 0;
		rot.y = 0;
		rot.z = 0;
		return;
	}

	// Detect negative scale with determinant and flip one arbitrary axis
	if(Determinant() < 0) {
		scale.x = -scale.x;
	}

	// Combined rotation matrix YXZ
	//
	// Cos[y]*Cos[z]+Sin[x]*Sin[y]*Sin[z]   Cos[z]*Sin[x]*Sin[y]-Cos[y]*Sin[z]  Cos[x]*Sin[y]
	// Cos[x]*Sin[z]                        Cos[x]*Cos[z]                       -Sin[x]
	// -Cos[z]*Sin[y]+Cos[y]*Sin[x]*Sin[z]  Cos[y]*Cos[z]*Sin[x]+Sin[y]*Sin[z]  Cos[x]*Cos[y]

	rot.x = asinf(-c[2][1] / scale.z);

	// Special case: Cos[x] == 0 (when Sin[x] is +/-1)
	T f = fabsf(c[2][1] / scale.z);

	if(f > 0.999f && f < 1.001f) {
		// Pin arbitrarily one of y or z to zero
		// Mathematical equivalent of gimbal lock
		rot.y = 0;

		// Now: Cos[x] = 0, Sin[x] = +/-1, Cos[y] = 1, Sin[y] = 0
		// => m[0][0] = Cos[z] and m[1][0] = Sin[z]
		rot.z = atan2f(-c[1][0] / scale.y, c[0][0] / scale.x);
	} else {
		// Standard case
		rot.y = atan2f(c[2][0] / scale.z, c[2][2] / scale.z);
		rot.z = atan2f(c[0][1] / scale.x, c[1][1] / scale.y);
	}
}

template <typename T>
Matrix4<T> Matrix4<T>::Translated(T x, T y, T z)
{
	Matrix4 m;
    m.x[12] = x;
    m.x[13] = y;
    m.x[14] = z;
	return m;
}

template <typename T>
Matrix4<T> Matrix4<T>::Scaled(T x, T y, T z)
{
	Matrix4 m;
    m.x[0]  = x;
    m.x[5]  = y;
    m.x[10] = z;
	return m;
}

template <typename T>
Matrix4<T> Matrix4<T>::Rotated(T x, T y, T z)
{
	return Matrix4(QuaternionT<T>(x, y ,z));
}

template <typename T>
Matrix4<T> Matrix4<T>::RotatedX(T degrees)
{
	T radians = degrees * SM_DEG_TO_RAD;
	T s = sin(radians);
	T c = cos(radians);

	Matrix4 m;
    m.x[5] =  c;  m.x[9]  = s;
    m.x[6] = -s;  m.x[10] = c;
	return m;
}

template <typename T>
Matrix4<T> Matrix4<T>::RotatedY(T degrees)
{
	T radians = degrees * SM_DEG_TO_RAD;
	T s = sin(radians);
	T c = cos(radians);

	Matrix4 m;
    m.x[0] = c;  m.x[8]  = -s;
    m.x[2] = s;  m.x[10] =  c;
	return m;
}

template <typename T>
Matrix4<T> Matrix4<T>::RotatedZ(T degrees)
{
	T radians = degrees * SM_DEG_TO_RAD;
	T s = sin(radians);
	T c = cos(radians);

	Matrix4 m;
    m.x[0] =  c;  m.x[4] = s;
    m.x[1] = -s;  m.x[5] = c;
	return m;
}

template <typename T>
Matrix4<T> Matrix4<T>::RotatedAxis(const Vector3<T>& axis, T angle)
{
	T t = sin(angle * 0.5f);
	T x = axis.x * t;
	T y = axis.y * t;
	T z = axis.z * t;
	return Matrix4(QuaternionT<T>(x, y, z, cos(angle * 0.5f)));
}

template <typename T>
Matrix4<T> Matrix4<T>::Sheared(T kx, T ky)
{
	Matrix4 m;
	m.x[1] = ky;
	m.x[4] = kx;
	return m;
}

template <typename T>
Matrix4<T> Matrix4<T>::SkewY(T sx, T sz)
{
	mat4 m;
	m.x[2] = sz;
	m.x[8] = sx;
	return m;
}

template <typename T>
Matrix4<T> Matrix4<T>::Perspective(T fovy, T aspect, T znear, T zfar)
{
	assert(abs(aspect - std::numeric_limits<T>::epsilon()) > static_cast<T>(0));

	Matrix4 m;
	memset(m.x, 0, sizeof(m));

	T const tan_half_fovy = static_cast<T>(tan(fovy / static_cast<T>(2) * SM_DEG_TO_RAD));
	m.c[0][0] = static_cast<T>(1) / (aspect * tan_half_fovy);
	m.c[1][1] = static_cast<T>(1) / (tan_half_fovy);
	m.c[2][2] = - (zfar + znear) / (zfar - znear);
	m.c[2][3] = - static_cast<T>(1);
	m.c[3][2] = -(2 * zfar * znear) / (zfar - znear);

	return m;
}

template <typename T>
Matrix4<T> Matrix4<T>::Orthographic(T l, T r, T b, T t, T n, T f)
{
	Matrix4 m;
	m.x[0] = 2 / (r - l);
	m.x[5] = 2 / (t - b);
	m.x[10] = - 2 / (f - n);
	m.x[12] = -(r + l) / (r - l);
	m.x[13] = -(t + b) / (t - b);
	m.x[14] = -(f + n) / (f - n);
	return m;
}

template <typename T>
Matrix4<T> Matrix4<T>::LookAt(const Vector3<T>& eye, const Vector3<T>& center, const Vector3<T>& up)
{
    auto f = (center - eye).Normalized();
    auto s = f.Cross(up).Normalized();
    auto u = s.Cross(f);

    Matrix4 m;
    m.c[0][0] = s.x;
    m.c[1][0] = s.y;
    m.c[2][0] = s.z;
    m.c[0][1] = u.x;
    m.c[1][1] = u.y;
    m.c[2][1] = u.z;
    m.c[0][2] = -f.x;
    m.c[1][2] = -f.y;
    m.c[2][2] = -f.z;
    m.c[3][0] = -s.Dot(eye);
    m.c[3][1] = -u.Dot(eye);
    m.c[3][2] =  f.Dot(eye);
    return m;
}

}

#endif // _SPATIAL_MATH_MATRIX_INL_