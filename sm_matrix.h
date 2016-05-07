#ifndef _SPATIAL_MATH_MATRIX_H_
#define _SPATIAL_MATH_MATRIX_H_

#include "SM_Vector.h"
#include "SM_Quaternion.h"

namespace sm
{

/**
 *  @brief
 *    matrix2
 */
template <typename T>
class Matrix2
{
public:
	union {
		float c[2][2];	// Column major order for OpenGL: c[column][row]
		float x[4];
	};

public:
	Matrix2();
	Matrix2(const T* m);

}; // Matrix2

template <typename T>
class Matrix4;

/**
 *  @brief
 *    matrix3
 */
template <typename T>
class Matrix3
{
public:
	union {
		float c[3][3];	// Column major order for OpenGL: c[column][row]
		float x[9];
	};

public:
	Matrix3();
	Matrix3(const T* m);
	Matrix3(const Matrix4<T>& m);

}; // Matrix3

/**
 *  @brief
 *    matrix4
 */
template <typename T>
class Matrix4
{
public:
	union {
		float c[4][4];	// Column major order for OpenGL: c[column][row]
		float x[16];
	};

public:
	Matrix4();
	Matrix4(const T* m);
	Matrix4(const Matrix3<T>& m);
	Matrix4(const QuaternionT<T>& q);

	Matrix4<T> operator * (const Matrix4<T>& b) const;
	Matrix4<T>& operator *= (const Matrix4<T>& b);

	Vector2<T> operator * (const Vector2<T>& v) const;
	Vector3<T> operator * (const Vector3<T>& v) const;
	Vector4<T> operator * (const Vector4<T>& v) const;

	void Identity();

	void Translate(T x, T y, T z);
	void Scale(T x, T y, T z);

	Matrix4<T> FastMul43(const Matrix4<T>& b);

	void Transposed();
	float Determinant() const;
	Matrix4<T> Inverted();

	Vector3<T> GetTranslate() const;
	Vector3<T> GetScale() const;
	void Decompose(Vector3<T>& trans, Vector3<T>& rot, Vector3<T>& scale) const;

	static Matrix4<T> Translated(T x, T y, T z);
	static Matrix4<T> Scaled(T x, T y, T z);
	static Matrix4<T> Rotated(T x, T y, T z);
	static Matrix4<T> RotatedX(T degrees);
	static Matrix4<T> RotatedY(T degrees);
	static Matrix4<T> RotatedZ(T degrees);
	static Matrix4<T> RotatedAxis(const Vector3<T>& axis, T angle);

	static Matrix4<T> Perspective(T left, T right, T bottom, T top, T near, T far);
	static Matrix4<T> Orthographic(T left, T right, T bottom, T top, T near, T far);

}; // Matrix4

typedef Matrix2<float> mat2;
typedef Matrix3<float> mat3;
typedef Matrix4<float> mat4;

}

#include "SM_Matrix.inl"

#endif // _SPATIAL_MATH_MATRIX_H_