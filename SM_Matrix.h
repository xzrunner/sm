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
    // Column major, c[column][row]
    // | x0 x2 |    | c00 c10 |
    // | x1 x3 |    | c01 c11 |
	union {
		T c[2][2];
		T x[4];
	};

public:
	Matrix2();

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
    // Column major    c[column][row]
    // | x0 x3 x5 |    | c00 c10 c20 |
    // | x1 x4 x7 |    | c01 c11 c21 |
    // | x2 x5 x8 |    | c02 c12 c22 |
	union {
		T c[3][3];
		T x[9];
	};

public:
	Matrix3();
	Matrix3(const Matrix4<T>& m);

    T Determinant() const;

}; // Matrix3

/**
 *  @brief
 *    matrix4
 */
template <typename T>
class Matrix4
{
public:
    // Column major           c[column][row]
    // | x0 x4 x8  x12 |    | c00 c10 c20 c30 |
    // | x1 x5 x9  x13 |    | c01 c11 c21 c31 |
    // | x2 x6 x10 x14 |    | c02 c12 c22 c32 |
    // | x3 x7 x11 x15 |    | c03 c13 c23 c33 |
	union {
		T c[4][4];
		T x[16];
	};

public:
	Matrix4();
	Matrix4(const Matrix3<T>& m);
	Matrix4(const QuaternionT<T>& q);

    Matrix4<T>& operator = (const Matrix4<T>& b);

	bool operator == (const Matrix4<T>& b) const;
	bool operator != (const Matrix4<T>& b) const;

    Matrix4<T> operator * (const Matrix4<T>& b) const;

	Vector2<T> operator * (const Vector2<T>& v) const;
	Vector3<T> operator * (const Vector3<T>& v) const;
	Vector4<T> operator * (const Vector4<T>& v) const;

	void Identity();

	void Translate(T x, T y, T z);
	void Scale(T x, T y, T z);
	void Shear(T kx, T ky);
	void RotateZ(T degrees);

	/**
	* @param x The translation along the x-axis.
	* @param y The translation along the y-axis.
	* @param angle The rotation (rad) around the center with offset (ox,oy).
	* @param sx Scale along x-axis.
	* @param sy Scale along y-axis.
	* @param ox The offset for rotation along the x-axis.
	* @param oy The offset for rotation along the y-axis.
	* @param kx Shear along x-axis
	* @param ky Shear along y-axis
	**/
	void SetTransformation(T x, T y, T angle, T sx, T sy, T ox, T oy, T kx, T ky);

	void SetTransformation(const Vector3<T>& scale, const Vector3<T>& rotation_origin,
		const Vector4<T>& rotation_quaternion, const Vector3<T>& translation);

	Matrix4<T> FastMul43(const Matrix4<T>& b);

    Matrix4<T> Transposed() const;
	T Determinant() const;
	Matrix4<T> Inverted() const;

	QuaternionT<T> ToQuaternion() const;

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
	static Matrix4<T> Sheared(T kx, T ky);

    // left handed, -1 to 1
	static Matrix4<T> Perspective(T left, T right, T bottom, T top, T znear, T zfar);
	static Matrix4<T> Perspective(T fovy, T aspect, T znear, T zfar);
	static Matrix4<T> Orthographic(T left, T right, T bottom, T top, T znear, T zfar);
    static Matrix4<T> LookAt(const Vector3<T>& eye, const Vector3<T>& center, const Vector3<T>& up);

}; // Matrix4

typedef Matrix2<float> mat2;
typedef Matrix3<float> mat3;
typedef Matrix4<float> mat4;

}

#include "SM_Matrix.inl"

#endif // _SPATIAL_MATH_MATRIX_H_