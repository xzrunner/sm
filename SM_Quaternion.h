#ifndef _SPATIAL_MATH_QUATERNION_H_
#define _SPATIAL_MATH_QUATERNION_H_

#include "SM_Vector.h"

namespace sm
{

template <typename T>
class QuaternionT
{
public:
	QuaternionT();
	QuaternionT(T x, T y, T z);
	QuaternionT(T x, T y, T z, T w);

	bool operator == (const QuaternionT<T>& q) const;
	bool operator != (const QuaternionT<T>& q) const;

	QuaternionT<T> operator - () const;

	QuaternionT<T> operator + (const QuaternionT<T>& q) const;
	QuaternionT<T> operator - (const QuaternionT<T>& q) const;
	QuaternionT<T> operator * (const QuaternionT<T>& q) const;

	void Normalize();
	T Dot(const QuaternionT<T>& q) const;

	void Slerp(const QuaternionT<T>& a, const QuaternionT<T>& b, T t);
	void NSlerp(const QuaternionT<T>& a, const QuaternionT<T>& b, T t);

	void Inverted();

	void Rotate(const QuaternionT<T>& q);
	void Scale(T scale);

	QuaternionT<T> Rotated(const QuaternionT<T>& b) const;
	QuaternionT<T> Scaled(T scale) const;

	Vector4<T> ToVector() const;
	//// use Matrix4's QuaternionT ctor
	//Matrix3<T> ToMatrix() const;

	static QuaternionT<T> CreateFromVectors(const Vector3<T>& v0, const Vector3<T>& v1);
	static QuaternionT<T> CreateFromAxisAngle(const Vector3<T>& axis, T radians);
	static QuaternionT<T> CreateFromEulerAngle(T roll, T pitch, T yaw);

	static void TransToEulerAngle(const QuaternionT<T>& q, T& roll, T& pitch, T& yaw);

public:
	T x;
	T y;
	T z;
	T w;

}; // QuaternionT

typedef QuaternionT<float> Quaternion;

}

#include "SM_Quaternion.inl"

#endif // _SPATIAL_MATH_QUATERNION_H_