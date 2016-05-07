#ifndef _SPATIAL_MATH_QUATERNION_H_
#define _SPATIAL_MATH_QUATERNION_H_

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

	void Slerp(const QuaternionT<T>& a, const QuaternionT<T>& b, float t);
	void NSlerp(const QuaternionT<T>& a, const QuaternionT<T>& b, float t);

	void Inverted();

	void Rotate(const QuaternionT<T>& q);
	void Scale(T scale);

private:
	T x;
	T y;
	T z;
	T w;

}; // QuaternionT

typedef QuaternionT<float> Quaternion;

}

#include "SM_Quaternion.inl"

#endif // _SPATIAL_MATH_QUATERNION_H_