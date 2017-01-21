#include "sm_const.h"

namespace sm
{

template <typename T>
QuaternionT<T>::QuaternionT()
	: x(0)
	, y(0)
	, z(0)
	, w(1)
{
}

template <typename T>
QuaternionT<T>::QuaternionT(T x, T y, T z)
{
	QuaternionT<T> roll(sinf( x * 0.5f ), 0, 0, cosf( x * 0.5f )),
		           pitch(0, sinf( y * 0.5f ), 0, cosf( y * 0.5f )),
				   yaw(0, 0, sinf( z * 0.5f ), cosf( z * 0.5f ));
	// Order: y * x * z
	*this = pitch * roll * yaw;
}

template <typename T>
QuaternionT<T>::QuaternionT(T x, T y, T z, T w)
	: x(x)
	, y(y)
	, z(z)
	, w(w)
{
}

template <typename T>
bool QuaternionT<T>::operator == (const QuaternionT<T>& q) const
{
	return x == q.x && y == q.y && z == q.z && w == q.w;
}

template <typename T>
bool QuaternionT<T>::operator != (const QuaternionT<T>& q) const
{
	return !(*this == q);
}

template <typename T>
QuaternionT<T> QuaternionT<T>::operator - () const
{
	return QuaternionT<T>(-x, -y, -z, -w);
}

template <typename T>
QuaternionT<T> QuaternionT<T>::operator + (const QuaternionT<T>& q) const
{
	return QuaternionT<T>(x + q.x, y + q.y, z + q.z, w + q.w);
}

template <typename T>
QuaternionT<T> QuaternionT<T>::operator - (const QuaternionT<T>& q) const
{
	return QuaternionT<T>(x - q.x, y - q.y, z - q.z, w - q.w);
}

template <typename T>
QuaternionT<T> QuaternionT<T>::operator * (const QuaternionT<T>& q) const
{
	T x = y * q.z - z * q.y + q.x * w + x * q.w;
	T y = z * q.x - x * q.z + q.y * w + y * q.w;
	T z = x * q.y - y * q.x + q.z * w + z * q.w;
	T w = w * q.w - (x * q.x + y * q.y + z * q.z);
	return QuaternionT<T>(x, y, z, w);
}

template <typename T>
void QuaternionT<T>::Normalize()
{
	*this = Scaled(1 / sqrt(Dot(*this)));
}

template <typename T>
T QuaternionT<T>::Dot(const QuaternionT<T>& q) const
{
	return x * q.x + y * q.y + z * q.z + w * q.w;
}

template <typename T>
void QuaternionT<T>::Slerp(const QuaternionT<T>& a, const QuaternionT<T>& b, T t)
{
	T cos_theta = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
	if (cos_theta < 0) {
		cos_theta = -cos_theta; 
		x = -b.x; y = -b.y;
		z = -b.z; w = -b.w;
	} else {
		*this = *b;
	}
	T scale0 = 1 - t, scale1 = t;
	if( (1 - cos_theta) > 0.001f ) {
		// use spherical interpolation
		T theta = acosf( cos_theta );
		T sin_theta = sinf( theta );
		scale0 = sinf( (1 - t) * theta ) / sin_theta;
		scale1 = sinf( t * theta ) / sin_theta;
	}

	x = a.x * scale0 + x * scale1;
	y = a.y * scale0 + y * scale1;
	z = a.z * scale0 + z * scale1;
	w = a.w * scale0 + w * scale1;

}

template <typename T>
void QuaternionT<T>::NSlerp(const QuaternionT<T>& a, const QuaternionT<T>& b, T t)
{
	// Normalized linear sm_quaternion interpolation
	// Note: NLERP is faster than SLERP and commutative but does not yield constant velocity

	T cos_theta = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;

	if( cos_theta < 0 ) {
		x = a.x + (-b.x - a.x) * t;
		y = a.y + (-b.y - a.y) * t;
		z = a.z + (-b.z - a.z) * t;
		w = a.w + (-b.w - a.w) * t;
	} else {
		x = a.x + (b.x - a.x) * t;
		y = a.y + (b.y - a.y) * t;
		z = a.z + (b.z - a.z) * t;
		w = a.w + (b.w - a.w) * t;
	}

	T inv_len = 1.0f / sqrtf( x * x + y * y + z * z + w * w );
	x *= inv_len;
	y *= inv_len;
	z *= inv_len;
	w *= inv_len;
}

template <typename T>
void QuaternionT<T>::Inverted()
{
	T len = x * x + y * y + z * z + w * w;
	if( len > 0 ) 
	{
		T invLen = - 1.0f / len;
		x *= invLen;
		y *= invLen;
		z *= invLen;
		w *= invLen;
		w = -w;
	} 
	else 
	{
		x = y = z = w = 0;
	}
}

template <typename T>
void QuaternionT<T>::Rotate(const QuaternionT<T>& q)
{
	w = w * q.w - x * q.x - y * q.y - z * q.z;
	x = w * q.x + x * q.w + y * q.z - z * q.y;
	y = w * q.y + y * q.w + z * q.x - x * q.z;
	z = w * q.z + z * q.w + x * q.y - y * q.x;
	Normalize();
}

template <typename T>
void QuaternionT<T>::Scale(T scale)
{

}

template <typename T>
inline QuaternionT<T> QuaternionT<T>::Rotated(const QuaternionT<T>& b) const
{
	QuaternionT<T> q;
	q.w = w * b.w - x * b.x - y * b.y - z * b.z;
	q.x = w * b.x + x * b.w + y * b.z - z * b.y;
	q.y = w * b.y + y * b.w + z * b.x - x * b.z;
	q.z = w * b.z + z * b.w + x * b.y - y * b.x;
	q.Normalize();
	return q;
}

template <typename T>
inline QuaternionT<T> QuaternionT<T>::Scaled(T s) const
{
	return QuaternionT<T>(x * s, y * s, z * s, w * s);
}

template <typename T>
Vector4<T> QuaternionT<T>::ToVector() const
{
	return Vector4<T>(x, y, z, w);
}

// Compute the quaternion that rotates from a to b, avoiding numerical instability.
// Taken from "The Shortest Arc Quaternion" by Stan Melax in "Game Programming Gems".
template <typename T>
QuaternionT<T> QuaternionT<T>::CreateFromVectors(const Vector3<T>& v0, const Vector3<T>& v1)
{
	if (v0 == -v1) {
		return QuaternionT<T>::CreateFromAxisAngle(vec3(1, 0, 0), SM_PI);
	}

	Vector3<T> c = v0.Cross(v1);
	T d = v0.Dot(v1);
	T s = sqrt((1 + d) * 2);

	QuaternionT<T> q;
	q.x = c.x / s;
	q.y = c.y / s;
	q.z = c.z / s;
	q.w = s / 2.0f;
	return q;
}

template <typename T>
QuaternionT<T>  QuaternionT<T>::CreateFromAxisAngle(const Vector3<T>& axis, T radians)
{
	QuaternionT<T> q;
	q.w = sm_cos(radians / 2);
	q.x = q.y = q.z = sm_sin(radians / 2);
	q.x *= axis.x;
	q.y *= axis.y;
	q.z *= axis.z;
	return q;
}


}