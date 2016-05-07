#ifndef _SPATIAL_MATH_VECTOR_INL_
#define _SPATIAL_MATH_VECTOR_INL_

#include <math.h>

namespace sm
{

/************************************************************************/
/* Vector2                                                              */
/************************************************************************/

template <typename T>
Vector2<T>::Vector2() 
	: x(0), y(0) 
{}

template <typename T>
Vector2<T>::Vector2(T x, T y) 
	: x(x), y(y) 
{}

template <typename T>
Vector2<T>::Vector2(const Vector2& v) 
	: x(v.x), y(v.y) 
{}

template <typename T>
Vector2<T>& Vector2<T>::operator = (const Vector2& v)
{
	x = v.x;
	y = v.y;
	return *this;
}

template <typename T>
void Vector2<T>::Assign(T x, T y) 
{
	this->x = x;
	this->y = y;
}

template <typename T>
bool Vector2<T>::operator != (const Vector2& v) const
{
	return x != v.x || y != v.y;
}

template <typename T>
bool Vector2<T>::operator == (const Vector2& v) const
{
	return x == v.x && y == v.y;
}

template <typename T>
Vector2<T> Vector2<T>::operator - () const
{
	return Vector2(-x, -y);
}

template <typename T>
void Vector2<T>::operator += (const Vector2& v)
{
	x += v.x; y += v.y;
}

template <typename T>
void Vector2<T>::operator -= (const Vector2& v)
{
	x -= v.x; y -= v.y;
}

template <typename T>
void Vector2<T>::operator *= (float f)
{
	x *= f; y *= f;
}

template <typename T>
void Vector2<T>::operator /= (float f)
{
	x /= f; y /= f;
}

template <typename T>
Vector2<T> Vector2<T>::operator + (const Vector2& v) const
{
	return Vector2(x + v.x, y + v.y);
}

template <typename T>
Vector2<T> Vector2<T>::operator - (const Vector2& v) const
{
	return Vector2(x - v.x, y - v.y);
}

template <typename T>
Vector2<T> Vector2<T>::operator * (float f) const
{
	return Vector2(x * f, y * f);
}

template <typename T>
Vector2<T> Vector2<T>::operator / (float f) const
{
	return Vector2(x / f, y / f);
}

template <typename T>
T Vector2<T>::Length() const
{
	return sqrt(x * x + y * y);
}

template <typename T>
void Vector2<T>::Normalize()
{
	float s = 1.0f / Length();
	x *= s;
	y *= s;
}

template <typename T>
Vector2<T> Vector2<T>::Normalized() const 
{
	Vector2 v = *this;
	v.Normalize();
	return v;
}

template <typename T>
T Vector2<T>::Cross(const Vector2& v) const
{
	return x * v.y - y * v.x;
}

template <typename T>
T Vector2<T>::Dot(const Vector2& v) const
{
	return x * v.x + y * v.y;
}

/************************************************************************/
/* Vector3                                                              */
/************************************************************************/

template <typename T>
Vector3<T>::Vector3()
	: x(0), y(0), z(0)
{
}

template <typename T>
Vector3<T>::Vector3(T x, T y, T z)
	: x(x), y(y), z(z)
{
}

template <typename T>
Vector3<T>::Vector3(const Vector3& v)
	: x(v.x), y(v.y), z(v.z)
{
}

template <typename T>
Vector3<T>& Vector3<T>::operator = (const Vector3& v)
{
	x = v.x;
	y = v.y;
	z = v.z;
	return *this;
}

template <typename T>
void Vector3<T>::Assign(T x, T y, T z) 
{
	this->x = x;
	this->y = y;
	this->z = z;
}

template <typename T>
bool Vector3<T>::operator != (const Vector3& v) const
{
	return x != v.x || y != v.y || z != v.z;
}

template <typename T>
bool Vector3<T>::operator == (const Vector3& v) const
{
	return x == v.x && y == v.y && z == v.z;
}

template <typename T>
Vector3<T> Vector3<T>::operator - () const
{
	return Vector3(-x, -y, -z);
}

template <typename T>
void Vector3<T>::operator += (const Vector3& v)
{
	x += v.x; y += v.y; z += v.z;
}

template <typename T>
void Vector3<T>::operator -= (const Vector3& v)
{
	x -= v.x; y -= v.y; z -= v.z;
}

template <typename T>
void Vector3<T>::operator *= (float f)
{
	x *= f; y *= f; z *= f;
}

template <typename T>
void Vector3<T>::operator /= (float f)
{
	x /= f; y /= f; z /= f;
}

template <typename T>
Vector3<T> Vector3<T>::operator + (const Vector3& v) const
{
	return Vector3(x + v.x, y + v.y, z + v.z);
}

template <typename T>
Vector3<T> Vector3<T>::operator - (const Vector3& v) const
{
	return Vector3(x - v.x, y - v.y, z - v.z);
}

template <typename T>
Vector3<T> Vector3<T>::operator * (float f) const
{
	return Vector3(x * f, y * f, z * f);
}

template <typename T>
Vector3<T> Vector3<T>::operator / (float f) const
{
	return Vector3(x / f, y / f, z / f);
}

template <typename T>
T Vector3<T>::Length() const
{
	return sqrt(x * x + y * y + z * z);
}

template <typename T>
void Vector3<T>::Normalize()
{
	float s = 1.0f / Length();
	x *= s;
	y *= s;
	z *= s;
}

template <typename T>
Vector3<T> Vector3<T>::Normalized() const
{
	Vector3 v = *this;
	v.Normalize();
	return v;
}

template <typename T>
Vector3<T> Vector3<T>::Cross(const Vector3& v) const
{
	return Vector3<T>(
		y * v.z - z * v.y,
		z * v.x - x * v.z,
		x * v.y - y * v.x);
}

template <typename T>
T Vector3<T>::Dot(const Vector3& v) const
{
	return x * v.x + y * v.y + z * v.z;
}

/************************************************************************/
/* Vector4                                                              */
/************************************************************************/

template <typename T>
Vector4<T>::Vector4()
	: x(0), y(0), z(0), w(0)
{
}

template <typename T>
Vector4<T>::Vector4(T x, T y, T z, T w)
	: x(x), y(y), z(z), w(w)
{
}

template <typename T>
Vector4<T>::Vector4(const Vector4& v)
	: x(v.x), y(v.y), z(v.z), w(v.w)
{
}

template <typename T>
Vector4<T>& Vector4<T>::operator = (const Vector4& v)
{
	x = v.x;
	y = v.y;
	z = v.z;
	w = v.w;
	return *this;
}

template <typename T>
void Vector4<T>::Assign(T x, T y, T z, T w) 
{
	this->x = x;
	this->y = y;
	this->z = z;
	this->w = w;
}

template <typename T>
bool Vector4<T>::operator != (const Vector4& v) const
{
	return x != v.x || y != v.y || z != v.z || w != v.w;
}

template <typename T>
bool Vector4<T>::operator == (const Vector4& v) const
{
	return x == v.x && y == v.y && z == v.z && w == v.w;
}

template <typename T>
T Vector4<T>::Dot(const Vector4& v) const
{
	return x * v.x + y * v.y + z * v.z + w * v.w;
}

}

#endif // _SPATIAL_MATH_VECTOR_INL_