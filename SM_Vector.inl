#ifndef _SPATIAL_MATH_VECTOR_INL_
#define _SPATIAL_MATH_VECTOR_INL_

#include <limits>

#include <math.h>
#include <assert.h>

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

//template <typename T>
//Vector2<T>::Vector2(const Vector2& v)
//	: x(v.x), y(v.y)
//{}

template <typename T>
Vector2<T>& Vector2<T>::operator = (const Vector2& v)
{
	x = v.x;
	y = v.y;
	return *this;
}

template <typename T>
void Vector2<T>::Set(T x, T y)
{
	this->x = x;
	this->y = y;
}

template <typename T>
void Vector2<T>::MakeInvalid()
{
	x = y = (std::numeric_limits<T>::max)();
}

template <typename T>
bool Vector2<T>::IsValid() const
{
	return x != (std::numeric_limits<T>::max)()
		|| y != (std::numeric_limits<T>::max)();
}

template <typename T>
bool Vector2<T>::operator == (const Vector2& v) const
{
	return x == v.x && y == v.y;
}

template <typename T>
bool Vector2<T>::operator != (const Vector2& v) const
{
	return !(*this == v);
}

template <typename T>
bool Vector2<T>::operator < (const Vector2& v) const
{
    return x < v.x
        || (x == v.x && y < v.y);
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
void Vector2<T>::operator *= (const Vector2& v)
{
	x *= v.x; y *= v.y;
}

template <typename T>
void Vector2<T>::operator /= (const Vector2& v)
{
	if (v.x == 0) {
		x = FLT_MAX;
	} else {
		x /= v.x;
	}
	if (v.y == 0) {
		y = FLT_MAX;
	} else {
		y /= v.y;
	}
}

template <typename T>
void Vector2<T>::operator *= (T f)
{
	x *= f; y *= f;
}

template <typename T>
void Vector2<T>::operator /= (T f)
{
	x /= f; y /= f;
}

template <typename T>
Vector2<T> Vector2<T>::operator + (const Vector2& v) const
{
	Vector2 ret(*this);
	ret += v;
	return ret;
}

template <typename T>
Vector2<T> Vector2<T>::operator - (const Vector2& v) const
{
	Vector2 ret(*this);
	ret -= v;
	return ret;
}

template <typename T>
Vector2<T> Vector2<T>::operator * (const Vector2& v) const
{
	Vector2 ret(*this);
	ret *= v;
	return ret;
}

template <typename T>
Vector2<T> Vector2<T>::operator / (const Vector2& v) const
{
	Vector2 ret(*this);
	ret /= v;
	return ret;
}

template <typename T>
Vector2<T> Vector2<T>::operator * (T f) const
{
	return Vector2(x * f, y * f);
}

template <typename T>
Vector2<T> Vector2<T>::operator / (T f) const
{
	return Vector2(x / f, y / f);
}

template <typename T>
T Vector2<T>::Length() const
{
	return static_cast<T>(sqrt(LengthSquared()));
}

template <typename T>
T Vector2<T>::LengthSquared() const
{
	return x * x + y * y;
}

template <typename T>
void Vector2<T>::Normalize()
{
	T s = 1.0f / Length();
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

template <typename T>
bool Vector2Cmp::operator () (const Vector2<T>& p0, const Vector2<T>& p1) const
{
	return p0.x < p1.x
		|| (p0.x == p1.x && p0.y < p1.y);
}

template <typename T>
bool Vector2CmpX::operator () (const Vector2<T>& p0, const Vector2<T>& p1) const
{
	return p0.x < p1.x
		|| (p0.x == p1.x && p0.y < p1.y);
}

template <typename T>
bool Vector2CmpY::operator () (const Vector2<T>& p0, const Vector2<T>& p1) const
{
	return p0.y < p1.y
		|| (p0.y == p1.y && p0.x < p1.x);
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
Vector3<T>::Vector3(const T* xyz)
	: x(xyz[0]), y(xyz[1]), z(xyz[2])
{
}

template <typename T>
Vector3<T>::Vector3(T x, T y, T z)
	: x(x), y(y), z(z)
{
}

//template <typename T>
//Vector3<T>::Vector3(const Vector3& v)
//	: x(v.x), y(v.y), z(v.z)
//{
//}

template <typename T>
T Vector3<T>::operator[](size_t i) const
{
	assert(i < 3);
	return xyz[i];
}

template <typename T>
T& Vector3<T>::operator[](size_t i)
{
	assert(i < 3);
	return xyz[i];
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
void Vector3<T>::Set(T x, T y, T z)
{
	this->x = x;
	this->y = y;
	this->z = z;
}

template <typename T>
void Vector3<T>::MakeInvalid()
{
	x = y = z = (std::numeric_limits<T>::max)();
}

template <typename T>
bool Vector3<T>::IsValid() const
{
	return x != (std::numeric_limits<T>::max)()
		|| y != (std::numeric_limits<T>::max)()
		|| z != (std::numeric_limits<T>::max)();
}

template <typename T>
bool Vector3<T>::operator == (const Vector3& v) const
{
	return x == v.x && y == v.y && z == v.z;
}

template <typename T>
bool Vector3<T>::operator != (const Vector3& v) const
{
	return !(*this == v);
}

template <typename T>
bool Vector3<T>::operator < (const Vector3& v) const
{
	return x < v.x
		|| (x == v.x && y < v.y)
		|| (x == v.x && y == v.y && z < v.z);
}

template <typename T>
bool Vector3<T>::operator > (const Vector3& v) const
{
	return x > v.x
		|| (x == v.x && y > v.y)
		|| (x == v.x && y == v.y && z > v.z);
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
void Vector3<T>::operator *= (T f)
{
	x *= f; y *= f; z *= f;
}

template <typename T>
void Vector3<T>::operator /= (T f)
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
Vector3<T> Vector3<T>::operator * (const Vector3& v) const
{
    return Vector3(x * v.x, y * v.y, z * v.z);
}

template <typename T>
Vector3<T> Vector3<T>::operator * (T f) const
{
	return Vector3(x * f, y * f, z * f);
}

template <typename T>
Vector3<T> Vector3<T>::operator / (T f) const
{
	return Vector3(x / f, y / f, z / f);
}

template <typename T>
T Vector3<T>::Length() const
{
	return static_cast<T>(sqrt(LengthSquared()));
}

template <typename T>
T Vector3<T>::LengthSquared() const
{
	return x * x + y * y + z * z;
}

template <typename T>
void Vector3<T>::Normalize()
{
	T s = 1.0f / Length();
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

template <typename T>
bool Vector3Cmp::operator () (const Vector3<T>& p0, const Vector3<T>& p1) const
{
    return p0 < p1;
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

//template <typename T>
//Vector4<T>::Vector4(const Vector4& v)
//	: x(v.x), y(v.y), z(v.z), w(v.w)
//{
//}

template <typename T>
T Vector4<T>::operator[](size_t i) const
{
	assert(i < 4);
	return xyzw[i];
}

template <typename T>
T& Vector4<T>::operator[](size_t i)
{
	assert(i < 4);
	return xyzw[i];
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
bool Vector4<T>::operator == (const Vector4& v) const
{
	return x == v.x && y == v.y && z == v.z && w == v.w;
}

template <typename T>
bool Vector4<T>::operator != (const Vector4& v) const
{
	return !(*this == v);
}

template <typename T>
void Vector4<T>::operator += (const Vector4& v)
{
	x += v.x; y += v.y; z += v.z; w += v.w;
}

template <typename T>
void Vector4<T>::operator -= (const Vector4& v)
{
	x -= v.x; y -= v.y; z -= v.z; w -= v.w;
}

template <typename T>
void Vector4<T>::operator *= (T f)
{
	x *= f; y *= f; z *= f; w *= f;
}

template <typename T>
void Vector4<T>::operator /= (T f)
{
	x /= f; y /= f; z /= f; w /= f;
}

template <typename T>
Vector4<T> Vector4<T>::operator + (const Vector4& v) const
{
	return Vector4(x + v.x, y + v.y, z + v.z, w + z.w);
}

template <typename T>
Vector4<T> Vector4<T>::operator - (const Vector4& v) const
{
	return Vector4(x - v.x, y - v.y, z - v.z, w - v.w);
}

template <typename T>
Vector4<T> Vector4<T>::operator * (T f) const
{
	return Vector4(x * f, y * f, z * f, w * f);
}

template <typename T>
Vector4<T> Vector4<T>::operator / (T f) const
{
	return Vector4(x / f, y / f, z / f, w / f);
}

template <typename T>
T Vector4<T>::Dot(const Vector4& v) const
{
	return x * v.x + y * v.y + z * v.z + w * v.w;
}

}

#endif // _SPATIAL_MATH_VECTOR_INL_